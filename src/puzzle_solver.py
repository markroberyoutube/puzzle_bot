import sys, os, logging, tempfile, math, glob, json
from queue import Queue
import cv2 as cv
import numpy as np
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThread, QObject
from PyQt5.QtWidgets import QApplication

from utils import estimate_linear_regression_coefficients

# Add Ryan's code to our path
sys.path.append(
    os.path.join(os.path.abspath(os.pardir), "solver_library", "src")
)
import process, solve
from common import util
from common.config import *

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

class PuzzleSolver(QThread):
    """"Puzzle Solver

    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    """
    
    # Define QT Signals for inter-process communication
    trigger_solution_computation = pyqtSignal(str)
    solution_ready = pyqtSignal()
    output_data_ready = pyqtSignal(str)
    
    trigger_move_pieces = pyqtSignal()
    pieces_moved = pyqtSignal()
    
    thread_closing = pyqtSignal() # Emit to this when you want the thread to exit
    
    def __init__(self, parent):
        # Superclass initializer
        QThread.__init__(self, parent)
        
        # Connect signals for inter-process communication
        self.trigger_solution_computation.connect(self.compute_solution)
        self.trigger_move_pieces.connect(self.move_pieces)
        self.thread_closing.connect(self.exit_thread)
        
        # Set to True to cause the Thread's main run loop to exit
        self._thread_exiting = False

    @pyqtSlot(str)
    def compute_solution(self, solver_batch_working_dir):
        # Redirect STDOUT (used by Ryan's puzzle solver code) to emit to a textbox
        # Hijack stdout.write temporarily to intercept print statements
        self.old_write = sys.stdout.write
        def capture_it(data):
            self.old_write(data) # Print the data to the original stdout
            self.output_data_ready.emit(data) # Also emit it to other threads
        sys.stdout.write = capture_it
        
        ##########################################################################################
        # Call Ryan's code
        
        # Prepare working directories
        working_dir = solver_batch_working_dir
        for d in [PHOTO_BMP_DIR, SEGMENT_DIR, VECTOR_DIR, DEDUPED_DIR, CONNECTIVITY_DIR, SOLUTION_DIR]:
            os.makedirs(os.path.join(working_dir, d), exist_ok=True)
            for f in os.listdir(os.path.join(working_dir, d)):
                os.remove(os.path.join(working_dir, d, f))
        
        # Open batch info (prepared by the serpentine photo thread)
        input_dir = os.path.join(working_dir, PHOTOS_DIR)
        batch_info_file = os.path.join(input_dir, "batch.json")
        batch_info = {}
        if not os.path.exists(batch_info_file):
            logging.error(f"[Ui.compute_solution]: batch info file does not exist at {batch_info_file}")
            return
        with open(batch_info_file, "r") as jsonfile:
            batch_info = json.load(jsonfile)
                
        # Process the photos one by one
        piece_id = 1
        for f in os.listdir(input_dir):
            # Handle thread exit in this long running process.
            if self._thread_exiting: return
            # Process the photo
            if f.endswith('.jpg') or f.endswith('.jpeg'):
                print(f"{util.YELLOW}### Processing {f} ###{util.WHITE}")

                # Find the motor position this photo was taken at
                photo_info = [p for p in batch_info['photos'] if p['file_name'] == f][0]
                robot_state = {'photo_at_motor_position': photo_info['position']}
                
                QApplication.processEvents()

                # Process the photo
                piece_id = process.process_photo(
                    photo_path = os.path.join(input_dir, f),
                    working_dir = working_dir,
                    starting_piece_id = piece_id,
                    robot_state = robot_state
                )

        print("Solving")
        solve.solve(path=working_dir)
        
        # Put the original stdout back in place
        sys.stdout.write = self.old_write
        
        # Tell other threads that the solution has been found
        self.solution_ready.emit()
        
    @pyqtSlot()
    def move_pieces():
        pass

    @pyqtSlot()
    def exit_thread(self):
        """Emit to this slot to stop this thread."""
        # Ask the main run loop to exit
        self._thread_exiting = True

    def run(self):
        """Main thread loop"""

        # Currently the run loop does nothing other than sleep
        while (not self._thread_exiting):
            self.msleep(10) # Delay 10 ms using QThread's version of sleep
            QApplication.processEvents()

