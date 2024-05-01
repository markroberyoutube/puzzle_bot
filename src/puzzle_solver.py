import sys, os, logging, tempfile, math, glob
import cv2 as cv
import numpy as np
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThread

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
    def compute_solution(self, solver_batch_dir):
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
        input_dir = solver_batch_dir
        working_dir = solver_batch_dir
        for d in [PHOTO_BMP_DIR, SEGMENT_DIR, VECTOR_DIR, DEDUPED_DIR, CONNECTIVITY_DIR, SOLUTION_DIR]:
            os.makedirs(os.path.join(working_dir, d), exist_ok=True)
            for f in os.listdir(os.path.join(working_dir, d)):
                os.remove(os.path.join(working_dir, d, f))
                
        piece_id = 1
        for f in os.listdir(input_dir):
            if f.endswith('.jpg') or f.endswith('.jpeg'):
                print(f"{util.YELLOW}### Processing {f} ###{util.WHITE}")
                robot_state = [motor_x, motor_y]  # TODO
                piece_id = process.process_photo(
                    photo_path = os.path.join(input_dir, f), 
                    working_dir = working_dir, 
                    starting_piece_id = piece_id, 
                    robot_state = robot_state
                )

        print("Solving")
        solution = solve.solve(path=args.working_dir)
        
        puzzle_motor_origin = (20000, 50000)
        for piece in solution:
            motor_photo_origin_point, pixel_grip_point, angle, pixel_destination_point = piece
            grip_motor_point = gripper_motor_coordinates(motor_photo_origin_point, pixel_grip_point)
            move_piece(grip_motor_point, angle, motor_coordinates(puzzle_motor_origin, pixel_destination_point))

        # Put the original stdout back in place
        sys.stdout.write = self.old_write

        
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
            self.msleep(100) # Delay 100 ms using QThread's version of sleep

