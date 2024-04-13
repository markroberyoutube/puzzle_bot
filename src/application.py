#!/usr/bin/env python3

import sys, os, threading, time, signal
import exif, json, math
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, QThread 
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5 import QtTest

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from utils import parse_int
from serial_driver import ClearCore, DummyClearCore
from camera_driver import GalaxyS24

APPLICATION_NAME = "Puzzlin' Pete"

# QT Class inheritance code from
# https://nitratine.net/blog/post/how-to-import-a-pyqt5-ui-file-in-a-python-gui/

# QT Thread code from
# https://gist.github.com/majabojarska/952978eb83bcc19653be138525c4b9da
# https://stackoverflow.com/questions/6783194/background-thread-with-qthread-in-pyqt

class Ui(QMainWindow):
    def __init__(self, argv):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi(os.path.join("qt", "application.ui"), self) # Load the .ui file
        self.setWindowTitle(APPLICATION_NAME) # Set window name (works on MS Windows)
        self.show() # Show the GUI

        # Instance variables
        self.screenshot_dir = None
        self.clearcore = None
        self.serpentine_move_finished = None
        self.serpentine_paused = None
        
        # Open configuration file
        if len(argv) < 2:
            logging.error(f"[Ui.init] Config file not specified in arguments: {argv}")
            self.close()
            return
        self.config_file_path = argv[1]
        if not os.path.exists(self.config_file_path):
            logging.debug(f"[Ui.init] Creating config file at: {self.config_file_path}")
            self.config = {}
            self.write_config()
        
        # Read the configuration file
        self.read_config()
        
        # Setup tabs
        self.setup_move_robot_tab()
        self.setup_serpentine_photos_tab()
    
    def read_config(self):
        """Read self.config from the JSON-formatted config file"""
        with open(self.config_file_path, "r") as jsonfile:
            self.config = json.load(jsonfile)
            self.photo_directory_textbox.setText(self.config.get('photo_directory', ''))
            self.start_x_textbox.setText(self.config.get('start_x', ''))
            self.start_y_textbox.setText(self.config.get('start_y', ''))
            self.start_z_textbox.setText(self.config.get('start_z', ''))
            self.end_x_textbox.setText(self.config.get('end_x', ''))
            self.end_y_textbox.setText(self.config.get('end_y', ''))
            self.end_z_textbox.setText(self.config.get('end_z', ''))
            self.overlap_x_textbox.setText(self.config.get('overlap_x', ''))
            self.overlap_y_textbox.setText(self.config.get('overlap_y', ''))
    
    def write_config(self):
        """Update self.config from input boxes and write it to a JSON-formatted config file"""
        # Update self.config from input boxes
        self.config['photo_directory'] = self.photo_directory_textbox.text()
        self.config['start_x'] = self.start_x_textbox.text()
        self.config['start_y'] = self.start_y_textbox.text()
        self.config['start_z'] = self.start_z_textbox.text()
        self.config['end_x'] = self.end_x_textbox.text()
        self.config['end_y'] = self.end_y_textbox.text()
        self.config['end_z'] = self.end_z_textbox.text()
        self.config['overlap_x'] = self.overlap_x_textbox.text()
        self.config['overlap_y'] = self.overlap_y_textbox.text()
        
        # Write self.config to a JSON-formatted config file
        with open(self.config_file_path, "w") as jsonfile:
            json.dump(self.config, jsonfile)        

    
    def closeEvent(self, event):
        logging.debug(f"[Ui.closeEvent] closing main window")
        
        # Close the ClearCore serial thread
        if self.clearcore is not None:
            self.clearcore.thread_closing.emit() # Tell the thread to exit its run loop
            self.clearcore.wait() # Wait for the thread to close (also closes the serial port)
        
        # Continue closing the app
        event.accept()


    ################################################################################################
    # "MOVE ROBOT" TAB
    
    def setup_move_robot_tab(self):
        """Configure UI on the MOVE ROBOT tab"""
        # Configure ClearCore output to always scroll to the bottom
        scrollbar = self.clearcore_serial_output_textarea.verticalScrollBar()
        scrollbar.rangeChanged.connect(lambda minVal, maxVal: scrollbar.setValue(scrollbar.maximum()))
        
        # Set all inputs as unavailable until we're connected to the ClearCore
        self.set_clearcore_availability(False)
        self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Opening ClearCore...</font><br/>")
        
        # Connect to the ClearCore over serial
        self.clearcore = ClearCore(parent=self)
        #self.clearcore = DummyClearCore(parent=self)
        self.clearcore.line_received.connect(self.process_clearcore_response)
        self.clearcore.start()
        
        # Connect all the buttons (coding this is faster than creating slots in the qt designer gui)
        self.vacuum_on_button.clicked.connect(lambda: self.send_clearcore_command("1"))
        self.vacuum_off_button.clicked.connect(lambda: self.send_clearcore_command("0"))
        self.home_button.clicked.connect(lambda: self.send_clearcore_command("h"))
        self.jog_positive_x_button.clicked.connect(lambda: self.send_clearcore_command("d"))
        self.jog_negative_x_button.clicked.connect(lambda: self.send_clearcore_command("a"))
        self.jog_positive_y_button.clicked.connect(lambda: self.send_clearcore_command("w"))
        self.jog_negative_y_button.clicked.connect(lambda: self.send_clearcore_command("s"))
        self.jog_positive_z_button.clicked.connect(lambda: self.send_clearcore_command("o"))
        self.jog_negative_z_button.clicked.connect(lambda: self.send_clearcore_command("l"))
        @pyqtSlot()
        def move_absolute():
            # Get desired position
            x = parse_int(self.move_x_textbox.text())
            y = parse_int(self.move_y_textbox.text())
            z = parse_int(self.move_z_textbox.text())
            # Send command to move to this absolute position
            self.send_clearcore_command(f"m {x},{y},{z}")
        self.move_absolute_button.clicked.connect(move_absolute)
        
        # Wait 2 seconds for the ClearCore to reset before we make any writes to it
        @pyqtSlot(bool)
        def on_port_open(port_status):
            if (port_status):
                self.set_clearcore_availability(True)
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>ClearCore opened!</font><br/>")
            else:
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Error opening ClearCore.</font><br/>")
        self.clearcore.port_opened.connect(on_port_open)

    @pyqtSlot(str)
    def process_clearcore_response(self, line):
        """Process ClearCore responses."""
        # Process STOP/GO events
        if line.startswith("STOP: Soft Stop"):
            self.soft_stop_label.setText("🔴" + self.soft_stop_label.text()[1:])
        if line.startswith("GO: Soft Stop"):
            self.soft_stop_label.setText("🟢" + self.soft_stop_label.text()[1:])
        if line.startswith("STOP: E-Stop"):
            self.e_stop_label.setText("🔴" + self.e_stop_label.text()[1:])
        if line.startswith("GO: E-Stop"):
            self.e_stop_label.setText("🟢" + self.e_stop_label.text()[1:])
        
        # Process 'p' position responses
        if line.startswith("SUCCESS: printCurrentPosXYZ"):
            # Parse the position from the response
            x, y, z = parse_int(line.split(",")[0]), \
                      parse_int(line.split(",")[1]), \
                      parse_int(line.split(",")[2])
            # Update textboxes with the new position
            self.move_x_textbox.setText(str(x))
            self.move_y_textbox.setText(str(y))
            self.move_z_textbox.setText(str(z))
            # Hide these position responses from the serial output so they don't clog up that textarea
            return
            
        # Format line and add it to the textarea
        html_line = ""
        status = line.split(":")[0]
        message = ":".join(line.split(":")[1:])

        red = "#ff3300"
        green = "#009900"
        if status in ["SUCCESS", "GO"]:
            html_line += f"<font style='color: {green};'>{status}:</font> {message}<br/>"
        else:
            html_line += f"<font style='color: {red};'>{status}:</font> {message}<br/>"
        self.clearcore_serial_output_textarea.insertHtml(html_line)

    def set_clearcore_availability(self, enabled=True):
        """Set the availability of all input buttons related to the ClearCore to enabled or disabled."""        
        self.home_button.setEnabled(enabled)
        self.vacuum_on_button.setEnabled(enabled)
        self.vacuum_off_button.setEnabled(enabled)
        self.jog_positive_x_button.setEnabled(enabled)
        self.jog_negative_x_button.setEnabled(enabled)
        self.jog_positive_y_button.setEnabled(enabled)
        self.jog_negative_y_button.setEnabled(enabled)
        self.jog_positive_z_button.setEnabled(enabled)
        self.jog_negative_z_button.setEnabled(enabled)
        self.move_absolute_button.setEnabled(enabled)
    
    def send_clearcore_command(self, command):
        """Send command to the clearcore serial port, disabling corresponding button inputs until we receive a response."""
        if (not command):
            logging.error(f"[Ui.send_clearcore_command] command parameter missing or empty")
            return
        
        # Disable input buttons until we get a response
        self.set_clearcore_availability(False)
        
        # Setup a response callback
        @pyqtSlot(str)
        def on_response(line):
            # Remove this callback
            self.clearcore.line_received.disconnect(on_response)
            
            # After every command, send a 'p' command to get the current position
            @pyqtSlot(str)
            def on_position_response(line):
                # Remove this callback
                self.clearcore.line_received.disconnect(on_position_response)
                # Finally re-enable the input buttons
                self.set_clearcore_availability(True)
            self.clearcore.line_received.connect(on_position_response)
            self.clearcore.line_ready_to_transmit.emit("p")
        self.clearcore.line_received.connect(on_response)
                
        # Emit the command to the ClearCore QThread, for transmission over the serial port
        self.clearcore.line_ready_to_transmit.emit(command)


    ################################################################################################
    # "SERPENTINE PHOTOS" TAB

    def setup_serpentine_photos_tab(self):
        """Configure UI on the SERPENTINE PHOTOS tab"""
        # Configure all textboxes to auto-save config file upon change
        self.photo_directory_textbox.textChanged.connect(self.write_config)
        self.start_x_textbox.textChanged.connect(self.write_config)
        self.start_y_textbox.textChanged.connect(self.write_config)
        self.start_z_textbox.textChanged.connect(self.write_config)
        self.end_x_textbox.textChanged.connect(self.write_config)
        self.end_y_textbox.textChanged.connect(self.write_config)
        self.end_z_textbox.textChanged.connect(self.write_config)
        self.overlap_x_textbox.textChanged.connect(self.write_config)
        self.overlap_y_textbox.textChanged.connect(self.write_config)
        
        # Configure buttons
        self.photo_directory_browse_button.clicked.connect(self.browse_for_photo_directory)
        self.serpentine_start_button.clicked.connect(self.take_serpentine_photos)
        self.screenshot_button.clicked.connect(self.take_screenshot)
        
        # Start up a thread for the camera and screenshot
        self.camera = GalaxyS24(parent=self)
        self.screenshot_button.clicked.connect(self.take_screenshot)

    def take_serpentine_photos(self):
        """Take photos in a serpentine path given the config parameters"""
        
        # Change the "START" button to "PAUSE"
        start_text = self.serpentine_start_button.text() 
        pause_text = "PAUSE ⏸️"
        self.serpentine_start_button.setText(pause_text)
        self.serpentine_paused = False
        @pyqtSlot()
        def pause_serpentine_photos():
            # When paused, change the "PAUSE" button to a "START" button
            self.serpentine_paused = True
            self.serpentine_start_button.setText(start_text)
            self.serpentine_start_button.clicked.disconnect(pause_serpentine_photos)
            @pyqtSlot()
            def restart_serpentine_photos():
                # When restarted, change the "PAUSE" button back to a "START" button
                self.serpentine_paused = False
                self.serpentine_start_button.setText(pause_text)
                self.serpentine_start_button.clicked.disconnect(restart_serpentine_photos)
                self.serpentine_start_button.clicked.connect(pause_serpentine_photos)    
            self.serpentine_start_button.clicked.connect(restart_serpentine_photos)
        self.serpentine_start_button.clicked.connect(pause_serpentine_photos)
        
        # Make sure the photo directory exists
        photo_directory = self.photo_directory_textbox.text()
        if not os.path.exists(photo_directory) and os.path.isdir(photo_directory):
            logger.error(f"[Ui.take_serpentine_photos] Photo directory does not exist at {photo_directory}")
            return
        
        # Auto-generate the next batch directory (they are numbered sequentially)
        last_batch_number = 0
        sub_dirs = [d for d in os.listdir(photo_directory) if os.path.isdir(os.path.join(photo_directory, d))]
        batch_dirs = [d for d in sub_dirs if d.isnumeric()]
        batch_dirs.sort() # sorts in place
        if batch_dirs:
            last_batch_number = int(batch_dirs[-1])
        current_batch_number = last_batch_number + 1
        batch_dir = os.path.join(photo_directory, str(current_batch_number))
        os.makedirs(batch_dir)
        self.batch_number_textbox.setText(str(current_batch_number))
        
        # Calculate delta movements to satisfy minimum overlap and equal (integer) overlap requirements
        start_x = int(self.start_x_textbox.text())
        start_y = int(self.start_y_textbox.text())
        start_z = int(self.start_z_textbox.text())
        end_x = int(self.end_x_textbox.text())
        end_y = int(self.end_y_textbox.text())
        end_z = int(self.end_z_textbox.text())
        min_x_overlap = int(self.overlap_x_textbox.text())
        min_y_overlap = int(self.overlap_y_textbox.text())
        num_x_passes = math.ceil((end_x - start_x) / min_x_overlap)
        num_y_passes = math.ceil((end_y - start_y) / min_y_overlap)
        total_photos = num_x_passes * num_y_passes
        x_pass_distance = math.floor((end_x - start_x) / num_x_passes)
        y_pass_distance = math.floor((end_y - start_y) / num_y_passes)
        
        # Move to each photo location and take a photo
        num_photos_taken = 1
        x = start_x
        y = start_y
        z = start_z
        moving_right = True
        # Move to each Y pass
        for y in range(start_y, end_y+1, y_pass_distance):
            # Calculate x_steps given that a serpentine path alternates moving left-to-right and right-to-left
            x_steps = list(range(start_x, end_x+1, x_pass_distance))
            if not moving_right:
                x_steps.reverse()
            # Take a photo for each x pass
            for x in x_steps:
                # Update status bar
                self.statusBar().showMessage("Taking picture {num_photos_taken} of {total_photos} at {x},{y},{z}")
                # Set up callback to wait for move to finish
                self.serpentine_move_finished = False
                @pyqtSlot(str)
                def on_response(line):
                    # Remove this callback
                    self.clearcore.line_received.disconnect(on_response)
                    # Let the main thread know the move has finished
                    self.serpentine_move_finished = True
                # Connect the callback
                self.clearcore.line_received.connect(on_response)
                # Send command to begin moving to the desired location
                self.send_clearcore_command(f"m {x},{y},{z}")
                # Wait for move to finish
                while not self.serpentine_move_finished:
                    QtTest.QTest.qWait(100) # Delay 100 ms using a non-blocking version of sleep
                # Wait 1 second for machine vibrations to stop
                QtTest.QTest.qWait(1000)
                # Take photo
                self.take_photo((x,y,z), batch_dir)
            # After all x passes are finished, reset the direction of motion
            moving_right = not moving_right
        
        # After all photos have been taken, reset the START button action
        self.serpentine_start_button.clicked.connect(self.take_serpentine_photos)
        self.serpentine_paused = None
        self.serpentine_start_button.setText(start_text)
        
        # Set status bar message
        self.statusBar().showMessage("Finished taking serpentine photos!")
        

    def browse_for_photo_directory(self):
        """Open a file browser dialog to select the photo directory"""
        
        photo_directory = QFileDialog.getExistingDirectory(self, 
            "Open Directory",
            self.photo_directory_textbox.text(),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
        )
        if photo_directory:
            self.photo_directory_textbox.setText(photo_directory)
            self.write_config()

    def take_screenshot(self):
        """Take a screenshot from the Galaxy S24 and display it"""
        # Disable the screenshot until the photo has been completed
        self.screenshot_button.setEnabled(False)
        
        # Create a temporary screenshot directory if one does not exist
        if self.screenshot_dir is None:
            self.screenshot_dir = tempfile.mkdtemp()
        
        # Set up a callback to display the screenshot once it has been taken
        @pyqtSlot(str)
        def on_screenshot(image_path):
            # Remove this callback
            self.camera.screenshot_captured.disconnect(on_screenshot)
            # Re-enable the input button
            self.screenshot_button.setEnabled(True)
            # Display the screenshot
            self.show_screenshot(image_path)
        self.camera.screenshot_captured.connect(on_screenshot)

        # Finally trigger a screenshot
        self.camera.trigger_screenshot.emit(self.screenshot_dir)

    def show_screenshot(self, image_path):
        """Display a screenshot in the designated label area"""
        
        # Create a QPixmap from the image data
        logging.debug(f"[Ui.show_screenshot] Showing screenshot: {path}")
        try:
            pixmap = QPixmap(image_path)
        except Exception as e:
            logger.error(f"[Ui.show_screenshot] Could not open screenshot at {image_path}: {e}")
            return
            
        # Display the QPixmap in the label area
        label_area.setPixmap(pixmap)
        label_area.setScaledContents(True)

    def take_photo(self, position, batch_dir):
        """Take a photo using the Galaxy S24, store it in batch_dir, and display it along with its x,y,z position tuple"""
        
        # Create the batch dir if it does not exist
        if not batch_dir:
            logger.error(f"[Ui.take_photo] Invalid batch directory: {batch_dir}")
            return
        try:
            if not os.path.exists(batch_dir):
                os.makedirs(batch_dir)
        except Exception as e:
            logger.error(f"[Ui.take_photo] Could not create batch directory at {batch_dir}: {e}")
            return
        
        # Set up a callback to display the photo once it has been taken
        @pyqtSlot(str)
        def on_photo(image_path):
            # Remove this callback
            self.camera.photo_captured.disconnect(on_photo)
            # Display the screenshot
            x,y,z = position
            caption = f"{x}, {y}"
            self.show_photo(image_path, caption)
            # Save photo info in the batch json file
            self.save_photo_info(image_path, position, batch_dir)
        self.camera.photo_captured.connect(on_photo)
        
        # Finally, trigger the photo to be taken
        self.camera.trigger_photo.emit(batch_dir)

    def save_photo_info(self, image_path, position, batch_dir):
        """Save the position and path info to the batch JSON file"""
        
        # Open up an existing (or create a new) batch JSON file
        batch_info_file = os.path.join(batch_dir, "batch.json")
        batch_info = {}
        if os.path.exists(batch_info_file):
            with open(batch_info_file, "r") as jsonfile:
                batch_info = json.load(jsonfile)
        
        # Add info about the new photo to the batch_info
        if batch_info.get('photos', None) is None:
            batch_info['photos'] = []
        file_name = os.path.split(image_path)[-1]
        batch_info['photos'].append(
            dict(file_name=file_name, position=position)
        )
        
        # Write the JSON file back to disk
        with open(batch_info_file, "w") as jsonfile:
            json.dump(batch_info, jsonfile)

    def show_photo(self, image_path, caption):
        """Display an photo in the designated label area, labeled with the supplied caption"""

        # Create a QPixmap from the image data
        logging.debug(f"[Ui.show_image] Showing image path: {path}")
        try:
            pixmap = QPixmap(image_path)
        except Exception as e:
            logger.error(f"[Ui.show_image] Could not open photo at {path}: {e}")
            return

        # If exif orientation information exists, rotate QPixmap accordingly
        try:
            with open(image_path, 'rb') as image_file:
                img = exif.Image(image_file)
                orientation = img.orientation
                if orientation == 1: # Normal orientation, no rotation
                    pass
                elif orientation == 3: # Rotate 180
                    pixmap = pixmap.transformed(QTransform().rotate(180))
                elif orientation == 6: # Rotate 90 CCW
                    pixmap = pixmap.transformed(QTransform().rotate(90))
                elif orientation == 8: # Rotate 270 CCW
                    pixmap = pixmap.transformed(QTransform().rotate(270))
                else:
                    logger.error(f"[Ui.show_photo] Unsupported jpg orientation: {orientation}")
        except Exception as e:
            pass

        # If a photo is already present in the "current" photo label, move it to the "prior" photo label
        if current_photo_label.pixmap() is not None:
            prior_photo_label.setPixmap(current_photo_label.pixmap())
            prior_photo_label.setScaledContents(True)
            prior_photo_caption_label.setText(current_photo_caption_label.text())

        # Display the image and caption in the "current" photo label area
        current_photo_label.setPixmap(pixmap)
        current_photo_label.setScaledContents(True)
        image_filename = os.path.split(image_path)[-1]
        current_photo_caption_label.setText(f"{image_filename} ({caption})")


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    QApplication.closeAllWindows()
    QApplication.quit()

def main():   
    # Attempt to properly set app name on OSX
    if sys.platform.startswith('darwin'):
        # Set app name, if PyObjC is installed
        # Python 2 has PyObjC preinstalled
        # Python 3: pip3 install pyobjc-framework-Cocoa
        try:
            from Foundation import NSBundle
            bundle = NSBundle.mainBundle()
            if bundle:
                app_name = os.path.splitext(os.path.basename(sys.argv[0]))[0]
                app_info = bundle.localizedInfoDictionary() or bundle.infoDictionary()
                if app_info:
                    app_info['CFBundleName'] = APPLICATION_NAME
        except ImportError:
            pass
    
    app = QApplication(sys.argv) # Create an instance of QtWidgets.QApplication
    app.setApplicationName(APPLICATION_NAME) # Set application name (works on MS Windows)
    window = Ui(sys.argv) # Create an instance of our class
    
    # Set up a signal handler to quit on CTRL-C, and use a timer to let the interpreter run now and then
    signal.signal(signal.SIGINT, sigint_handler)
    timer = QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    
    # Start the application
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()