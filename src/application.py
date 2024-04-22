#!/usr/bin/env python3

import sys, os, threading, time, signal
import exif, json, math, argparse, tempfile
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, QThread, Qt
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5 import QtTest

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from utils import parse_int
from serial_driver import Gripper, ClearCore, DummyClearCore
from camera_driver import GalaxyS24
from camera_calibration import CameraCalibration

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
        self.clearcore = None # For the ClearCore Serial thread
        self.gripper = None # For the Gripper Serial thread
        self.camera = None # For the GalaxyS24 thread
        self.camera_calibration = None # For the CameraCalibration thread
        
        # Open configuration file
        try:
            parser = argparse.ArgumentParser(description="Puzzle Bot GUI Application", exit_on_error=False)
            parser.add_argument('config_file', help='JSON file representing runtime configuration')
            args = parser.parse_args()
            self.config_file_path = args.config_file
        except (argparse.ArgumentError, SystemExit) as e:
            logging.error(f"[Ui.init] Config file not specified in arguments: {e}")
            sys.exit()
            return
        if not self.config_file_path or not os.path.exists(self.config_file_path):
            logging.debug(f"[Ui.init] Creating config file at: {self.config_file_path}")
            self.config = {}
            # Write self.config to a JSON-formatted config file
            with open(self.config_file_path, "w") as jsonfile:
                json.dump(self.config, jsonfile)
        
        # Setup tabs
        self.setup_move_robot_tab()
        self.setup_serpentine_photos_tab()
        self.setup_camera_calibration_tab()
        self.setup_gripper_calibration_tab()
    
    def closeEvent(self, event):
        logging.debug(f"[Ui.closeEvent] closing main window")
        
        # Close the ClearCore serial thread
        if self.clearcore is not None:
            logging.debug(f"[Ui.closeEvent] closing ClearCoreSerial thread")
            self.clearcore.thread_closing.emit() # Tell the thread to exit its run loop
            self.clearcore.wait() # Wait for the thread to close (also closes the serial port)
            
        # Close the Gripper serial thread
        if self.gripper is not None:
            logging.debug(f"[Ui.closeEvent] closing Gripper thread")
            self.gripper.thread_closing.emit() # Tell the thread to exit its run loop
            self.gripper.wait() # Wait for the thread to close (also closes the serial port)

        # Close the GalaxyS24 thread
        if self.camera is not None:
            logging.debug(f"[Ui.closeEvent] closing GalaxyS24 thread")
            self.camera.thread_closing.emit() # Tell the thread to exit its run loop
            self.camera.wait() # Wait for the thread to close
        
        # Close the CameraCalibration thread
        if self.camera_calibration is not None:
            logging.debug(f"[Ui.closeEvent] closing CameraCalibration thread")
            self.camera_calibration.thread_closing.emit() # Tell the thread to exit its run loop
            self.camera_calibration.wait() # Wait for the thread to close
        
        # Continue closing the app
        logging.debug(f"[Ui.closeEvent] continuing to close")
        event.accept()
                
    def pixmap_from_image(self, image_path):
        """Create a pixmap from the image file at the supplied path, rotating for orientation as needed"""
        # Try to open the file as a pixmap
        logging.debug(f"[Ui.pixmap_from_image] Opening image path: {image_path}")
        try:
            pixmap = QPixmap(image_path)
        except Exception as e:
            logger.error(f"[Ui.pixmap_from_image] Could not open photo at {image_path}: {e}")
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
                    logger.error(f"[Ui.pixmap_from_image] Unsupported jpg orientation: {orientation}")
        except Exception as e:
            pass
        
        # Return the results
        return pixmap
            
    def display_image(self, image_path, label_area):
        """Display an image in a label area"""
        logging.debug(f"[Ui.display_image] displaying image at: {image_path}")
        if label_area is not None:
            # Create a pixmap from the image
            pixmap = self.pixmap_from_image(image_path)
            # Display the QPixmap in the label area, scaled and centered
            pixmap = pixmap.scaledToHeight(label_area.height(), Qt.SmoothTransformation)
            label_area.setPixmap(pixmap)
            label_area.setAlignment(Qt.AlignCenter)

        # Return the results
        return pixmap
 
    def browse_for_image(self, textbox, label_area=None):
        """Open a file browser dialog to select an image file, optionally displaying it in a label area"""
        preselected_filepath = textbox.text()
        directory = ''
        if preselected_filepath:
            directory = os.path.split(preselected_filepath)[0]
        new_filepath, filter = QFileDialog.getOpenFileName(self, 
            "Open Image",
            directory,
            "Images (*.png *.jpg)"
        )
        if new_filepath:
            textbox.setText(new_filepath)
            self.display_image(new_filepath, label_area)
       
    def browse_for_directory(self, textbox):
        """Open a file browser dialog to select a directory"""
        directory = QFileDialog.getExistingDirectory(self, 
            "Open Directory",
            textbox.text(),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
        )
        if directory:
            textbox.setText(directory)


    ################################################################################################
    ################################################################################################
    # "MOVE ROBOT" TAB
    ################################################################################################
    ################################################################################################
    
    def setup_move_robot_tab(self):
        """Configure UI on the MOVE ROBOT tab"""
        # Configure serial output textarea to always scroll to the bottom
        scrollbar = self.clearcore_serial_output_textarea.verticalScrollBar()
        scrollbar.rangeChanged.connect(lambda minVal, maxVal: scrollbar.setValue(scrollbar.maximum()))
        
        # Set all inputs as unavailable until we're connected to the ClearCore and Gripper
        self.set_clearcore_availability(False)
        self.set_gripper_availability(False)
        self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Opening ClearCore...</font><br/>")
        self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Opening Gripper...</font><br/>")
        
        # Connect to the ClearCore over serial
        self.clearcore = ClearCore(parent=self)
        #self.clearcore = DummyClearCore(parent=self)
        self.clearcore.line_received.connect(self.process_clearcore_response)
        self.clearcore.start()
        
        # Connect to the Gripper Controller
        self.gripper = Gripper(parent=self)
        self.gripper.line_received.connect(self.process_gripper_response)
        self.gripper.start()
        
        # Connect all the buttons (coding this is faster than creating slots in the qt designer gui)
        self.vacuum_on_button.clicked.connect(lambda: self.send_clearcore_command("1", update_position=False))
        self.vacuum_off_button.clicked.connect(lambda: self.send_clearcore_command("0", update_position=False))
        self.home_button.clicked.connect(lambda: self.send_clearcore_command("h"))
        self.jog_positive_x_button.clicked.connect(lambda: self.send_clearcore_command("d"))
        self.jog_negative_x_button.clicked.connect(lambda: self.send_clearcore_command("a"))
        self.jog_positive_y_button.clicked.connect(lambda: self.send_clearcore_command("w"))
        self.jog_negative_y_button.clicked.connect(lambda: self.send_clearcore_command("s"))
        self.jog_positive_z_button.clicked.connect(lambda: self.send_clearcore_command("o"))
        self.jog_negative_z_button.clicked.connect(lambda: self.send_clearcore_command("l"))
        self.linear_encoder_button.clicked.connect(lambda: self.send_gripper_command('d', update_position=False))
        self.home_gripper_button.clicked.connect(lambda: self.send_gripper_command('h'))

        @pyqtSlot()
        def rotate_absolute():
            # Get desired position (rotational counts)
            angle_counts = parse_int(self.gripper_angle_textbox.text())
            if not angle_counts or angle_counts is None:
                angle_counts = 0
                self.gripper_angle_textbox.setText(str(angle_counts))
            # Send command to move to this position
            self.send_gripper_command(f"r {angle_counts}")
        self.rotate_gripper_button.clicked.connect(rotate_absolute)
        
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
        def on_clearcore_open(port_status):
            if (port_status):
                self.set_clearcore_availability(True)
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>ClearCore opened!</font><br/>")
            else:
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Error opening ClearCore.</font><br/>")
        self.clearcore.port_opened.connect(on_clearcore_open)
        
        # Wait 2 seconds for the Arduino to reset before we make any writes to it
        @pyqtSlot(bool)
        def on_gripper_open(port_status):
            if (port_status):
                self.set_gripper_availability(True)
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Gripper opened!</font><br/>")
            else:
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Error opening Gripper.</font><br/>")
        self.gripper.port_opened.connect(on_gripper_open)

    @pyqtSlot(str)
    def process_gripper_response(self, line):
        """Process Gripper responses."""

        logging.debug(f"[Ui.process_gripper_response] line received: {repr(line)}")

        # Process 'p' motor position responses
        if line.startswith("SUCCESS: printMotorPosition"):
            # Parse the encoder position from the response (ex: "SUCCESS: printMotorPosition 10000 10002")
            motor_encoder_angle = parse_int(line.split(" ")[-1])
            # Update textbox with the new position
            self.rotate_gripper_results_label.setText(
                f"Actual:\n({str(motor_encoder_angle)})"
            )
            # Hide these position responses from the serial output so they don't clog up that textarea
            return
        
        # Process 'd' linear encoder position responses
        if line.startswith("SUCCESS: printEncoderDigitalValue"):
            linear_encoder_value = parse_int(line.split(" ")[-1])
            # Update the textbox with the value
            self.linear_encoder_textbox.setText(str(linear_encoder_value))

        # Format line and add it to the textarea
        html_line = ""
        status = line.split(":")[0]
        message = ":".join(line.split(":")[1:])

        red = "#ff3300"
        green = "#009900"
        if status == "SUCCESS":
            html_line += f"<font style='color: {green};'>GRIPPER {status}:</font> {message}<br/>"
        else:
            html_line += f"<font style='color: {red};'>GRIPPER {status}:</font> {message}<br/>"
        self.clearcore_serial_output_textarea.insertHtml(html_line) 

    def set_gripper_availability(self, enabled=True):
        """Set the availability of all input buttons related to the Gripper to enabled or disabled."""
        self.linear_encoder_button.setEnabled(enabled)
        self.rotate_gripper_button.setEnabled(enabled)
        self.home_gripper_button.setEnabled(enabled)
        

    def send_gripper_command(self, command, blocking=False, update_position=True):
        """Send command to the gripper serial port, disabling corresponding button inputs until we receive a response.
        
        If blocking is set to True, will sleep until command is finished before returning
        
        If update_position is set to True, will send a follow-on 'p' command to update position
        
        """
        if (not command):
            logging.error(f"[Ui.send_gripper_command] command parameter missing or empty")
            return
        else:
            logging.debug(f"[Ui.send_gripper_command] sending command: {command}")
        
        # Disable input buttons until we get a response
        self.set_gripper_availability(False)
        
        # Setup a response callback
        self.gripper_command_finished = False
        @pyqtSlot(str)
        def on_response(line):
            # Remove this callback
            self.gripper.line_received.disconnect(on_response)
            
            # If desired, send a follow-on 'p' command to get the current position
            if update_position:
                self.gripper_position_command_finished = False
                @pyqtSlot(str)
                def on_position_response(line):
                    # Remove this callback
                    self.gripper.line_received.disconnect(on_position_response)
                    # Indicate the follow-on command has finished
                    self.gripper_position_command_finished = True
                    # Finally re-enable the input buttons
                    self.set_gripper_availability(True)
                self.gripper.line_received.connect(on_position_response)
                # Send the follow-on 'p'
                self.gripper.line_ready_to_transmit.emit("p")
                # Optionally, block until the response has arrived
                if blocking:
                    while not self.gripper_position_command_finished:
                        QtTest.QTest.qWait(100) # Delay 100 ms using a non-blocking version of sleep
            else:
                # Finally re-enable the input buttons
                self.set_gripper_availability(True)
                        
            # Indicate the initial command has finished
            self.gripper_command_finished = True
            
        # Connect the callback    
        self.gripper.line_received.connect(on_response)

        # Emit the command to the ClearCore QThread, for transmission over the serial port
        self.gripper.line_ready_to_transmit.emit(command)
                
        # Optionally, block until the command response has arrived
        if blocking:
            while not self.gripper_command_finished:
                QtTest.QTest.qWait(100) # Delay 100 ms using a non-blocking version of sleep

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
        self.gripper_calibration_move_start_button.setEnabled(enabled)
        self.move_camera_delta_button.setEnabled(enabled)
        self.move_gripper_delta_button.setEnabled(enabled)
        self.gripper_calibration_rotate_button.setEnabled(enabled)

    def send_clearcore_command(self, command, blocking=False, update_position=True):
        """Send command to the clearcore serial port, disabling corresponding button inputs until we receive a response.
        
        If blocking is set to True, will sleep until command is finished before returning
        
        If update_position is set to True, will send a follow-on 'p' command to update position
        
        """
        if (not command):
            logging.error(f"[Ui.send_clearcore_command] command parameter missing or empty")
            return
        else:
            logging.debug(f"[Ui.send_clearcore_command] sending command: {command} with blocking={blocking} and update_position={update_position}")
        
        # Disable input buttons until we get a response
        self.set_clearcore_availability(False)
        
        # Setup a response callback
        self.clearcore_command_finished = False
        @pyqtSlot(str)
        def on_response(line):
            # Remove this callback
            self.clearcore.line_received.disconnect(on_response)
            
            # If desired, send a follow-on 'p' command to get the current position
            if update_position:
                self.clearcore_position_command_finished = False
                @pyqtSlot(str)
                def on_position_response(line):
                    # Remove this callback
                    self.clearcore.line_received.disconnect(on_position_response)
                    # Indicate the follow-on command has finished
                    self.clearcore_position_command_finished = True
                    # Finally re-enable the input buttons
                    self.set_clearcore_availability(True)
                self.clearcore.line_received.connect(on_position_response)
                # Send the follow-on 'p'
                self.clearcore.line_ready_to_transmit.emit("p")
                # Optionally, block until the response has arrived
                if blocking:
                    while not self.clearcore_position_command_finished:
                        QtTest.QTest.qWait(100) # Delay 100 ms using a non-blocking version of sleep
            else:
                # Finally re-enable the input buttons
                self.set_clearcore_availability(True)
                        
            # Indicate the initial command has finished
            self.clearcore_command_finished = True
            
        # Connect the callback    
        self.clearcore.line_received.connect(on_response)

        # Emit the command to the ClearCore QThread, for transmission over the serial port
        self.clearcore.line_ready_to_transmit.emit(command)
        
        # Optionally, block until the command response has arrived
        if blocking:
            while not self.clearcore_command_finished:
                QtTest.QTest.qWait(100) # Delay 100 ms using a non-blocking version of sleep


    ################################################################################################
    ################################################################################################
    # "SERPENTINE PHOTOS" TAB
    ################################################################################################
    ################################################################################################

    def setup_serpentine_photos_tab(self):
        """Configure UI on the SERPENTINE PHOTOS tab"""
        # Setup instance variables
        self.screenshot_dir = None
        self.serpentine_move_finished = None
        self.serpentine_paused = None
        self.photo_capture_finished = None
        
        # Read the config file and update all textboxes
        self.read_serpentine_config()
        
        # Configure all textboxes to auto-save config file upon change
        self.serpentine_photo_directory_textbox.textChanged.connect(self.write_serpentine_config)
        self.start_x_textbox.textChanged.connect(self.write_serpentine_config)
        self.start_y_textbox.textChanged.connect(self.write_serpentine_config)
        self.start_z_textbox.textChanged.connect(self.write_serpentine_config)
        self.end_x_textbox.textChanged.connect(self.write_serpentine_config)
        self.end_y_textbox.textChanged.connect(self.write_serpentine_config)
        self.end_z_textbox.textChanged.connect(self.write_serpentine_config)
        self.overlap_x_textbox.textChanged.connect(self.write_serpentine_config)
        self.overlap_y_textbox.textChanged.connect(self.write_serpentine_config)
        
        # Configure buttons
        self.puzzle_photo_directory_browse_button.clicked.connect(lambda: self.browse_for_directory(self.serpentine_photo_directory_textbox))
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
        self.serpentine_start_button.clicked.disconnect(self.take_serpentine_photos) # Disable default action
        
        # Make sure the photo directory exists
        photo_directory = self.serpentine_photo_directory_textbox.text()
        if not photo_directory or not os.path.exists(photo_directory) or not os.path.isdir(photo_directory):
            logger.error(f"[Ui.take_serpentine_photos] Photo directory does not exist at {photo_directory}")
            return
        
        # Auto-generate the next batch directory (they are numbered sequentially)
        last_batch_number = 0
        sub_dirs = [d for d in os.listdir(photo_directory) if os.path.isdir(os.path.join(photo_directory, d))]
        batch_dirs = [d for d in sub_dirs if d.isnumeric()]
        batch_dirs.sort(key=int) # sorts in place
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
                
                # Send command to begin moving to the desired location, and block until finished
                self.send_clearcore_command(f"m {x},{y},{z}", blocking=True)
                
                # Wait 1 second for machine vibrations to stop
                QtTest.QTest.qWait(1000)
                
                # Set up callback to wait for photo capture to finish
                self.photo_capture_finished = False
                @pyqtSlot(str)
                def on_capture(image_path):
                    # Remove this callback
                    self.camera.photo_captured.disconnect(on_capture)
                    # Let the main thread know the move has finished
                    self.photo_capture_finished = True
                self.camera.photo_captured.connect(on_capture)
                
                # Trigger a photo to be taken
                self.take_serpentine_photo((x,y,z), batch_dir)
                
                # Wait for the capture to finish
                while not self.photo_capture_finished:
                    QtTest.QTest.qWait(100) # Delay 100 ms using a non-blocking version of sleep

            # After all x passes are finished, reset the direction of motion
            moving_right = not moving_right
        
        # After all photos have been taken, reset the START button action
        self.serpentine_start_button.clicked.connect(self.take_serpentine_photos)
        self.serpentine_paused = None
        self.serpentine_start_button.setText(start_text)
        
        # Set status bar message
        self.statusBar().showMessage("Finished taking serpentine photos!")

    def take_screenshot(self):
        """Take a screenshot from the Galaxy S24 and display it"""
        # Disable the screenshot button until the photo has been completed
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
            self.display_image(image_path, self.screenshot_label)
        self.camera.screenshot_captured.connect(on_screenshot)

        # Finally trigger a screenshot
        self.camera.trigger_screenshot.emit(self.screenshot_dir)

    def take_serpentine_photo(self, position, batch_dir):
        """Take a photo using the Galaxy S24, store it in batch_dir, and display it along with its x,y,z position tuple"""
        
        # Create the batch dir if it does not exist
        if not batch_dir:
            logger.error(f"[Ui.take_serpentine_photo] Invalid batch directory: {batch_dir}")
            return
        try:
            if not batch_dir or not os.path.exists(batch_dir):
                os.makedirs(batch_dir)
        except Exception as e:
            logger.error(f"[Ui.take_serpentine_photo] Could not create batch directory at {batch_dir}: {e}")
            return
        
        # Set up a callback to display the photo once it has been taken
        @pyqtSlot(str)
        def on_capture(image_path):
            # Remove this callback
            self.camera.photo_captured.disconnect(on_capture)
            # Display the screenshot
            x,y,z = position
            caption = f"{x}, {y}"
            self.show_serpentine_photo(image_path, caption)
            # Save photo info in the batch json file
            self.save_serpentine_photo_info(image_path, position, batch_dir)
        self.camera.photo_captured.connect(on_capture)
        
        # Trigger the photo to be taken
        self.camera.trigger_photo.emit(batch_dir)

    def save_serpentine_photo_info(self, image_path, position, batch_dir):
        """Save the position and path info to the batch JSON file"""
        
        # Open up an existing (or create a new) batch JSON file
        batch_info_file = os.path.join(batch_dir, "batch.json")
        batch_info = {}
        if batch_info_file and os.path.exists(batch_info_file):
            with open(batch_info_file, "r") as jsonfile:
                batch_info = json.load(jsonfile)
        
        # Add info about the new photo to the batch_info
        if batch_info.get('photos', None) is None:
            batch_info['photos'] = []
        file_name = os.path.split(image_path)[-1]
        batch_info['photos'].append(
            dict(file_name=file_name, position=position)
        )
        
        # Ensure runtime parameters are saved in the JSON file as well
        batch_info['start_x'] = self.start_x_textbox.text()
        batch_info['start_y'] = self.start_y_textbox.text()
        batch_info['start_z'] = self.start_z_textbox.text()
        batch_info['end_x'] = self.end_x_textbox.text()
        batch_info['end_y'] = self.end_y_textbox.text()
        batch_info['end_z'] = self.end_z_textbox.text()
        batch_info['overlap_x'] = self.overlap_x_textbox.text()
        batch_info['overlap_y'] = self.overlap_y_textbox.text()
        
        # Write the JSON file back to disk
        with open(batch_info_file, "w") as jsonfile:
            json.dump(batch_info, jsonfile)

    def show_serpentine_photo(self, image_path, caption):
        """Display an photo in the designated label area, labeled with the supplied caption"""

        # Create a QPixmap from the image data
        pixmap = self.pixmap_from_image(image_path)

        # If a photo is already present in the "current" photo label, move it to the "prior" photo label
        if self.current_photo_label.pixmap() is not None:
            self.prior_photo_label.setPixmap(self.current_photo_label.pixmap())
            self.prior_photo_label.setAlignment(Qt.AlignCenter) # Center (letterbox) the image
            #self.prior_photo_label.setScaledContents(True) # Removed for now because it changes the aspect ratio
            self.prior_photo_caption_label.setText(self.current_photo_caption_label.text())

        # Display the image and caption in the "current" photo label area
        self.current_photo_label.setPixmap(pixmap)
        self.current_photo_label.setAlignment(Qt.AlignCenter) # Center (letterbox) the image
        #self.current_photo_label.setScaledContents(True) # Removed for now because it changes the aspect ratio
        image_filename = os.path.split(image_path)[-1]
        self.current_photo_caption_label.setText(f"{image_filename} ({caption})")

    def read_serpentine_config(self):
        """Update serpentine tab input boxes from JSON-formatted config file"""
        with open(self.config_file_path, "r") as jsonfile:
            self.config = json.load(jsonfile)
            self.serpentine_photo_directory_textbox.setText(self.config.get('serpentine_photo_directory', ''))
            self.start_x_textbox.setText(self.config.get('start_x', ''))
            self.start_y_textbox.setText(self.config.get('start_y', ''))
            self.start_z_textbox.setText(self.config.get('start_z', ''))
            self.end_x_textbox.setText(self.config.get('end_x', ''))
            self.end_y_textbox.setText(self.config.get('end_y', ''))
            self.end_z_textbox.setText(self.config.get('end_z', ''))
            self.overlap_x_textbox.setText(self.config.get('overlap_x', ''))
            self.overlap_y_textbox.setText(self.config.get('overlap_y', ''))
    
    def write_serpentine_config(self):
        """Update self.config from serpentine tab input boxes and write it to a JSON-formatted config file"""
        # Update self.config from input boxes
        self.config['serpentine_photo_directory'] = self.serpentine_photo_directory_textbox.text()
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

    ################################################################################################
    ################################################################################################
    # "CAMERA CALIBRATION" TAB
    ################################################################################################
    ################################################################################################
    def setup_camera_calibration_tab(self):
        """Configure UI on the CAMERA CALIBRATION tab"""
        # Read the config file and update all textboxes
        self.read_camera_calibration_config()
        
        # Configure all textboxes to auto-save config file upon change
        self.camera_calibration_checker_rows_spinbox.valueChanged.connect(self.write_camera_calibration_config)
        self.camera_calibration_checker_columns_spinbox.valueChanged.connect(self.write_camera_calibration_config)
        self.camera_calibration_checker_width_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_calibration_checker_height_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.pre_camera_calibration_photo_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_calibration_photo_directory_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.post_camera_calibration_photo_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_fx_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_fy_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_cx_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_cy_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_k1_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_k2_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_k3_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_p1_textbox.textChanged.connect(self.write_camera_calibration_config)
        self.camera_p2_textbox.textChanged.connect(self.write_camera_calibration_config)

        # Configure buttons
        self.camera_calibration_button.clicked.connect(self.perform_camera_calibration)
        self.pre_camera_calibration_measure_error_button.clicked.connect(self.measure_pre_camera_calibration_error)
        self.post_camera_calibration_measure_error_button.clicked.connect(self.measure_post_camera_calibration_error)
        self.pre_camera_calibration_photo_browse_button.clicked.connect(lambda: self.browse_for_image(self.pre_camera_calibration_photo_textbox, self.pre_camera_calibration_example_photo_label))
        self.post_camera_calibration_photo_browse_button.clicked.connect(lambda: self.browse_for_image(self.post_camera_calibration_photo_textbox, self.post_camera_calibration_example_photo_label))
        self.camera_calibration_photo_directory_browse_button.clicked.connect(lambda: self.browse_for_directory(self.camera_calibration_photo_directory_textbox))
        
        # Start up a thread for the camera calibration functionality
        self.camera_calibration = CameraCalibration(parent=self)
        self.camera_calibration.start()
    
    def measure_checkerboard_error(self, image_path, rms_error_textbox, max_error_textbox, annotated_image_label_area):
        """Find features in an image, fit lines to the rows and columns, and measure error of that fit"""
        # Make sure the selected image exists
        if not image_path or not os.path.exists(image_path):
            logger.error(f"[Ui.measure_checkerboard_error] File does not exist at image_path")
            return
        
        # Obtain checkerboard params
        checker_rows = self.camera_calibration_checker_rows_spinbox.value()
        checker_columns = self.camera_calibration_checker_columns_spinbox.value()
        checker_width = float(self.camera_calibration_checker_width_textbox.text())
        checker_height = float(self.camera_calibration_checker_height_textbox.text())
        
        # Set up a callback to report the results
        @pyqtSlot(float, float, float, float, str)
        def on_results(average_rms_error_px, max_error_px, average_rms_error_inches, max_error_inches, annotated_image_path):
            # Remove this callback
            self.camera_calibration.checkerboard_error_measurements_ready.disconnect(on_results)
            # Report the results
            rms_error_textbox.setText(f"{round(average_rms_error_px, 2)} px ({round(average_rms_error_inches, 5)} inches)")
            max_error_textbox.setText(f"{round(max_error_px, 2)} px ({round(max_error_inches, 5)} inches)")
            self.display_image(annotated_image_path, annotated_image_label_area)
        self.camera_calibration.checkerboard_error_measurements_ready.connect(on_results)
        # Trigger a measurement
        self.camera_calibration.trigger_checkerboard_error_measurements.emit(
            image_path, checker_rows, checker_columns, checker_width, checker_height
        )

    @pyqtSlot()
    def measure_pre_camera_calibration_error(self):
        """Take a measurement of the error in an uncalibrated image"""
        image_path = self.pre_camera_calibration_photo_textbox.text()
        self.measure_checkerboard_error(
            image_path, 
            self.pre_camera_calibration_rms_error_textbox,
            self.pre_camera_calibration_max_error_textbox,
            self.pre_camera_calibration_error_visualization_label
        )
        
    def get_camera_intrinsics(self):
        """Return (camera_matrix, distortion_coefficients) arrays from textbox infos"""
        # Put together camera matrix and distortion coefficients
        # [[fx 0  cx]
        #  [0  fy cy]
        #  [0  0  1]]
        camera_matrix = [[0, 0, 0],[0, 0, 0],[0, 0, 1]]
        camera_matrix[0][0] = float(self.camera_fx_textbox.text() or 0)
        camera_matrix[1][1] = float(self.camera_fy_textbox.text() or 0)
        camera_matrix[0][2] = float(self.camera_cx_textbox.text() or 0)
        camera_matrix[1][2] = float(self.camera_cy_textbox.text() or 0)
        # Distortion coefficients:
        distortion_coefficients = [[0, 0, 0, 0, 0]] # [[k1, k2, p1, p2, k3]]
        distortion_coefficients[0][0] = float(self.camera_k1_textbox.text() or 0)
        distortion_coefficients[0][1] = float(self.camera_k2_textbox.text() or 0)
        distortion_coefficients[0][2] = float(self.camera_p1_textbox.text() or 0)
        distortion_coefficients[0][3] = float(self.camera_p2_textbox.text() or 0)
        distortion_coefficients[0][4] = float(self.camera_k3_textbox.text() or 0)
        # Return results
        return (camera_matrix, distortion_coefficients)
    
    @pyqtSlot()
    def measure_post_camera_calibration_error(self):
        # Compute source and destination file paths
        source_image_path = self.post_camera_calibration_photo_textbox.text()
        path_parts = source_image_path.split(".")
        destination_image_path = ".".join(path_parts[:-1]) + "-undistorted." + path_parts[-1]
        
        # Compute camera intrinsics
        camera_matrix, distortion_coefficients = self.get_camera_intrinsics()
        
        # Setup a callback to report the results
        @pyqtSlot()
        def on_results():
            # Remove this callback
            self.camera_calibration.undistorted_image_ready.disconnect(on_results)    
            # Display the undistorted image along with error measurements
            self.measure_checkerboard_error(
                destination_image_path, 
                self.post_camera_calibration_rms_error_textbox,
                self.post_camera_calibration_max_error_textbox,
                self.post_camera_calibration_error_visualization_label
            )
        # Connect the callback
        self.camera_calibration.undistorted_image_ready.connect(on_results)
        
        # Trigger undistortion process
        self.camera_calibration.trigger_undistort_image.emit(
            source_image_path, destination_image_path, camera_matrix, distortion_coefficients
        )

    @pyqtSlot()
    def perform_camera_calibration(self):
        """Perform camera calibration routine"""
        # Setup a callback to report the results.
        @pyqtSlot(float, float, list, list)
        def on_results(rms_rpe, max_error, camera_matrix, distortion_coefficients):
            # Remove this callback
            self.camera_calibration.camera_calibration_results_ready.disconnect(on_results)
            # Camera matrix:
            # [[fx 0  cx]
            #  [0  fy cy]
            #  [0  0  1]]
            self.camera_fx_textbox.setText(str(round(camera_matrix[0][0],5)))
            self.camera_fy_textbox.setText(str(round(camera_matrix[1][1],5)))
            self.camera_cx_textbox.setText(str(round(camera_matrix[0][2],5)))
            self.camera_cy_textbox.setText(str(round(camera_matrix[1][2],5)))
            # Distortion coefficients:
            # [[k1, k2, p1, p2, k3]]
            self.camera_k1_textbox.setText(str(round(distortion_coefficients[0][0],5)))
            self.camera_k2_textbox.setText(str(round(distortion_coefficients[0][1],5)))
            self.camera_p1_textbox.setText(str(round(distortion_coefficients[0][2],5)))
            self.camera_p2_textbox.setText(str(round(distortion_coefficients[0][3],5)))
            self.camera_k3_textbox.setText(str(round(distortion_coefficients[0][4],5)))
            # Report rms and max errors
            self.camera_calibration_rms_error_textbox.setText(str(round(rms_rpe,5)))
            self.camera_calibration_max_error_textbox.setText(str(round(max_error,5)))
        # Add the callback
        self.camera_calibration.camera_calibration_results_ready.connect(on_results)
        
        # Obtain checkerboard params
        checker_rows = self.camera_calibration_checker_rows_spinbox.value()
        checker_columns = self.camera_calibration_checker_columns_spinbox.value()
        checker_width = float(self.camera_calibration_checker_width_textbox.text())
        checker_height = float(self.camera_calibration_checker_height_textbox.text())
        
        # Get camera intrinsics from textboxes
        camera_matrix, distortion_coefficients = self.get_camera_intrinsics()
        
        # Trigger the camera calibration routine
        photo_directory = self.camera_calibration_photo_directory_textbox.text()
        self.camera_calibration.trigger_camera_calibration.emit(
            photo_directory, checker_rows, checker_columns, checker_width, checker_height, camera_matrix, distortion_coefficients
        )
    
    def read_camera_calibration_config(self):
        """Update camera calibration tab input boxes from JSON-formatted config file"""
        with open(self.config_file_path, "r") as jsonfile:
            self.config = json.load(jsonfile)
            self.camera_calibration_checker_rows_spinbox.setValue(self.config.get('camera_calibration_checker_rows', 0))
            self.camera_calibration_checker_columns_spinbox.setValue(self.config.get('camera_calibration_checker_columns', 0))
            self.camera_calibration_checker_width_textbox.setText(self.config.get('camera_calibration_checker_width', '0.0'))
            self.camera_calibration_checker_height_textbox.setText(self.config.get('camera_calibration_checker_height', '0.0'))
            self.pre_camera_calibration_photo_textbox.setText(self.config.get('pre_camera_calibration_image', ''))
            self.camera_calibration_photo_directory_textbox.setText(self.config.get('camera_calibration_photo_directory', ''))
            self.post_camera_calibration_photo_textbox.setText(self.config.get('post_camera_calibration_image', ''))
            self.camera_fx_textbox.setText(self.config.get('camera_fx', ''))
            self.camera_fy_textbox.setText(self.config.get('camera_fy', ''))
            self.camera_cx_textbox.setText(self.config.get('camera_cx', ''))
            self.camera_cy_textbox.setText(self.config.get('camera_cy', ''))
            self.camera_k1_textbox.setText(self.config.get('camera_k1', ''))
            self.camera_k2_textbox.setText(self.config.get('camera_k2', ''))
            self.camera_k3_textbox.setText(self.config.get('camera_k3', ''))
            self.camera_p1_textbox.setText(self.config.get('camera_p1', ''))
            self.camera_p2_textbox.setText(self.config.get('camera_p2', ''))
    
    def write_camera_calibration_config(self):
        """Update self.config from camera calibration tab input boxes and write it to a JSON-formatted config file"""
        # Update self.config from input boxes
        self.config['camera_calibration_checker_rows'] = self.camera_calibration_checker_rows_spinbox.value()
        self.config['camera_calibration_checker_columns'] = self.camera_calibration_checker_columns_spinbox.value()
        self.config['camera_calibration_checker_width'] = self.camera_calibration_checker_width_textbox.text()
        self.config['camera_calibration_checker_height'] = self.camera_calibration_checker_height_textbox.text()
        self.config['pre_camera_calibration_image'] = self.pre_camera_calibration_photo_textbox.text()
        self.config['camera_calibration_photo_directory'] = self.camera_calibration_photo_directory_textbox.text()
        self.config['post_camera_calibration_image'] = self.post_camera_calibration_photo_textbox.text()
        self.config['camera_fx'] = self.camera_fx_textbox.text()
        self.config['camera_fy'] = self.camera_fy_textbox.text()
        self.config['camera_cx'] = self.camera_cx_textbox.text()
        self.config['camera_cy'] = self.camera_cy_textbox.text()
        self.config['camera_k1'] = self.camera_k1_textbox.text()
        self.config['camera_k2'] = self.camera_k2_textbox.text()
        self.config['camera_k3'] = self.camera_k3_textbox.text()
        self.config['camera_p1'] = self.camera_p1_textbox.text()
        self.config['camera_p2'] = self.camera_p2_textbox.text()
        
        # Write self.config to a JSON-formatted config file
        with open(self.config_file_path, "w") as jsonfile:
            json.dump(self.config, jsonfile)
            
            
    ################################################################################################
    ################################################################################################
    # "GRIPPER CALIBRATION" TAB
    ################################################################################################
    ################################################################################################
    def setup_gripper_calibration_tab(self):
        """Configure UI on the GRIPPER CALIBRATION tab"""
        
        # Create a temporary directory for the gripper calibration photos
        self.gripper_photo_dir = tempfile.mkdtemp()
        
        # Read the config file and update all textboxes
        self.read_gripper_calibration_config()
        
        # Configure all textboxes to auto-save config file upon change
        self.gripper_calibration_start_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_start_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_delta_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_delta_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_delta_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_delta_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_angle_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_pickup_z_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_spin_z_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.motor_counts_x_per_px_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.motor_counts_y_per_px_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_to_gripper_x_transform_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_to_gripper_y_transform_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_to_gripper_angle_transform_textbox.textChanged.connect(self.write_gripper_calibration_config)

        # Configure buttons
        
        @pyqtSlot()
        def gripper_calibration_move_start():
            # Get desired position
            x = parse_int(self.gripper_calibration_start_x_textbox.text())
            y = parse_int(self.gripper_calibration_start_y_textbox.text())
            z = parse_int(self.gripper_calibration_start_z_textbox.text())
            # Send command to move to this absolute position
            self.send_clearcore_command(f"m {x},{y},{z}")
        self.gripper_calibration_move_start_button.clicked.connect(gripper_calibration_move_start)
        
        @pyqtSlot()
        def gripper_calibration_move_camera_delta():
            # Get desired position
            x = parse_int(self.camera_delta_x_textbox.text())
            y = parse_int(self.camera_delta_y_textbox.text())
            z = parse_int(self.camera_delta_z_textbox.text())
            # Send command to move to this absolute position
            self.send_clearcore_command(f"m {x},{y},{z}")
        self.move_camera_delta_button.clicked.connect(gripper_calibration_move_camera_delta)
        
        @pyqtSlot()
        def gripper_calibration_move_gripper_delta():
            # Get desired position
            x = parse_int(self.gripper_delta_x_textbox.text())
            y = parse_int(self.gripper_delta_y_textbox.text())
            z = parse_int(self.gripper_delta_z_textbox.text())
            # Send command to move to this absolute position
            self.send_clearcore_command(f"m {x},{y},{z}")
        self.move_gripper_delta_button.clicked.connect(gripper_calibration_move_gripper_delta)
        
        @pyqtSlot()
        def gripper_calibration_rotate_target():
            # Get desired rotation angle and pickup z location
            angle = parse_int(self.gripper_calibration_angle_textbox.text())
            pickup_x = parse_int(self.gripper_delta_x_textbox.text())
            pickup_y = parse_int(self.gripper_delta_y_textbox.text())
            pickup_z = parse_int(self.gripper_calibration_pickup_z_textbox.text())
            spin_z = parse_int(self.gripper_calibration_spin_z_textbox.text())
            
            # Pick up the piece (the gripper calibration target), rotate it, and put it back down
            self.send_gripper_command(f"r 0", blocking=True) # Set rotation to 0
            self.pickup_piece(pickup_x, pickup_y, pickup_z, spin_z) # Pick up the target
            self.send_gripper_command(f"r {angle}", blocking=True) # Rotate the target
            self.putdown_piece(pickup_x, pickup_y, pickup_z, spin_z) # Put down the target
            
            # Finally, move back to the start position to get ready to take the final photo
            x = parse_int(self.gripper_calibration_start_x_textbox.text())
            y = parse_int(self.gripper_calibration_start_y_textbox.text())
            z = parse_int(self.gripper_calibration_start_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")
        self.gripper_calibration_rotate_button.clicked.connect(gripper_calibration_rotate_target)
        
        @pyqtSlot()
        def take_gripper_calibration_start_photo():
            destination_photo_path = os.path.join(self.gripper_photo_dir, "start.jpg")
            self.take_photo(destination_photo_path, self.gripper_calibration_start_photo_label)
        self.gripper_calibration_take_start_photo_button.clicked.connect(take_gripper_calibration_start_photo)
        
        @pyqtSlot()
        def take_camera_delta_photo():
            destination_photo_path = os.path.join(self.gripper_photo_dir, "camera_delta.jpg")
            self.take_photo(destination_photo_path, self.camera_delta_photo_label)
        self.take_camera_delta_photo_button.clicked.connect(take_camera_delta_photo)
        
        @pyqtSlot()
        def take_gripper_delta_photo():
            destination_photo_path = os.path.join(self.gripper_photo_dir, "gripper_delta.jpg")
            self.take_photo(destination_photo_path, self.gripper_delta_photo_label)
        self.take_gripper_delta_photo_button.clicked.connect(take_gripper_delta_photo)
        
        # Start up a thread for the gripper calibration functionality
        # self.gripper_calibration = GripperCalibration(parent=self)
        # self.gripper_calibration.start()
    
    def pickup_piece(self, pickup_x, pickup_y, pickup_z, travel_z):
        """Vacuum on, Lower Z to pickup_z, pick up piece, raise Z to travel_z"""
        
        # Vacuum ON
        self.send_clearcore_command("1", blocking=True, update_position=False)
        
        # Lower Z
        self.send_clearcore_command(f"m {pickup_x},{pickup_y},{pickup_z}", blocking=True, update_position=True)

        # Pause to allow vacuum to form
        QtTest.QTest.qWait(1000) # Delay 1000 ms (using a non-blocking version of sleep)
        
        # Raise Z
        self.send_clearcore_command(f"m {pickup_x},{pickup_y},{travel_z}", blocking=True, update_position=True)

    def putdown_piece(self, putdown_x, putdown_y, putdown_z, travel_z):
        """Lower Z to putdown_z, vacuum off, raise Z to travel_z"""
        # Lower Z
        self.send_clearcore_command(f"m {putdown_x},{putdown_y},{putdown_z}", blocking=True, update_position=True)
        
        # Vacuum OFF
        self.send_clearcore_command("0", blocking=True, update_position=False)
        
        # Pause to allow vacuum to purge
        QtTest.QTest.qWait(1000) # Delay 1000 ms (using a non-blocking version of sleep)
        
        # Raise Z
        self.send_clearcore_command(f"m {putdown_x},{putdown_y},{travel_z}", blocking=True, update_position=True)

    def take_photo(self, destination_photo_path, label_area):
        """Take a photo using the Galaxy S24, store it at the specified path, and display it in the label area"""
        # Determine destination dir and filename from given path
        destination_photo_dir, destination_photo_filename = os.path.split(destination_photo_path)
                
        # Set up a callback to display the photo once it has been taken
        @pyqtSlot(str)
        def on_capture(captured_image_path):
            # Remove this callback
            self.camera.photo_captured.disconnect(on_capture)
            # Rename the file
            os.rename(captured_image_path, destination_photo_path)
            # Display the photo
            self.display_image(destination_photo_path, label_area)
        self.camera.photo_captured.connect(on_capture)
        
        # Trigger the photo to be taken
        self.camera.trigger_photo.emit(destination_photo_dir)
        
    def read_gripper_calibration_config(self):
        """Update gripper calibration tab input boxes from JSON-formatted config file"""
        with open(self.config_file_path, "r") as jsonfile:
            self.config = json.load(jsonfile)
            self.gripper_calibration_start_x_textbox.setText(self.config.get('gripper_calibration_start_x', ''))
            self.gripper_calibration_start_y_textbox.setText(self.config.get('gripper_calibration_start_y', ''))
            self.camera_delta_x_textbox.setText(self.config.get('camera_delta_x', ''))
            self.camera_delta_y_textbox.setText(self.config.get('camera_delta_y', ''))
            self.gripper_delta_x_textbox.setText(self.config.get('gripper_delta_x', ''))
            self.gripper_delta_y_textbox.setText(self.config.get('gripper_delta_y', ''))
            self.gripper_calibration_angle_textbox.setText(self.config.get('gripper_calibration_angle', ''))
            self.gripper_calibration_pickup_z_textbox.setText(self.config.get('gripper_calibration_pickup_z', ''))
            self.gripper_calibration_spin_z_textbox.setText(self.config.get('gripper_calibration_spin_z', ''))
            self.motor_counts_x_per_px_x_textbox.setText(self.config.get('motor_counts_x_per_px_x', ''))
            self.motor_counts_y_per_px_y_textbox.setText(self.config.get('motor_counts_y_per_px_y', ''))
            self.camera_to_gripper_x_transform_textbox.setText(self.config.get('camera_to_gripper_x_transform', ''))
            self.camera_to_gripper_y_transform_textbox.setText(self.config.get('camera_to_gripper_y_transform', ''))
            self.camera_to_gripper_angle_transform_textbox.setText(self.config.get('camera_to_gripper_angle_transform', ''))
    
    def write_gripper_calibration_config(self):
        """Update self.config from gripper calibration tab input boxes and write it to a JSON-formatted config file"""
        # Update self.config from input boxes
        self.config['gripper_calibration_start_x'] = self.gripper_calibration_start_x_textbox.text()
        self.config['gripper_calibration_start_y'] = self.gripper_calibration_start_y_textbox.text()
        self.config['camera_delta_x'] = self.camera_delta_x_textbox.text()
        self.config['camera_delta_y'] = self.camera_delta_y_textbox.text()
        self.config['gripper_delta_x'] = self.gripper_delta_x_textbox.text()
        self.config['gripper_delta_y'] = self.gripper_delta_y_textbox.text()
        self.config['gripper_calibration_angle'] = self.gripper_calibration_angle_textbox.text()
        self.config['gripper_calibration_pickup_z'] = self.gripper_calibration_pickup_z_textbox.text()
        self.config['gripper_calibration_spin_z'] = self.gripper_calibration_spin_z_textbox.text()
        self.config['motor_counts_x_per_px_x'] = self.motor_counts_x_per_px_x_textbox.text()
        self.config['motor_counts_y_per_px_y'] = self.motor_counts_y_per_px_y_textbox.text()
        self.config['camera_to_gripper_x_transform'] = self.camera_to_gripper_x_transform_textbox.text()
        self.config['camera_to_gripper_y_transform'] = self.camera_to_gripper_y_transform_textbox.text()
        self.config['camera_to_gripper_angle_transform'] = self.camera_to_gripper_angle_transform_textbox.text()
        
        # Write self.config to a JSON-formatted config file
        with open(self.config_file_path, "w") as jsonfile:
            json.dump(self.config, jsonfile)


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