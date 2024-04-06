#!/usr/bin/env python3

import sys, os, threading, time, signal
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, QThread

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from utils import parse_int
from serial_driver import ClearCore, DummyClearCore

APPLICATION_NAME = "Puzzlin' Pete"

# QT Class inheritance code from
# https://nitratine.net/blog/post/how-to-import-a-pyqt5-ui-file-in-a-python-gui/

# QT Thread code from
# https://gist.github.com/majabojarska/952978eb83bcc19653be138525c4b9da
# https://stackoverflow.com/questions/6783194/background-thread-with-qthread-in-pyqt

class Ui(QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi(os.path.join("qt", "application.ui"), self) # Load the .ui file
        self.setWindowTitle(APPLICATION_NAME) # Set window name (works on MS Windows)
        self.show() # Show the GUI
        
        # Setup tabs
        self.setup_move_robot_tab()
    
    def closeEvent(self, event):
        logging.debug(f"[Ui.closeEvent] closing main window")
        
        # Close the ClearCore serial thread
        self.clearcore.thread_closing.emit() # Tell the thread to exit its run loop
        self.clearcore.wait() # Wait for the thread to close (also closes the serial port)
        
        # Continue closing the app
        event.accept()


    ################################################################################################
    # MOVE ROBOT TAB

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

    def setup_move_robot_tab(self):
        """Configure UI on the MOVE ROBOT tab"""
        # Configure ClearCore output to always scroll to the bottom
        scrollbar = self.clearcore_serial_output_textarea.verticalScrollBar()
        scrollbar.rangeChanged.connect(lambda minVal, maxVal: scrollbar.setValue(scrollbar.maximum()))
        
        # Set all inputs as unavailable until we're connected to the ClearCore
        self.set_clearcore_availability(False)
        self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Opening ClearCore...</font><br/>")
        
        # Connect to the ClearCore over serial
        #self.clearcore = ClearCore(self.process_clearcore_response)
        self.clearcore = DummyClearCore(parent=self)
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
    window = Ui() # Create an instance of our class
    
    # Set up a signal handler to quit on CTRL-C, and use a timer to let the interpreter run now and then
    signal.signal(signal.SIGINT, sigint_handler)
    timer = QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    
    # Start the application
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()