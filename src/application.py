#!/usr/bin/env python3

import sys, os
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSlot

APPLICATION_NAME = "Puzzlin' Pete"

# QT Class inheritance code from
# https://nitratine.net/blog/post/how-to-import-a-pyqt5-ui-file-in-a-python-gui/

class Ui(QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi(os.path.join("qt", "application.ui"), self) # Load the .ui file
        self.setWindowTitle(APPLICATION_NAME) # Set window name (works on MS Windows)
        self.show() # Show the GUI
        
        # Connect all the button and menu actions (it's faster than creating slots in the qt designer gui)
        #self.actionExit.triggered.connect(lambda: self.exit())
        # import pdb
        # pdb.set_trace()

    def exit():
        self.close()

    def say_hello(self):
        self.label1.setText("hieee!")

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
    app.exec_() # Start the application

if __name__ == "__main__":
    main()