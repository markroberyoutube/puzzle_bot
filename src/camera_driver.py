import time, sys, os, logging, subprocess
from datetime import datetime
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, QThread, QEventLoop

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

class GalaxyS24(QThread):
    """"Samsung Galaxy S24 driver class

    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    """
    
    # Define QT Signals for inter-process communication
    device_connected = pyqtSignal(bool) # This is emitted on run() with a bool (whether adb shell could connect to device)
    photo_captured = pyqtSignal(str) # This is emitted with the file path after a photo is taken
    screenshot_captured = pyqtSignal(str) # This is emitted with the file path after a screenshot is captured
    trigger_photo = pyqtSignal(str) # Emit to this to take a photo
    trigger_screenshot = pyqtSignal(str) # Emit to this to take a screenshot
    thread_closing = pyqtSignal() # Emit to this when you want the thread to exit
    
    def __init__(self, parent):
        # Superclass initializer
        QThread.__init__(self, parent)
        
        # Connect signals for inter-process communication
        self.trigger_screenshot.connect(self.capture_screenshot)
        self.trigger_photo.connect(self.capture_photo)
        self.thread_closing.connect(self.exit_thread)
        
        # Set to True to cause the Thread's main run loop to exit
        self._thread_exiting = False

    @pyqtSlot()
    def exit_thread(self):
        """Emit to this slot to stop this thread."""
        # Ask the main run loop to exit
        self._thread_exiting = True

    def adb(self, command, delay_ms_after=0, log_error=True):
        """Issue an adb command and return a (retcode, stdout) tuple"""
        logging.debug(f"[GalaxyS24.adb] running command: {command}")
        results = subprocess.run(command, capture_output=True, shell=True)
        if results.returncode != 0:
            if log_error:
                logging.error(f"[GalaxyS24.adb] Error running {command}: {results.stderr}\n{results.stdout}")
            return (False, results.stdout)
        else:
            self.msleep(delay_ms_after) # Delay using QThread's sleep
            return (True, results.stdout)

    @pyqtSlot(str)
    def capture_photo(self, batch_dir=None):
        """Capture a photo and save it to batch_dir with a timestamped filename"""
        
        # Ensure parent dir exists
        if not os.path.exists(batch_dir):
            logging.error(f"[GalaxyS24.capture_photo] Photo dir does not exist: {batch_dir}")
            return
        
        # Disable app auto-rotate
        logging.debug("[GalaxyS24.capture_photo] Disabling auto-rotate...")
        if not self.adb("adb shell settings put system accelerometer_rotation 0", 0)[0]: return
        
        # Tap the screen in a non-sensitive area to keep camera app awake if it's running
        logging.debug("[GalaxyS24.capture_photo] Tapping screen to keep alive...")
        if not self.adb("adb shell input tap 1 950", 0)[0]: return
        
        # Check to see if the camera app is running in the foreground
        retcode, stdout = self.adb("adb shell dumpsys activity activities | grep mFocusedWindow | grep camera", 0, False)
        # If camera app is not running, start it up
        if retcode:
            logging.debug("[GalaxyS24.capture_photo] Camera app is running...")
        else:
            logging.debug("[GalaxyS24.capture_photo] Starting camera app...")
            # Wake the device and get to the home screen
            if not self.adb("adb shell input keyevent KEYCODE_WAKEUP", 1000)[0]: return
            if not self.adb("adb shell input touchscreen swipe 540 2332 540 20", 2000)[0]: return
            if not self.adb("adb shell input keyevent KEYCODE_HOME", 1000)[0]: return

            # Start the camera app. 
            # NOTE: It NEEDS to have been on the following settings the last time it was opened:
            # PRO mode, 12Mp, ISO 100 (locked/yellow), Speed 1/30 (locked/yellow), 
            # Focus manual (locked/yellow), White balance 5300k (locked/yellow),
            # Flash on (icon is yellow), 3:4 aspect ratio, Matrix metering, no timer, 
            # Intelligent optimization: maximum
            if not self.adb("adb shell am start -n com.sec.android.app.camera/com.sec.android.app.camera.Camera", 2000)[0]: return
        
            # Select the 5X "Super Telescopic" lens 
            if not self.adb("adb shell input tap 676 1433", 2000)[0]: return
        
            # Click "Focus: Manual"
            if not self.adb("adb shell input tap 768 1616", 1000)[0]: return

            # Click "Multi" to enable a multi-point auto focus (and wait 3 secs for camera to focus)
            if not self.adb("adb shell input tap 536 1610", 3000)[0]: return
        
            # Click "Manual" to disable auto focus
            if not self.adb("adb shell input tap 799 1626", 1000)[0]: return

        # Take the photo (Tap the shutter button) and wait 2 seconds for the camera to take the photo
        logging.debug("[GalaxyS24.capture_photo] Taking photo...")
        if not self.adb("adb shell input tap 534 2006", 2000)[0]: return

        # Find the most recently taken photo
        retcode, stdout = self.adb("adb shell ls -clr /sdcard/DCIM/Camera/*.jpg | tail -n 1 | awk \'{print $NF}\'", 0)
        if not retcode: return
        remote_photo_path = stdout.decode('ascii').strip()
        remote_file_name = os.path.split(remote_photo_path)[-1]

        # Download that photo
        local_photo_path = os.path.join(batch_dir, remote_file_name)
        retcode, stdout = self.adb(f"adb pull -a '{remote_photo_path}' '{local_photo_path}'", 0)
        if not retcode: return
            
        # Let other threads know a new photo has been captured
        self.photo_captured.emit(local_photo_path)
    
    @pyqtSlot(str)
    def capture_screenshot(self, screenshot_dir):
        """Capture a screenshot and save it to screenshot_dir with a timestamped filename"""
        
        # Ensure screenshot_dir exists
        if not os.path.exists(screenshot_dir):
            logging.error(f"[GalaxyS24.capture_screenshot] Screenshot dir does not exist: {screenshot_dir}")
            return
        
        # Capture the screenshot
        screenshot_filename = datetime.now().strftime("%Y-%m-%d-%H%M%S") + ".png"
        screenshot_path = os.path.join(screenshot_dir, screenshot_filename)
        logging.debug("[GalaxyS24.capture_screenshot] Taking screenshot...")
        if not self.adb(f"adb exec-out screencap -p > {screenshot_path}", 0)[0]: return

        # Let other threads know a new screenshot has been captured
        self.screenshot_captured.emit(screenshot_path)

    def run(self):
        """Main thread loop"""
        
        # Ensure ADB can speak to the GalaxyS24 by running a shell no-op command (":")
        results = subprocess.run("adb shell :", capture_output=True, shell=True)
        if results.returncode != 0:
            logging.error(f"[GalaxyS24.run] Could not connect over ADB: {results.stderr}")
            self.device_connected.emit(False)
            return
        else:
            self.device_connected.emit(True)

        # Currently the run loop does nothing other than sleep
        while (not self._thread_exiting):
            self.msleep(100) # Delay 100 ms using QThread's version of sleep

