import logging
import subprocess
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import NamedTuple

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot

logger = logging.getLogger(__name__)


class AdbResult(NamedTuple):
    ok: bool
    stdout: bytes


def timestamped_name(suffix: str) -> str:
    return datetime.now().strftime("%Y-%m-%d-%H%M%S") + suffix


class CameraBase(QThread):
    device_connected = pyqtSignal(bool)
    photo_captured = pyqtSignal(str)
    screenshot_captured = pyqtSignal(str)
    trigger_photo = pyqtSignal(str)
    trigger_screenshot = pyqtSignal(str)
    thread_closing = pyqtSignal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.trigger_photo.connect(self.capture_photo)
        self.trigger_screenshot.connect(self.capture_screenshot)
        self.thread_closing.connect(self.exit_thread)
        self._thread_exiting = False

    @pyqtSlot()
    def exit_thread(self) -> None:
        self._thread_exiting = True

    @pyqtSlot(str)
    def capture_photo(self, batch_dir: str) -> None:
        raise NotImplementedError

    @pyqtSlot(str)
    def capture_screenshot(self, screenshot_dir: str) -> None:
        raise NotImplementedError

    def _idle_until_closed(self) -> None:
        while not self._thread_exiting:
            self.msleep(100)


class GalaxyS24(CameraBase):
    def adb(self, command: str, delay_ms_after: int = 0, log_error: bool = True, cwd: str | None = None) -> AdbResult:
        logger.debug("[GalaxyS24.adb] running command: %s", command)
        results = subprocess.run(command, capture_output=True, shell=True, cwd=cwd, check=False)
        if results.returncode != 0:
            if log_error:
                logger.error("[GalaxyS24.adb] Error running %s: %s\n%s", command, results.stderr, results.stdout)
            return AdbResult(ok=False, stdout=results.stdout)
        self.msleep(delay_ms_after)
        return AdbResult(ok=True, stdout=results.stdout)

    @pyqtSlot(str)
    def capture_photo(self, batch_dir: str) -> None:
        if not Path(batch_dir).exists():
            logger.error("[GalaxyS24.capture_photo] Photo dir does not exist: %s", batch_dir)
            return

        # Disable app auto-rotate
        logger.debug("[GalaxyS24.capture_photo] Disabling auto-rotate...")
        if not self.adb("adb shell settings put system accelerometer_rotation 0").ok:
            return

        # Tap a non-sensitive area to keep the camera app awake
        logger.debug("[GalaxyS24.capture_photo] Tapping screen to keep alive...")
        if not self.adb("adb shell input tap 1 950").ok:
            return

        # Check whether the camera app is in the foreground; start it if not
        if self.adb("adb shell dumpsys activity activities | grep mFocusedWindow | grep camera", log_error=False).ok:
            logger.debug("[GalaxyS24.capture_photo] Camera app is running...")
        else:
            logger.debug("[GalaxyS24.capture_photo] Starting camera app...")
            # Wake the device and get to the home screen
            if not self.adb("adb shell input keyevent KEYCODE_WAKEUP", 1000).ok:
                return
            if not self.adb("adb shell input touchscreen swipe 540 2332 540 20", 2000).ok:
                return
            if not self.adb("adb shell input keyevent KEYCODE_HOME", 1000).ok:
                return

            # NOTE: the camera app NEEDS the following settings from its last
            # session: PRO mode, 12MP, ISO 100 (locked), 1/15 speed (locked),
            # manual focus (locked), white balance 5300K (locked), flash OFF,
            # 3:4 aspect ratio, no timer, intelligent optimization: minimum.
            if not self.adb("adb shell am start -n com.sec.android.app.camera/com.sec.android.app.camera.Camera", 2000).ok:
                return

            # Select the 5X "Super Telescopic" lens
            if not self.adb("adb shell input tap 676 1433", 2000).ok:
                return

        # Tap the shutter button and wait for the photo
        logger.debug("[GalaxyS24.capture_photo] Taking photo...")
        if not self.adb("adb shell input tap 534 2006", 1000).ok:
            return

        # Find the most recently taken photo
        latest = self.adb("adb shell ls -clr sdcard/DCIM/Camera/*.jpg | tail -n 1 | awk '{print $NF}'")
        if not latest.ok:
            return
        remote_photo_path = latest.stdout.decode("ascii").strip().strip("'")
        remote_file_name = PurePosixPath(remote_photo_path).name

        # Download the photo
        local_photo_path = str(Path(batch_dir) / remote_file_name)
        if not self.adb(f"adb pull -a {remote_photo_path}", cwd=batch_dir).ok:
            return

        # Delete photos so the directory doesn't fill up (listing gets slow)
        self.adb("adb shell rm sdcard/DCIM/Camera/*.jpg")

        self.photo_captured.emit(local_photo_path)

    @pyqtSlot(str)
    def capture_screenshot(self, screenshot_dir: str) -> None:
        if not Path(screenshot_dir).exists():
            logger.error("[GalaxyS24.capture_screenshot] Screenshot dir does not exist: %s", screenshot_dir)
            return

        screenshot_path = str(Path(screenshot_dir) / timestamped_name(".png"))
        logger.debug("[GalaxyS24.capture_screenshot] Taking screenshot...")
        if not self.adb(f"adb exec-out screencap -p > {screenshot_path}").ok:
            return

        self.screenshot_captured.emit(screenshot_path)

    def run(self) -> None:
        results = subprocess.run("adb shell :", capture_output=True, shell=True, check=False)
        if results.returncode != 0:
            logger.error("[GalaxyS24.run] Could not connect over ADB: %s", results.stderr)
            self.device_connected.emit(False)
            return
        self.device_connected.emit(True)
        self._idle_until_closed()
