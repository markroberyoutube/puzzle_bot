import logging
import shutil
from collections.abc import Iterator
from datetime import datetime
from itertools import cycle
from pathlib import Path

import cv2 as cv
import numpy as np
from PyQt5.QtCore import QObject, QTimer, pyqtSlot

from puzzle_bot.hardware.camera import CameraBase
from puzzle_bot.hardware.serial import SerialBase

logger = logging.getLogger(__name__)

# Jog distances (in pulses), matching Dummy-MotionController.ino.
MOTOR_COUNTS_PER_INCH = 6523.748
JOG_DISTANCE_XY = int(6.0 * MOTOR_COUNTS_PER_INCH)
JOG_DISTANCE_Z = int(1.0 * MOTOR_COUNTS_PER_INCH)

# command -> (axis index, delta, response)
CLEARCORE_JOGS: dict[str, tuple[int, int, str]] = {
    "d": (0, JOG_DISTANCE_XY, "SUCCESS: moveRelativeX"),
    "a": (0, -JOG_DISTANCE_XY, "SUCCESS: moveRelativeX"),
    "w": (1, JOG_DISTANCE_XY, "SUCCESS: moveRelativeY"),
    "s": (1, -JOG_DISTANCE_XY, "SUCCESS: moveRelativeY"),
    "o": (2, JOG_DISTANCE_Z, "SUCCESS: moveRelativeZ"),
    "l": (2, -JOG_DISTANCE_Z, "SUCCESS: moveRelativeZ"),
}

# Value reported by the fake linear encoder, inside the healthy piece range.
FAKE_ENCODER_VALUE = 1429
FAKE_ANALOG_ENCODER_VALUE = 219

PHOTO_SUFFIXES = frozenset({".jpg", ".jpeg", ".png"})


def _unique_name(suffix: str) -> str:
    return datetime.now().strftime("%Y-%m-%d-%H%M%S-%f") + suffix


class _FakeSerialBase(SerialBase):
    def __init__(self, parent: QObject | None, name: str, response_delay_ms: int) -> None:
        super().__init__(parent, name=name, port_id="", baud_rate=0)
        self._response_delay_ms = response_delay_ms

    def run(self) -> None:
        self.port_opened.emit(True)
        while not self._thread_exiting:
            self.msleep(50)

    def _respond(self, line: str) -> None:
        if self._response_delay_ms <= 0:
            self.line_received.emit(line)
        else:
            QTimer.singleShot(self._response_delay_ms, lambda: self.line_received.emit(line))

    def _invalid(self, command: str) -> None:
        self._respond(f"ERROR: Invalid command: {command}")


class FakeClearCore(_FakeSerialBase):
    def __init__(self, parent: QObject | None = None, response_delay_ms: int = 200) -> None:
        super().__init__(parent, name="FakeClearCore", response_delay_ms=response_delay_ms)
        self._position = [0, 0, 0]

    @pyqtSlot(str)
    def writeline(self, line: str) -> None:
        command = line.strip()
        head, _, arg = command.partition(" ")

        if head in CLEARCORE_JOGS:
            axis, delta, response = CLEARCORE_JOGS[head]
            self._position[axis] += delta
            self._respond(response)
        elif head == "h":
            self._position = [0, 0, 0]
            self._respond("SUCCESS: homeAllAxes")
        elif head in ("x", "y", "z"):
            try:
                self._position["xyz".index(head)] = int(arg)
            except ValueError:
                self._invalid(command)
                return
            self._respond(f"SUCCESS: moveAbsolute{head.upper()}")
        elif head == "m":
            try:
                x, y, z = (int(v) for v in arg.split(","))
            except ValueError:
                self._invalid(command)
                return
            self._position = [x, y, z]
            self._respond("SUCCESS: moveAbsoluteXYZ")
        elif head == "0":
            self._respond("SUCCESS: vacuumOff")
        elif head == "1":
            self._respond("SUCCESS: vacuumOn")
        elif head == "p":
            x, y, z = self._position
            self._respond(f"SUCCESS: printCurrentPosXYZ {x}, {y}, {z}")
        else:
            self._invalid(command)


class FakeGripper(_FakeSerialBase):
    def __init__(self, parent: QObject | None = None, response_delay_ms: int = 200) -> None:
        super().__init__(parent, name="FakeGripper", response_delay_ms=response_delay_ms)
        self._rotation_position = 0

    @pyqtSlot(str)
    def writeline(self, line: str) -> None:
        command = line.strip()
        head, _, arg = command.partition(" ")

        if head == "p":
            self._respond(f"SUCCESS: printMotorPosition {self._rotation_position} {self._rotation_position}")
        elif head == "h":
            self._rotation_position = 0
            self._respond("SUCCESS: homeMotor")
        elif head == "r":
            try:
                position = int(arg)
            except ValueError:
                self._invalid(command)
                return
            if not 0 <= position <= 0xFFFF:
                self._respond(f"ERROR: motorCounts outside of valid range (0-0xFFFF): {position}")
                return
            self._rotation_position = position
            self._respond("SUCCESS: rotateMotorAbsolute")
        elif head == "e":
            if not arg.lstrip("-").isdigit():
                self._invalid(command)
                return
            self._respond("SUCCESS: moveEye")
        elif head == "a":
            self._respond(f"SUCCESS: printEncoderAnalogValue {FAKE_ANALOG_ENCODER_VALUE}")
        elif head == "d":
            self._respond(f"SUCCESS: printEncoderDigitalValue {FAKE_ENCODER_VALUE}")
        elif head == "1":
            self._respond("SUCCESS: setLightOn")
        elif head == "0":
            self._respond("SUCCESS: setLightOff")
        else:
            self._invalid(command)


class FakeCamera(CameraBase):
    def __init__(self, parent: QObject | None, photos_dir: Path) -> None:
        super().__init__(parent)
        self._photos_dir = photos_dir
        self._sources = self._load_sources()

    def _load_sources(self) -> Iterator[Path] | None:
        photos = sorted(p for p in self._photos_dir.glob("*") if p.suffix.lower() in PHOTO_SUFFIXES)
        return cycle(photos) if photos else None

    @staticmethod
    def synthesize_photo(path: Path) -> None:
        rng = np.random.default_rng()
        img = np.zeros((3000, 4000, 3), dtype=np.uint8)
        size = 420
        for row in range(3):
            for col in range(4):
                cx = 400 + col * 900 + int(rng.integers(-40, 40))
                cy = 400 + row * 850 + int(rng.integers(-40, 40))
                # A piece silhouette: body plus a circular tab and a blank.
                cv.rectangle(img, (cx - size // 2, cy - size // 2), (cx + size // 2, cy + size // 2), (235, 235, 235), -1)
                cv.circle(img, (cx, cy - size // 2 - 45), 90, (235, 235, 235), -1)
                cv.circle(img, (cx + size // 2 + 10, cy), 80, (0, 0, 0), -1)
                cv.circle(img, (cx, cy), 3, (0, 0, 255), -1)
        cv.imwrite(str(path), img)

    @pyqtSlot(str)
    def capture_photo(self, batch_dir: str) -> None:
        directory = Path(batch_dir)
        if not directory.exists():
            logger.error("[FakeCamera.capture_photo] Photo dir does not exist: %s", batch_dir)
            return

        local_photo_path = directory / _unique_name(".jpg")
        if self._sources is None:
            self._sources = self._load_sources()

        if self._sources is not None:
            shutil.copyfile(next(self._sources), local_photo_path)
        else:
            self.synthesize_photo(local_photo_path)

        self.photo_captured.emit(str(local_photo_path))

    @pyqtSlot(str)
    def capture_screenshot(self, screenshot_dir: str) -> None:
        directory = Path(screenshot_dir)
        if not directory.exists():
            logger.error("[FakeCamera.capture_screenshot] Screenshot dir does not exist: %s", screenshot_dir)
            return
        screenshot_path = directory / _unique_name(".png")
        img = np.full((800, 1200, 3), (40, 30, 25), dtype=np.uint8)
        cv.putText(img, "PUZZLE BOT DEMO", (300, 420), cv.FONT_HERSHEY_SIMPLEX, 2.5, (200, 200, 200), 4)
        cv.imwrite(str(screenshot_path), img)
        self.screenshot_captured.emit(str(screenshot_path))

    def run(self) -> None:
        self.device_connected.emit(True)
        self._idle_until_closed()
