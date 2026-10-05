import logging
import os
import posixpath
from typing import Any

from PyQt5 import uic
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import QLineEdit, QMainWindow

from puzzle_bot.gui.helpers import UiHelpers
from puzzle_bot.gui.tabs import (
    CameraCalibrationTab,
    ComputeSolutionTab,
    GripperCalibrationTab,
    MovePiecesTab,
    MoveRobotTab,
    PerspectiveCalibrationTab,
    SerpentinePhotosTab,
)
from puzzle_bot.hardware import CameraBase, ClearCore, FakeCamera, FakeClearCore, FakeGripper, GalaxyS24, Gripper, SerialBase
from puzzle_bot.paths import latest_batch_number, package_path
from puzzle_bot.settings import RobotParams, Settings

logger = logging.getLogger(__name__)

UI_FILE = package_path("gui", "application.ui")

WORKER_THREADS = ("clearcore", "gripper", "camera", "camera_calibration", "gripper_calibration", "solver")


class MainWindow(
    UiHelpers,
    MoveRobotTab,
    SerpentinePhotosTab,
    CameraCalibrationTab,
    PerspectiveCalibrationTab,
    GripperCalibrationTab,
    ComputeSolutionTab,
    MovePiecesTab,
    QMainWindow,
):
    def __init__(self, settings: Settings, demo: bool = False) -> None:
        super().__init__()
        uic.loadUi(str(UI_FILE), self)
        self.setWindowTitle(f"{settings.app_title} 🧩" + ("  [DEMO]" if demo else ""))
        self.show()

        self.settings = settings
        self.demo = demo
        self.robot: RobotParams = settings.robot_params()
        self.demo_config = settings.demo_config()

        # Worker threads, created by the tab setup methods
        self.clearcore: SerialBase | None = None
        self.gripper: SerialBase | None = None
        self.camera: CameraBase | None = None
        self.camera_calibration = None
        self.gripper_calibration = None
        self.solver = None

        if self.demo and self.demo_config.auto_create_dirs:
            self._create_demo_directories()

        self.setup_move_robot_tab()
        self.setup_serpentine_photos_tab()
        self.setup_camera_calibration_tab()
        self.setup_perspective_calibration_tab()
        self.setup_gripper_calibration_tab()
        self.setup_compute_solution_tab()
        self.setup_perform_moves_tab()

    @property
    def config(self) -> dict[str, Any]:
        return self.settings.ui

    def _create_demo_directories(self) -> None:
        for directory in [*self.settings.ui_directories, self.demo_config.photos_dir]:
            directory.mkdir(parents=True, exist_ok=True)

    def _resolve_batch_dir(self, photo_directory: str, batch_number_textbox: QLineEdit, label: str) -> str | None:
        if not photo_directory or not posixpath.isdir(photo_directory):
            logger.error("[MainWindow] %s photo directory does not exist at: %s", label, photo_directory)
            return None

        batch_number_text = batch_number_textbox.text().strip()
        if batch_number_text and batch_number_text.isnumeric():
            return posixpath.join(photo_directory, str(int(batch_number_text)))

        current_batch_number = latest_batch_number(photo_directory) + 1
        batch_dir = posixpath.join(photo_directory, str(current_batch_number))
        os.makedirs(batch_dir)
        batch_number_textbox.setText(str(current_batch_number))
        return batch_dir

    def _create_motion_devices(self) -> tuple[SerialBase, SerialBase]:
        if self.demo:
            delay = self.demo_config.response_delay_ms
            return FakeClearCore(parent=self, response_delay_ms=delay), FakeGripper(parent=self, response_delay_ms=delay)
        clearcore_cfg = self.settings.serial_config("clearcore")
        gripper_cfg = self.settings.serial_config("gripper")
        return (
            ClearCore(parent=self, port_id=clearcore_cfg.port_id, baud_rate=clearcore_cfg.baud_rate),
            Gripper(parent=self, port_id=gripper_cfg.port_id, baud_rate=gripper_cfg.baud_rate),
        )

    def _create_camera(self) -> CameraBase:
        if self.demo:
            return FakeCamera(parent=self, photos_dir=self.demo_config.photos_dir)
        return GalaxyS24(parent=self)

    def closeEvent(self, event: QCloseEvent) -> None:
        logger.debug("[MainWindow.closeEvent] closing main window")
        for name in WORKER_THREADS:
            thread = getattr(self, name)
            if thread is not None:
                logger.debug("[MainWindow.closeEvent] closing %s thread", name)
                thread.thread_closing.emit()
                thread.wait()
        event.accept()
