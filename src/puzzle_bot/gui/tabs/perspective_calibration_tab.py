import ast
import logging
import posixpath

import cv2 as cv
import numpy as np
from PyQt5.QtCore import pyqtSlot

from puzzle_bot.utils import open_image_undistorted_and_rotated, parse_int
from puzzle_bot.vision import calibrate_perspective, calibrate_rotation, correct_perspective, rotate_image

logger = logging.getLogger(__name__)


class PerspectiveCalibrationTab:
    def setup_perspective_calibration_tab(self) -> None:
        self.read_perspective_calibration_config()

        self.perspective_calibration_start_x_textbox.textChanged.connect(self.write_perspective_calibration_config)
        self.perspective_calibration_start_y_textbox.textChanged.connect(self.write_perspective_calibration_config)
        self.perspective_calibration_end_x_textbox.textChanged.connect(self.write_perspective_calibration_config)
        self.perspective_calibration_photo_directory_textbox.textChanged.connect(self.write_perspective_calibration_config)
        self.perspective_rotation_angle_textbox.textChanged.connect(self.write_perspective_calibration_config)
        self.perspective_starting_quad_textbox.textChanged.connect(self.write_perspective_calibration_config)
        self.perspective_corrected_quad_textbox.textChanged.connect(self.write_perspective_calibration_config)

        self.perspective_calibration_photo_directory_browse_button.clicked.connect(
            lambda: self.browse_for_directory(self.perspective_calibration_photo_directory_textbox)
        )

        @pyqtSlot()
        def perspective_calibration_move_start() -> None:
            x = parse_int(self.perspective_calibration_start_x_textbox.text())
            y = parse_int(self.perspective_calibration_start_y_textbox.text())
            z = parse_int(self.perspective_calibration_start_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.perspective_calibration_move_start_button.clicked.connect(perspective_calibration_move_start)

        @pyqtSlot()
        def perspective_calibration_move_end() -> None:
            x = parse_int(self.perspective_calibration_end_x_textbox.text())
            y = parse_int(self.perspective_calibration_start_y_textbox.text())
            z = parse_int(self.perspective_calibration_start_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.perspective_calibration_move_end_button.clicked.connect(perspective_calibration_move_end)

        @pyqtSlot()
        def take_perspective_calibration_start_photo() -> None:
            destination_photo_path = posixpath.join(self.get_perspective_calibration_batch_dir(), "start.jpg")
            self.take_photo(destination_photo_path, self.perspective_calibration_start_photo_label)

        self.perspective_calibration_take_start_photo_button.clicked.connect(take_perspective_calibration_start_photo)

        @pyqtSlot()
        def take_perspective_calibration_end_photo() -> None:
            destination_photo_path = posixpath.join(self.get_perspective_calibration_batch_dir(), "end.jpg")
            self.take_photo(destination_photo_path, self.perspective_calibration_end_photo_label)

        self.perspective_calibration_take_end_photo_button.clicked.connect(take_perspective_calibration_end_photo)

        @pyqtSlot()
        def run_perspective_calibration() -> None:
            start_photo_path = posixpath.join(self.get_perspective_calibration_batch_dir(), "start.jpg")
            end_photo_path = posixpath.join(self.get_perspective_calibration_batch_dir(), "end.jpg")
            camera_matrix, distortion_coefficients = self.get_camera_intrinsics()

            start_img = open_image_undistorted_and_rotated(start_photo_path, camera_matrix, distortion_coefficients)
            end_img = open_image_undistorted_and_rotated(end_photo_path, camera_matrix, distortion_coefficients)

            checker_rows = self.camera_calibration_checker_rows_spinbox.value()
            checker_columns = self.camera_calibration_checker_columns_spinbox.value()

            starting_quad, corrected_quad = calibrate_perspective(end_img, checker_columns, checker_rows)

            start_img = correct_perspective(start_img, starting_quad, corrected_quad, crop=True)
            end_img = correct_perspective(end_img, starting_quad, corrected_quad, crop=True)

            angle_degrees = calibrate_rotation(start_img, end_img)

            start_img = rotate_image(start_img, angle_degrees, crop=True)
            end_img = rotate_image(end_img, angle_degrees, crop=True)

            calibrated_start_photo_path = posixpath.join(self.get_perspective_calibration_batch_dir(), "start_calibrated.jpg")
            calibrated_end_photo_path = posixpath.join(self.get_perspective_calibration_batch_dir(), "end_calibrated.jpg")
            cv.imwrite(calibrated_start_photo_path, start_img)
            cv.imwrite(calibrated_end_photo_path, end_img)

            self.perspective_rotation_angle_textbox.setText(str(angle_degrees))
            self.perspective_starting_quad_textbox.setText(str(starting_quad.tolist()))
            self.perspective_corrected_quad_textbox.setText(str(corrected_quad.tolist()))

            self.display_image(calibrated_end_photo_path, self.perspective_calibration_corrected_end_photo_label)

        self.perspective_calibration_run_calibration_button.clicked.connect(run_perspective_calibration)

    def get_perspective_calibration_batch_dir(self):
        return self._resolve_batch_dir(
            self.config.get("perspective_calibration_photo_directory", ""),
            self.perspective_calibration_photo_batch_number_textbox,
            "Perspective",
        )

    def get_perspective_corrections(self):
        perspective_angle = float(self.perspective_rotation_angle_textbox.text())
        perspective_starting_quad = np.float32(ast.literal_eval(self.perspective_starting_quad_textbox.text()))
        perspective_corrected_quad = np.float32(ast.literal_eval(self.perspective_corrected_quad_textbox.text()))
        return perspective_angle, perspective_starting_quad, perspective_corrected_quad

    def read_perspective_calibration_config(self) -> None:
        config = self.config
        self.perspective_calibration_start_x_textbox.setText(config.get("perspective_calibration_start_x", ""))
        self.perspective_calibration_start_y_textbox.setText(config.get("perspective_calibration_start_y", ""))
        self.perspective_calibration_end_x_textbox.setText(config.get("perspective_calibration_end_x", ""))
        self.perspective_calibration_photo_directory_textbox.setText(config.get("perspective_calibration_photo_directory", ""))
        self.perspective_rotation_angle_textbox.setText(config.get("perspective_rotation_angle", ""))
        self.perspective_starting_quad_textbox.setText(config.get("perspective_starting_quad", ""))
        self.perspective_corrected_quad_textbox.setText(config.get("perspective_corrected_quad", ""))

    def write_perspective_calibration_config(self) -> None:
        config = self.config
        config["perspective_calibration_start_x"] = self.perspective_calibration_start_x_textbox.text()
        config["perspective_calibration_start_y"] = self.perspective_calibration_start_y_textbox.text()
        config["perspective_calibration_end_x"] = self.perspective_calibration_end_x_textbox.text()
        config["perspective_calibration_photo_directory"] = self.perspective_calibration_photo_directory_textbox.text()
        config["perspective_rotation_angle"] = self.perspective_rotation_angle_textbox.text()
        config["perspective_starting_quad"] = self.perspective_starting_quad_textbox.text()
        config["perspective_corrected_quad"] = self.perspective_corrected_quad_textbox.text()
        self.settings.save()
