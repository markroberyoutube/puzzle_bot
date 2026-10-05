import json
import logging
import os
import posixpath

from PyQt5 import QtTest
from PyQt5.QtCore import pyqtSlot

from puzzle_bot.utils import parse_int
from puzzle_bot.vision import GripperCalibration

logger = logging.getLogger(__name__)


class GripperCalibrationTab:
    def setup_gripper_calibration_tab(self) -> None:
        self.read_gripper_calibration_config()

        self.gripper_calibration_start_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_start_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_delta_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_delta_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_delta_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_delta_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_angle_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_pickup_z_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_spin_z_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.gripper_calibration_photo_directory_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.motor_counts_x_per_px_x_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.motor_counts_y_per_px_y_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_to_gripper_x_transform_textbox.textChanged.connect(self.write_gripper_calibration_config)
        self.camera_to_gripper_y_transform_textbox.textChanged.connect(self.write_gripper_calibration_config)

        self.gripper_calibration_photo_directory_browse_button.clicked.connect(
            lambda: self.browse_for_directory(self.gripper_calibration_photo_directory_textbox)
        )

        @pyqtSlot()
        def gripper_calibration_move_start() -> None:
            x = parse_int(self.gripper_calibration_start_x_textbox.text())
            y = parse_int(self.gripper_calibration_start_y_textbox.text())
            z = parse_int(self.gripper_calibration_start_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.gripper_calibration_move_start_button.clicked.connect(gripper_calibration_move_start)

        @pyqtSlot()
        def gripper_calibration_move_camera_delta() -> None:
            x = parse_int(self.camera_delta_x_textbox.text())
            y = parse_int(self.camera_delta_y_textbox.text())
            z = parse_int(self.camera_delta_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.move_camera_delta_button.clicked.connect(gripper_calibration_move_camera_delta)

        @pyqtSlot()
        def gripper_calibration_move_gripper_delta() -> None:
            x = parse_int(self.gripper_delta_x_textbox.text())
            y = parse_int(self.gripper_delta_y_textbox.text())
            z = parse_int(self.gripper_delta_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.move_gripper_delta_button.clicked.connect(gripper_calibration_move_gripper_delta)

        @pyqtSlot()
        def gripper_calibration_rotate_target() -> None:
            angle = parse_int(self.gripper_calibration_angle_textbox.text())
            pickup_x = parse_int(self.gripper_delta_x_textbox.text())
            pickup_y = parse_int(self.gripper_delta_y_textbox.text())
            pickup_z = parse_int(self.gripper_calibration_pickup_z_textbox.text())
            spin_z = parse_int(self.gripper_calibration_spin_z_textbox.text())

            # Pick up the target, rotate it, put it back down
            self.send_gripper_command("r 0", blocking=True)
            self.pickup_piece(pickup_x, pickup_y, pickup_z, spin_z)
            self.send_gripper_command(f"r {angle}", blocking=True)
            self.putdown_piece(pickup_x, pickup_y, pickup_z, spin_z)

            # Move back to the start position for the final photo
            x = parse_int(self.gripper_calibration_start_x_textbox.text())
            y = parse_int(self.gripper_calibration_start_y_textbox.text())
            z = parse_int(self.gripper_calibration_start_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.gripper_calibration_rotate_button.clicked.connect(gripper_calibration_rotate_target)

        @pyqtSlot()
        def take_gripper_calibration_start_photo() -> None:
            destination_photo_path = posixpath.join(self.get_gripper_calibration_batch_dir(), "start.jpg")
            self.take_photo(destination_photo_path, self.gripper_calibration_start_photo_label)
            x = parse_int(self.gripper_calibration_start_x_textbox.text())
            y = parse_int(self.gripper_calibration_start_y_textbox.text())
            z = parse_int(self.gripper_calibration_start_z_textbox.text())
            self.update_gripper_calibration_info(dict(start_x=x, start_y=y, start_z=z))

        self.gripper_calibration_take_start_photo_button.clicked.connect(take_gripper_calibration_start_photo)

        @pyqtSlot()
        def take_camera_delta_photo() -> None:
            destination_photo_path = posixpath.join(self.get_gripper_calibration_batch_dir(), "camera_delta.jpg")
            self.take_photo(destination_photo_path, self.camera_delta_photo_label)
            x = parse_int(self.camera_delta_x_textbox.text())
            y = parse_int(self.camera_delta_y_textbox.text())
            z = parse_int(self.camera_delta_z_textbox.text())
            self.update_gripper_calibration_info(dict(camera_delta_x=x, camera_delta_y=y, camera_delta_z=z))

        self.take_camera_delta_photo_button.clicked.connect(take_camera_delta_photo)

        @pyqtSlot()
        def take_gripper_delta_photo() -> None:
            destination_photo_path = posixpath.join(self.get_gripper_calibration_batch_dir(), "gripper_delta.jpg")
            self.take_photo(destination_photo_path, self.gripper_delta_photo_label)
            x = parse_int(self.gripper_delta_x_textbox.text())
            y = parse_int(self.gripper_delta_y_textbox.text())
            z = parse_int(self.gripper_delta_z_textbox.text())
            angle = parse_int(self.gripper_calibration_angle_textbox.text())
            self.update_gripper_calibration_info(dict(gripper_delta_x=x, gripper_delta_y=y, gripper_delta_z=z, gripper_angle=angle))

        self.take_gripper_delta_photo_button.clicked.connect(take_gripper_delta_photo)

        @pyqtSlot()
        def run_gripper_calibration() -> None:
            @pyqtSlot(list, list)
            def on_results(motor_counts_per_px, camera_to_gripper_transform):
                self.gripper_calibration.gripper_calibration_results_ready.disconnect(on_results)
                self.motor_counts_x_per_px_x_textbox.setText(str(round(motor_counts_per_px[0], 4)))
                self.motor_counts_y_per_px_y_textbox.setText(str(round(motor_counts_per_px[1], 4)))
                self.camera_to_gripper_x_transform_textbox.setText(str(round(camera_to_gripper_transform[0], 4)))
                self.camera_to_gripper_y_transform_textbox.setText(str(round(camera_to_gripper_transform[1], 4)))

            self.gripper_calibration.gripper_calibration_results_ready.connect(on_results)

            gripper_calibration_batch_directory = self.get_gripper_calibration_batch_dir()

            camera_matrix, distortion_coefficients = self.get_camera_intrinsics()
            perspective_angle, perspective_starting_quad, perspective_corrected_quad = self.get_perspective_corrections()

            self.gripper_calibration.trigger_gripper_calibration.emit(
                gripper_calibration_batch_directory,
                camera_matrix,
                distortion_coefficients,
                perspective_angle,
                perspective_starting_quad.tolist(),
                perspective_corrected_quad.tolist(),
            )

        self.gripper_calibration_run_calibration_button.clicked.connect(run_gripper_calibration)

        self.gripper_calibration = GripperCalibration(parent=self)
        self.gripper_calibration.start()

    def get_gripper_calibration_batch_dir(self):
        return self._resolve_batch_dir(
            self.config.get("gripper_calibration_photo_directory", ""),
            self.gripper_calibration_photo_batch_number_textbox,
            "Gripper",
        )

    def update_gripper_calibration_info(self, data_dict: dict) -> None:
        gripper_calibration_info_filepath = posixpath.join(
            self.get_gripper_calibration_batch_dir(),
            "gripper_calibration_info.json",
        )

        gripper_calibration_info = {}
        if gripper_calibration_info_filepath and posixpath.exists(gripper_calibration_info_filepath):
            with open(gripper_calibration_info_filepath) as jsonfile:
                gripper_calibration_info = json.load(jsonfile)

        gripper_calibration_info.update(data_dict)

        with open(gripper_calibration_info_filepath, "w") as jsonfile:
            json.dump(gripper_calibration_info, jsonfile)

    def pickup_piece(self, pickup_x: int, pickup_y: int, pickup_z: int, travel_z: int) -> None:
        self.send_clearcore_command("1", blocking=True, update_position=False)
        self.send_clearcore_command(f"m {pickup_x},{pickup_y},{pickup_z}", blocking=True, update_position=True)
        # Let the vacuum form
        QtTest.QTest.qWait(1000)
        self.send_clearcore_command(f"m {pickup_x},{pickup_y},{travel_z}", blocking=True, update_position=True)

    def putdown_piece(self, putdown_x: int, putdown_y: int, putdown_z: int, travel_z: int) -> None:
        self.send_clearcore_command(f"m {putdown_x},{putdown_y},{putdown_z}", blocking=True, update_position=True)
        self.send_clearcore_command("0", blocking=True, update_position=False)
        # Let the vacuum purge
        QtTest.QTest.qWait(1000)
        self.send_clearcore_command(f"m {putdown_x},{putdown_y},{travel_z}", blocking=True, update_position=True)

    def take_photo(self, destination_photo_path: str, label_area) -> None:
        destination_photo_dir, _ = posixpath.split(destination_photo_path)

        @pyqtSlot(str)
        def on_capture(captured_image_path: str) -> None:
            self.camera.photo_captured.disconnect(on_capture)
            os.rename(captured_image_path, destination_photo_path)
            self.display_image(destination_photo_path, label_area)

        self.camera.photo_captured.connect(on_capture)
        self.camera.trigger_photo.emit(destination_photo_dir)

    def read_gripper_calibration_config(self) -> None:
        config = self.config
        self.gripper_calibration_start_x_textbox.setText(config.get("gripper_calibration_start_x", ""))
        self.gripper_calibration_start_y_textbox.setText(config.get("gripper_calibration_start_y", ""))
        self.camera_delta_x_textbox.setText(config.get("camera_delta_x", ""))
        self.camera_delta_y_textbox.setText(config.get("camera_delta_y", ""))
        self.gripper_delta_x_textbox.setText(config.get("gripper_delta_x", ""))
        self.gripper_delta_y_textbox.setText(config.get("gripper_delta_y", ""))
        self.gripper_calibration_angle_textbox.setText(config.get("gripper_calibration_angle", ""))
        self.gripper_calibration_pickup_z_textbox.setText(config.get("gripper_calibration_pickup_z", ""))
        self.gripper_calibration_spin_z_textbox.setText(config.get("gripper_calibration_spin_z", ""))
        self.gripper_calibration_photo_directory_textbox.setText(config.get("gripper_calibration_photo_directory", ""))
        self.motor_counts_x_per_px_x_textbox.setText(config.get("motor_counts_x_per_px_x", ""))
        self.motor_counts_y_per_px_y_textbox.setText(config.get("motor_counts_y_per_px_y", ""))
        self.camera_to_gripper_x_transform_textbox.setText(config.get("camera_to_gripper_x_transform", ""))
        self.camera_to_gripper_y_transform_textbox.setText(config.get("camera_to_gripper_y_transform", ""))

    def write_gripper_calibration_config(self) -> None:
        config = self.config
        config["gripper_calibration_start_x"] = self.gripper_calibration_start_x_textbox.text()
        config["gripper_calibration_start_y"] = self.gripper_calibration_start_y_textbox.text()
        config["camera_delta_x"] = self.camera_delta_x_textbox.text()
        config["camera_delta_y"] = self.camera_delta_y_textbox.text()
        config["gripper_delta_x"] = self.gripper_delta_x_textbox.text()
        config["gripper_delta_y"] = self.gripper_delta_y_textbox.text()
        config["gripper_calibration_angle"] = self.gripper_calibration_angle_textbox.text()
        config["gripper_calibration_pickup_z"] = self.gripper_calibration_pickup_z_textbox.text()
        config["gripper_calibration_spin_z"] = self.gripper_calibration_spin_z_textbox.text()
        config["gripper_calibration_photo_directory"] = self.gripper_calibration_photo_directory_textbox.text()
        config["motor_counts_x_per_px_x"] = self.motor_counts_x_per_px_x_textbox.text()
        config["motor_counts_y_per_px_y"] = self.motor_counts_y_per_px_y_textbox.text()
        config["camera_to_gripper_x_transform"] = self.camera_to_gripper_x_transform_textbox.text()
        config["camera_to_gripper_y_transform"] = self.camera_to_gripper_y_transform_textbox.text()
        self.settings.save()
