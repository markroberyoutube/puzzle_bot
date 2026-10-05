import json
import logging
import math
import os
import posixpath
import shutil
import tempfile

import cv2 as cv
from PyQt5 import QtTest
from PyQt5.QtCore import Qt, pyqtSlot

from puzzle_bot.paths import latest_batch_number
from puzzle_bot.utils import open_image_undistorted_and_rotated
from puzzle_bot.vision import correct_perspective, rotate_image

logger = logging.getLogger(__name__)


class SerpentinePhotosTab:
    def setup_serpentine_photos_tab(self) -> None:
        self.screenshot_dir = None
        self.serpentine_move_finished = None
        self.serpentine_paused = None
        self.photo_capture_finished = None

        self.read_serpentine_config()

        self.serpentine_photo_directory_textbox.textChanged.connect(self.write_serpentine_config)
        self.start_x_textbox.textChanged.connect(self.write_serpentine_config)
        self.start_y_textbox.textChanged.connect(self.write_serpentine_config)
        self.start_z_textbox.textChanged.connect(self.write_serpentine_config)
        self.end_x_textbox.textChanged.connect(self.write_serpentine_config)
        self.end_y_textbox.textChanged.connect(self.write_serpentine_config)
        self.end_z_textbox.textChanged.connect(self.write_serpentine_config)
        self.max_step_size_x_textbox.textChanged.connect(self.write_serpentine_config)
        self.max_step_size_y_textbox.textChanged.connect(self.write_serpentine_config)

        self.puzzle_photo_directory_browse_button.clicked.connect(
            lambda: self.browse_for_directory(self.serpentine_photo_directory_textbox)
        )
        self.serpentine_start_button.clicked.connect(self.take_serpentine_photos)
        self.screenshot_button.clicked.connect(self.take_screenshot)

        # Camera object (real Galaxy S24 over ADB, or the fake in demo mode)
        self.camera = self._create_camera()

    def take_serpentine_photos(self) -> None:
        # Metadata dict written to batch.json after all photos are taken
        self.serpentine_photo_batch_info = {}

        self.serpentine_photo_batch_info["start_x"] = self.start_x_textbox.text()
        self.serpentine_photo_batch_info["start_y"] = self.start_y_textbox.text()
        self.serpentine_photo_batch_info["start_z"] = self.start_z_textbox.text()
        self.serpentine_photo_batch_info["end_x"] = self.end_x_textbox.text()
        self.serpentine_photo_batch_info["end_y"] = self.end_y_textbox.text()
        self.serpentine_photo_batch_info["end_z"] = self.end_z_textbox.text()
        self.serpentine_photo_batch_info["max_step_size_x"] = self.max_step_size_x_textbox.text()
        self.serpentine_photo_batch_info["max_step_size_y"] = self.max_step_size_y_textbox.text()

        # Change the "START" button to "PAUSE"
        start_text = self.serpentine_start_button.text()
        pause_text = "PAUSE ⏸️"
        self.serpentine_start_button.setText(pause_text)
        self.serpentine_paused = False

        @pyqtSlot()
        def pause_serpentine_photos() -> None:
            self.serpentine_paused = True
            self.serpentine_start_button.setText(start_text)
            self.serpentine_start_button.clicked.disconnect(pause_serpentine_photos)

            @pyqtSlot()
            def restart_serpentine_photos() -> None:
                self.serpentine_paused = False
                self.serpentine_start_button.setText(pause_text)
                self.serpentine_start_button.clicked.disconnect(restart_serpentine_photos)
                self.serpentine_start_button.clicked.connect(pause_serpentine_photos)

            self.serpentine_start_button.clicked.connect(restart_serpentine_photos)

        self.serpentine_start_button.clicked.connect(pause_serpentine_photos)
        self.serpentine_start_button.clicked.disconnect(self.take_serpentine_photos)

        # Make sure the photo directory exists
        photo_directory = self.serpentine_photo_directory_textbox.text()
        if not photo_directory or not posixpath.isdir(photo_directory):
            logger.error("[SerpentinePhotosTab.take_serpentine_photos] Photo directory does not exist at %s", photo_directory)
            return

        # Auto-generate the next numbered batch directory
        current_batch_number = latest_batch_number(photo_directory) + 1
        batch_dir = posixpath.join(photo_directory, str(current_batch_number))
        os.makedirs(batch_dir)
        self.batch_number_textbox.setText(str(current_batch_number))

        solver_directory = self.solver_directory_textbox.text()
        if not solver_directory or not posixpath.isdir(solver_directory):
            logger.error("[SerpentinePhotosTab.take_serpentine_photos] Solver directory does not exist at %s", solver_directory)
            return
        self.solver_batch_photos_dir = posixpath.join(solver_directory, str(current_batch_number), "0_photos")
        os.makedirs(self.solver_batch_photos_dir)

        # Step sizes: satisfy the maximum step while keeping steps equal integers
        start_x = int(self.start_x_textbox.text())
        start_y = int(self.start_y_textbox.text())
        start_z = int(self.start_z_textbox.text())
        end_x = int(self.end_x_textbox.text())
        end_y = int(self.end_y_textbox.text())
        max_x_step_size = int(self.max_step_size_x_textbox.text())
        max_y_step_size = int(self.max_step_size_y_textbox.text())
        num_x_passes = math.ceil((end_x - start_x) / max_x_step_size)
        num_y_passes = math.ceil((end_y - start_y) / max_y_step_size)
        total_photos = num_x_passes * num_y_passes
        x_pass_distance = math.floor((end_x - start_x) / num_x_passes)
        y_pass_distance = math.floor((end_y - start_y) / num_y_passes)

        num_photos_taken = 1

        z = start_z
        moving_right = True
        for y in range(start_y, end_y + 1, y_pass_distance):
            x_steps = list(range(start_x, end_x + 1, x_pass_distance))
            if not moving_right:
                x_steps.reverse()
            for x in x_steps:
                self.statusBar().showMessage(f"Taking picture {num_photos_taken} of {total_photos} at {x},{y},{z}")

                self.serpentine_move_finished = False

                @pyqtSlot(str)
                def on_response(line: str) -> None:
                    self.clearcore.line_received.disconnect(on_response)
                    self.serpentine_move_finished = True

                self.clearcore.line_received.connect(on_response)

                # Point the eyes in the direction of travel
                eye_angle = self.robot.right_eye_angle if moving_right else self.robot.left_eye_angle
                self.send_gripper_command(f"e {eye_angle}", blocking=True)

                # Move to the spot and block until the move finishes
                self.send_clearcore_command(f"m {x},{y},{z}", blocking=True)

                self.photo_capture_finished = False

                @pyqtSlot(str)
                def on_capture(image_path: str) -> None:
                    self.camera.photo_captured.disconnect(on_capture)
                    self.last_serpentine_image_path = image_path
                    self.photo_capture_finished = True

                self.camera.photo_captured.connect(on_capture)

                self.take_serpentine_photo((x, y, z), batch_dir)

                while not self.photo_capture_finished:
                    QtTest.QTest.qWait(100)

                # Rotate and undistort the image
                camera_matrix, distortion_coefficients = self.get_camera_intrinsics()
                img = open_image_undistorted_and_rotated(self.last_serpentine_image_path, camera_matrix, distortion_coefficients)

                perspective_angle, perspective_starting_quad, perspective_corrected_quad = self.get_perspective_corrections()

                img = correct_perspective(img, perspective_starting_quad, perspective_corrected_quad, crop=True)
                img = rotate_image(img, perspective_angle, crop=True)

                solver_filename = posixpath.basename(self.last_serpentine_image_path)
                solver_img_path = posixpath.join(self.solver_batch_photos_dir, solver_filename)
                cv.imwrite(solver_img_path, img)

                num_photos_taken += 1

            moving_right = not moving_right

        # Point the eyes down again
        self.send_gripper_command(f"e {self.robot.neutral_eye_angle}", blocking=True)

        # Write batch.json and copy it into the solver batch dir
        batch_info_file = posixpath.join(batch_dir, "batch.json")
        with open(batch_info_file, "w") as jsonfile:
            json.dump(self.serpentine_photo_batch_info, jsonfile)

        destination_file = posixpath.join(self.solver_batch_photos_dir, "batch.json")
        shutil.copyfile(batch_info_file, destination_file)

        self.serpentine_start_button.clicked.connect(self.take_serpentine_photos)
        self.serpentine_paused = None
        self.serpentine_start_button.setText(start_text)

        self.statusBar().showMessage("Finished taking and processing serpentine photos!")

    def take_screenshot(self) -> None:
        self.screenshot_button.setEnabled(False)

        if self.screenshot_dir is None:
            self.screenshot_dir = tempfile.mkdtemp()

        @pyqtSlot(str)
        def on_screenshot(image_path: str) -> None:
            self.camera.screenshot_captured.disconnect(on_screenshot)
            self.screenshot_button.setEnabled(True)
            self.display_image(image_path, self.screenshot_label)

        self.camera.screenshot_captured.connect(on_screenshot)
        self.camera.trigger_screenshot.emit(self.screenshot_dir)

    def take_serpentine_photo(self, position, batch_dir: str) -> None:
        if not batch_dir:
            logger.error("[SerpentinePhotosTab.take_serpentine_photo] Invalid batch directory: %s", batch_dir)
            return
        try:
            if not posixpath.exists(batch_dir):
                os.makedirs(batch_dir)
        except Exception as e:
            logger.error("[SerpentinePhotosTab.take_serpentine_photo] Could not create batch directory at %s: %s", batch_dir, e)
            return

        @pyqtSlot(str)
        def on_capture(image_path: str) -> None:
            self.camera.photo_captured.disconnect(on_capture)
            x, y, _ = position
            caption = f"{x}, {y}"
            self.show_serpentine_photo(image_path, caption)
            self.save_serpentine_photo_info(image_path, position, batch_dir)

        self.camera.photo_captured.connect(on_capture)
        self.camera.trigger_photo.emit(batch_dir)

    def save_serpentine_photo_info(self, image_path: str, position, batch_dir: str) -> None:
        if self.serpentine_photo_batch_info.get("photos", None) is None:
            self.serpentine_photo_batch_info["photos"] = []
        file_name = posixpath.split(image_path)[-1]
        self.serpentine_photo_batch_info["photos"].append(dict(file_name=file_name, position=position))

    def show_serpentine_photo(self, image_path: str, caption: str) -> None:
        pixmap = self.pixmap_from_image(image_path)
        if pixmap is None:
            return
        pixmap = pixmap.scaledToWidth(self.current_photo_caption_label.width(), Qt.SmoothTransformation)

        if self.current_photo_label.pixmap() is not None:
            self.prior_photo_label.setPixmap(self.current_photo_label.pixmap())
            self.prior_photo_label.setAlignment(Qt.AlignCenter)
            self.prior_photo_caption_label.setText(self.current_photo_caption_label.text())

        self.current_photo_label.setPixmap(pixmap)
        self.current_photo_label.setAlignment(Qt.AlignCenter)
        image_filename = posixpath.split(image_path)[-1]
        self.current_photo_caption_label.setText(f"{image_filename} ({caption})")

    def read_serpentine_config(self) -> None:
        config = self.config
        self.serpentine_photo_directory_textbox.setText(config.get("serpentine_photo_directory", ""))
        self.start_x_textbox.setText(config.get("start_x", ""))
        self.start_y_textbox.setText(config.get("start_y", ""))
        self.end_x_textbox.setText(config.get("end_x", ""))
        self.end_y_textbox.setText(config.get("end_y", ""))
        self.max_step_size_x_textbox.setText(config.get("max_step_size_x", ""))
        self.max_step_size_y_textbox.setText(config.get("max_step_size_y", ""))

    def write_serpentine_config(self) -> None:
        config = self.config
        config["serpentine_photo_directory"] = self.serpentine_photo_directory_textbox.text()
        config["start_x"] = self.start_x_textbox.text()
        config["start_y"] = self.start_y_textbox.text()
        config["start_z"] = self.start_z_textbox.text()
        config["end_x"] = self.end_x_textbox.text()
        config["end_y"] = self.end_y_textbox.text()
        config["end_z"] = self.end_z_textbox.text()
        config["max_step_size_x"] = self.max_step_size_x_textbox.text()
        config["max_step_size_y"] = self.max_step_size_y_textbox.text()
        self.settings.save()
