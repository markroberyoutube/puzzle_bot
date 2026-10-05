import logging
import posixpath

from PyQt5.QtCore import pyqtSlot

from puzzle_bot.vision import CameraCalibration

logger = logging.getLogger(__name__)


class CameraCalibrationTab:
    def setup_camera_calibration_tab(self) -> None:
        self.read_camera_calibration_config()

        # Auto-save every textbox change into the config file
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

        self.camera_calibration_button.clicked.connect(self.perform_camera_calibration)
        self.pre_camera_calibration_measure_error_button.clicked.connect(self.measure_pre_camera_calibration_error)
        self.post_camera_calibration_measure_error_button.clicked.connect(self.measure_post_camera_calibration_error)
        self.pre_camera_calibration_photo_browse_button.clicked.connect(
            lambda: self.browse_for_image(self.pre_camera_calibration_photo_textbox, self.pre_camera_calibration_example_photo_label)
        )
        self.post_camera_calibration_photo_browse_button.clicked.connect(
            lambda: self.browse_for_image(self.post_camera_calibration_photo_textbox, self.post_camera_calibration_example_photo_label)
        )
        self.camera_calibration_photo_directory_browse_button.clicked.connect(
            lambda: self.browse_for_directory(self.camera_calibration_photo_directory_textbox)
        )

        self.camera_calibration = CameraCalibration(parent=self)
        self.camera_calibration.start()

    def measure_checkerboard_error(self, image_path, rms_error_textbox, max_error_textbox, annotated_image_label_area) -> None:
        if not image_path or not posixpath.exists(image_path):
            logger.error("[CameraCalibrationTab.measure_checkerboard_error] File does not exist at %s", image_path)
            return

        checker_rows = self.camera_calibration_checker_rows_spinbox.value()
        checker_columns = self.camera_calibration_checker_columns_spinbox.value()
        checker_width = float(self.camera_calibration_checker_width_textbox.text())
        checker_height = float(self.camera_calibration_checker_height_textbox.text())

        @pyqtSlot(float, float, float, float, str)
        def on_results(average_rms_error_px, max_error_px, average_rms_error_inches, max_error_inches, annotated_image_path):
            self.camera_calibration.checkerboard_error_measurements_ready.disconnect(on_results)
            rms_error_textbox.setText(f"{round(average_rms_error_px, 2)} px ({round(average_rms_error_inches, 5)} inches)")
            max_error_textbox.setText(f"{round(max_error_px, 2)} px ({round(max_error_inches, 5)} inches)")
            self.display_image(annotated_image_path, annotated_image_label_area)

        self.camera_calibration.checkerboard_error_measurements_ready.connect(on_results)
        self.camera_calibration.trigger_checkerboard_error_measurements.emit(
            image_path, checker_rows, checker_columns, checker_width, checker_height
        )

    @pyqtSlot()
    def measure_pre_camera_calibration_error(self) -> None:
        image_path = self.pre_camera_calibration_photo_textbox.text()
        self.measure_checkerboard_error(
            image_path,
            self.pre_camera_calibration_rms_error_textbox,
            self.pre_camera_calibration_max_error_textbox,
            self.pre_camera_calibration_error_visualization_label,
        )

    def get_camera_intrinsics(self):
        camera_matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 1]]
        camera_matrix[0][0] = float(self.camera_fx_textbox.text() or 0)
        camera_matrix[1][1] = float(self.camera_fy_textbox.text() or 0)
        camera_matrix[0][2] = float(self.camera_cx_textbox.text() or 0)
        camera_matrix[1][2] = float(self.camera_cy_textbox.text() or 0)
        # Distortion coefficients: [[k1, k2, p1, p2, k3]]
        distortion_coefficients = [[0, 0, 0, 0, 0]]
        distortion_coefficients[0][0] = float(self.camera_k1_textbox.text() or 0)
        distortion_coefficients[0][1] = float(self.camera_k2_textbox.text() or 0)
        distortion_coefficients[0][2] = float(self.camera_p1_textbox.text() or 0)
        distortion_coefficients[0][3] = float(self.camera_p2_textbox.text() or 0)
        distortion_coefficients[0][4] = float(self.camera_k3_textbox.text() or 0)
        return (camera_matrix, distortion_coefficients)

    @pyqtSlot()
    def measure_post_camera_calibration_error(self) -> None:
        source_image_path = self.post_camera_calibration_photo_textbox.text()
        path_parts = source_image_path.split(".")
        destination_image_path = ".".join(path_parts[:-1]) + "-undistorted." + path_parts[-1]

        camera_matrix, distortion_coefficients = self.get_camera_intrinsics()
        logger.debug(
            "[CameraCalibrationTab.measure_post_camera_calibration_error] matrix: %s\ndistortion: %s",
            camera_matrix,
            distortion_coefficients,
        )

        @pyqtSlot()
        def on_results() -> None:
            self.camera_calibration.undistorted_image_ready.disconnect(on_results)
            self.measure_checkerboard_error(
                destination_image_path,
                self.post_camera_calibration_rms_error_textbox,
                self.post_camera_calibration_max_error_textbox,
                self.post_camera_calibration_error_visualization_label,
            )

        self.camera_calibration.undistorted_image_ready.connect(on_results)
        self.camera_calibration.trigger_undistort_image.emit(
            source_image_path, destination_image_path, camera_matrix, distortion_coefficients
        )

    @pyqtSlot()
    def perform_camera_calibration(self) -> None:
        @pyqtSlot(float, float, list, list)
        def on_results(rms_rpe, max_error, camera_matrix, distortion_coefficients):
            self.camera_calibration.camera_calibration_results_ready.disconnect(on_results)
            # Camera matrix: [[fx 0 cx] [0 fy cy] [0 0 1]]
            self.camera_fx_textbox.setText(str(round(camera_matrix[0][0], 5)))
            self.camera_fy_textbox.setText(str(round(camera_matrix[1][1], 5)))
            self.camera_cx_textbox.setText(str(round(camera_matrix[0][2], 5)))
            self.camera_cy_textbox.setText(str(round(camera_matrix[1][2], 5)))
            # Distortion coefficients: [[k1, k2, p1, p2, k3]]
            self.camera_k1_textbox.setText(str(round(distortion_coefficients[0][0], 5)))
            self.camera_k2_textbox.setText(str(round(distortion_coefficients[0][1], 5)))
            self.camera_p1_textbox.setText(str(round(distortion_coefficients[0][2], 5)))
            self.camera_p2_textbox.setText(str(round(distortion_coefficients[0][3], 5)))
            self.camera_k3_textbox.setText(str(round(distortion_coefficients[0][4], 5)))
            self.camera_calibration_rms_error_textbox.setText(str(round(rms_rpe, 5)))
            self.camera_calibration_max_error_textbox.setText(str(round(max_error, 5)))

        self.camera_calibration.camera_calibration_results_ready.connect(on_results)

        checker_rows = self.camera_calibration_checker_rows_spinbox.value()
        checker_columns = self.camera_calibration_checker_columns_spinbox.value()
        checker_width = float(self.camera_calibration_checker_width_textbox.text())
        checker_height = float(self.camera_calibration_checker_height_textbox.text())

        camera_matrix, distortion_coefficients = self.get_camera_intrinsics()

        photo_directory = self.camera_calibration_photo_directory_textbox.text()
        self.camera_calibration.trigger_camera_calibration.emit(
            photo_directory, checker_rows, checker_columns, checker_width, checker_height, camera_matrix, distortion_coefficients
        )

    def read_camera_calibration_config(self) -> None:
        config = self.config
        self.camera_calibration_checker_rows_spinbox.setValue(config.get("camera_calibration_checker_rows", 0))
        self.camera_calibration_checker_columns_spinbox.setValue(config.get("camera_calibration_checker_columns", 0))
        self.camera_calibration_checker_width_textbox.setText(config.get("camera_calibration_checker_width", "0.0"))
        self.camera_calibration_checker_height_textbox.setText(config.get("camera_calibration_checker_height", "0.0"))
        self.pre_camera_calibration_photo_textbox.setText(config.get("pre_camera_calibration_image", ""))
        self.camera_calibration_photo_directory_textbox.setText(config.get("camera_calibration_photo_directory", ""))
        self.post_camera_calibration_photo_textbox.setText(config.get("post_camera_calibration_image", ""))
        self.camera_fx_textbox.setText(config.get("camera_fx", ""))
        self.camera_fy_textbox.setText(config.get("camera_fy", ""))
        self.camera_cx_textbox.setText(config.get("camera_cx", ""))
        self.camera_cy_textbox.setText(config.get("camera_cy", ""))
        self.camera_k1_textbox.setText(config.get("camera_k1", ""))
        self.camera_k2_textbox.setText(config.get("camera_k2", ""))
        self.camera_k3_textbox.setText(config.get("camera_k3", ""))
        self.camera_p1_textbox.setText(config.get("camera_p1", ""))
        self.camera_p2_textbox.setText(config.get("camera_p2", ""))

    def write_camera_calibration_config(self) -> None:
        config = self.config
        config["camera_calibration_checker_rows"] = self.camera_calibration_checker_rows_spinbox.value()
        config["camera_calibration_checker_columns"] = self.camera_calibration_checker_columns_spinbox.value()
        config["camera_calibration_checker_width"] = self.camera_calibration_checker_width_textbox.text()
        config["camera_calibration_checker_height"] = self.camera_calibration_checker_height_textbox.text()
        config["pre_camera_calibration_image"] = self.pre_camera_calibration_photo_textbox.text()
        config["camera_calibration_photo_directory"] = self.camera_calibration_photo_directory_textbox.text()
        config["post_camera_calibration_image"] = self.post_camera_calibration_photo_textbox.text()
        config["camera_fx"] = self.camera_fx_textbox.text()
        config["camera_fy"] = self.camera_fy_textbox.text()
        config["camera_cx"] = self.camera_cx_textbox.text()
        config["camera_cy"] = self.camera_cy_textbox.text()
        config["camera_k1"] = self.camera_k1_textbox.text()
        config["camera_k2"] = self.camera_k2_textbox.text()
        config["camera_k3"] = self.camera_k3_textbox.text()
        config["camera_p1"] = self.camera_p1_textbox.text()
        config["camera_p2"] = self.camera_p2_textbox.text()
        self.settings.save()
