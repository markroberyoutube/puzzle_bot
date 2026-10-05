import glob
import logging
import math
import posixpath
import tempfile
from pathlib import Path
from typing import Any

import cv2 as cv
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot

from puzzle_bot.utils import estimate_linear_regression_coefficients

logger = logging.getLogger(__name__)


class CameraCalibration(QThread):
    trigger_checkerboard_error_measurements = pyqtSignal(str, int, int, float, float)
    checkerboard_error_measurements_ready = pyqtSignal(float, float, float, float, str)

    trigger_camera_calibration = pyqtSignal(str, int, int, float, float, list, list)
    camera_calibration_results_ready = pyqtSignal(float, float, list, list)

    trigger_undistort_image = pyqtSignal(str, str, list, list)
    undistorted_image_ready = pyqtSignal()

    thread_closing = pyqtSignal()

    def __init__(self, parent: Any) -> None:
        super().__init__(parent)

        self.trigger_checkerboard_error_measurements.connect(self.measure_checkerboard_error)
        self.trigger_camera_calibration.connect(self.calibrate_camera)
        self.trigger_undistort_image.connect(self.undistort_image)
        self.thread_closing.connect(self.exit_thread)

        self._thread_exiting = False

        self.temp_dir = tempfile.mkdtemp()

    @pyqtSlot(str, str, list, list)
    def undistort_image(
        self,
        source_image_path: str,
        destination_image_path: str,
        camera_matrix: list,
        distortion_coefficients: list,
    ) -> None:
        if not source_image_path or not Path(source_image_path).exists():
            logger.error("[CameraCalibration.undistort_image] Source image does not exist at: %s", source_image_path)
            return

        img = cv.imread(source_image_path)

        camera_matrix_arr = np.array(camera_matrix)
        distortion_arr = np.array(distortion_coefficients)

        logger.debug("[CameraCalibration.undistort_image] camera_matrix: %s", camera_matrix_arr)
        logger.debug("[CameraCalibration.undistort_image] distortion_coefficients: %s", distortion_arr)

        dst = cv.undistort(img, camera_matrix_arr, distortion_arr)
        cv.imwrite(destination_image_path, dst)

        self.undistorted_image_ready.emit()

    @pyqtSlot(str, int, int, float, float, list, list)
    def calibrate_camera(
        self,
        photo_directory: str,
        checker_rows: int,
        checker_cols: int,
        checker_width_mm: float,
        checker_height_mm: float,
        camera_matrix: list,
        distortion_coefficients: list,
    ) -> None:
        # Features are the intersections between checkers
        features_x = checker_cols - 1
        features_y = checker_rows - 1

        camera_matrix_arr = np.array(camera_matrix)
        distortion_arr = np.array(distortion_coefficients)

        if not photo_directory or not Path(photo_directory).is_dir():
            logger.error("[CameraCalibration.calibrate_camera] Calibration photo directory does not exist at: %s", photo_directory)
            return

        photos = glob.glob(posixpath.join(photo_directory, "*.jpg"))
        if not photos:
            logger.error("[CameraCalibration.calibrate_camera] No jpg files found in Calibration photo directory at: %s", photo_directory)
            return

        three_d_points = []  # 3D points known from checkerboard dimensions
        two_d_points = []  # 2D points observed in the images

        # (x, y, z) feature locations in mm: (0,0,0), (5,0,0), (10,0,0), ...
        # Presumes a perfectly flat checkerboard with square checkers.
        objectp3d = np.zeros((1, features_x * features_y, 3), np.float32)
        objectp3d[0, :, :2] = np.mgrid[0:features_x, 0:features_y].T.reshape(-1, 2) * checker_height_mm

        # Find features (corners / saddle points) in every calibration photo
        checkerboards_found = 0
        img = None
        for image_path in photos:
            img = cv.imread(image_path)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Sector-based corner detection, http://bmvc2018.org/contents/papers/0508.pdf
            retcode, corners = cv.findChessboardCornersSB(
                gray,
                (features_x, features_y),
                cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY,
            )

            if not retcode or corners is None or len(corners) < features_x * features_y:
                continue

            checkerboards_found += 1
            three_d_points.append(objectp3d)
            two_d_points.append(corners)

        logger.debug("[CameraCalibration.calibrate_camera] Found checkerboards: %d", checkerboards_found)

        img_height, img_width, _ = img.shape
        img_size = (img_width, img_height)
        flags = cv.CALIB_SAME_FOCAL_LENGTH + cv.CALIB_USE_INTRINSIC_GUESS

        max_iterations = 1000
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, max_iterations, 0.00001)

        # Provide initial guesses for f_x, f_y, c_x, c_y when not given.
        # [[fx 0 cx] [0 fy cy] [0 0 1]]
        if camera_matrix_arr[0][0] == 0:
            camera_matrix_arr[0][0] = 10000  # f_x
        if camera_matrix_arr[1][1] == 0:
            camera_matrix_arr[1][1] = 10000  # f_y
        if camera_matrix_arr[0][2] == 0:
            camera_matrix_arr[0][2] = img_width / 2  # c_x
        if camera_matrix_arr[0][2] == 0:
            camera_matrix_arr[1][2] = img_height / 2  # c_y

        rms_reprojection_error, camera_matrix_arr, distortion_arr, r_vecs, t_vecs = cv.calibrateCamera(
            three_d_points, two_d_points, img_size, camera_matrix_arr, distortion_arr, None, None, flags, criteria
        )

        logger.debug("[CameraCalibration.calibrate_camera] RMS RPE: %s", rms_reprojection_error)
        logger.debug("[CameraCalibration.calibrate_camera] Camera matrix: %s", camera_matrix_arr)
        logger.debug("[CameraCalibration.calibrate_camera] Distortion coefficients: %s", distortion_arr)

        # Compute the reprojection error over all points
        total_error = 0.0
        total_points = 0
        largest_error = 0.0
        for i in range(len(three_d_points)):
            reprojected_points, _ = cv.projectPoints(three_d_points[i], r_vecs[i], t_vecs[i], camera_matrix_arr, distortion_arr)
            error = cv.norm(two_d_points[i], reprojected_points, cv.NORM_L2)
            largest_error = max(largest_error, error)
            total_error += error * error
            total_points += three_d_points[i].shape[1]
        calculated_rms_rpe = math.sqrt(total_error / total_points)
        logger.debug("[CameraCalibration.calibrate_camera] RMS Error (re-calculated): %s", calculated_rms_rpe)
        logger.debug("[CameraCalibration.calibrate_camera] Largest Error: %s", largest_error)

        self.camera_calibration_results_ready.emit(calculated_rms_rpe, largest_error, camera_matrix_arr.tolist(), distortion_arr.tolist())

    @pyqtSlot(str, int, int, float, float)
    def measure_checkerboard_error(
        self,
        image_path: str,
        checker_rows: int,
        checker_cols: int,
        checker_width_mm: float,
        checker_height_mm: float,
    ) -> None:
        features_x = checker_cols - 1
        features_y = checker_rows - 1

        radius = 50
        line_color = (255, 255, 255)  # white (BGR)
        circle_color = (243, 10, 213)  # pink (BGR)
        thickness = 10

        if not image_path or not Path(image_path).exists():
            logger.error("[CameraCalibration.measure_checkerboard_error] Image file does not exist at: %s", image_path)
            return

        img = cv.imread(image_path)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        retcode, corners = cv.findChessboardCornersSB(
            gray,
            (features_x, features_y),
            cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY,
        )
        if not retcode or corners is None or len(corners) < features_x * features_y:
            logger.error("[CameraCalibration.measure_checkerboard_error] Could not find all features in image at %s", image_path)
            return
        features = [list(c[0]) for c in corners]

        for x, y in features:
            img = cv.circle(img, (int(x), int(y)), radius, circle_color, thickness)

        # Fit a line to each row and accumulate the error terms
        rows = zip(*(iter(features),) * features_x, strict=False)
        _, img_width, _ = img.shape
        max_error_px = 0.0
        error_squared_total_px = 0.0
        num_error_terms = 0
        for row in rows:
            (a, b) = estimate_linear_regression_coefficients(row)
            logger.debug("[CameraCalibration.measure_checkerboard_error] Linear regression a=%s, b=%s", a, b)
            start_point = (0, int(a))
            end_point = (img_width, int(a + (b * img_width)))
            img = cv.line(img, start_point, end_point, line_color, thickness)
            for point in row:
                x, y = point
                estimated_y = a + b * x
                abs_error_px = abs(y - estimated_y)
                error_squared_total_px += abs_error_px**2
                num_error_terms += 1
                max_error_px = max(max_error_px, abs_error_px)

        average_rms_error_px = math.sqrt(error_squared_total_px / num_error_terms)

        # Smallest feature spacing converts pixels to mm/inches
        rows = zip(*(iter(features),) * features_x, strict=False)
        min_feature_spacing_x = None
        for row in rows:
            prev_x, _ = row[0]
            for point in row[1:]:
                cur_x, _ = point
                spacing_x = abs(cur_x - prev_x)
                if min_feature_spacing_x is None or spacing_x < min_feature_spacing_x:
                    min_feature_spacing_x = spacing_x
                prev_x = cur_x

        px_per_mm = min_feature_spacing_x / checker_width_mm
        px_per_inch = px_per_mm * 25.4
        average_rms_error_inches = average_rms_error_px / px_per_inch
        max_error_inches = max_error_px / px_per_inch

        annotated_image_path = posixpath.join(self.temp_dir, posixpath.split(image_path)[-1])
        cv.imwrite(annotated_image_path, img)

        self.checkerboard_error_measurements_ready.emit(
            average_rms_error_px,
            max_error_px,
            average_rms_error_inches,
            max_error_inches,
            annotated_image_path,
        )

    @pyqtSlot()
    def exit_thread(self) -> None:
        self._thread_exiting = True

    def run(self) -> None:
        while not self._thread_exiting:
            self.msleep(100)
