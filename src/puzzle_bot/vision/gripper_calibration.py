import json
import logging
import posixpath
from collections.abc import Sequence
from typing import Any

import cv2 as cv
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot

from puzzle_bot.utils import (
    get_escape_points,
    get_intersect,
    minimum_distance,
    open_image_undistorted_and_rotated,
    order_rectangle_corners,
)
from puzzle_bot.vision.perspective import correct_perspective, rotate_image

logger = logging.getLogger(__name__)

# From the Miranda motor datasheet:
# https://www.mouser.com/pdfDocs/201215miranda-manual-ovz20008_rev5.pdf
ROTATION_MOTOR_COUNTS_PER_DEGREE = 182


class GripperCalibration(QThread):
    trigger_gripper_calibration = pyqtSignal(str, list, list, float, list, list)
    gripper_calibration_results_ready = pyqtSignal(list, list)

    thread_closing = pyqtSignal()

    def __init__(self, parent: Any) -> None:
        super().__init__(parent)

        self.trigger_gripper_calibration.connect(self.calibrate_gripper)
        self.thread_closing.connect(self.exit_thread)

        self._thread_exiting = False

    def gradient_descent_cost(self, point: Sequence[float], lines: Sequence[Sequence]) -> float:
        cost = 0.0
        for line in lines:
            cost += minimum_distance(point, line) ** 2
        return float(cost)

    def find_chessboard(self, original_img: np.ndarray, debug: bool = False):
        gray_img = cv.cvtColor(original_img, cv.COLOR_BGR2GRAY)

        # https://stackoverflow.com/questions/55169645/square-detection-in-image
        blur_img = cv.medianBlur(gray_img, 45)
        if debug:
            cv.imshow("blur", blur_img)
        sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpen_img = cv.filter2D(blur_img, -1, sharpen_kernel)
        if debug:
            cv.imshow("sharpen", sharpen_img)

        thresh_img = cv.threshold(sharpen_img, 100, 255, cv.THRESH_BINARY)[1]
        if debug:
            cv.imshow("thresh", thresh_img)

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        close_img = cv.morphologyEx(thresh_img, cv.MORPH_CLOSE, kernel, iterations=2)
        if debug:
            cv.imshow("close", close_img)

        contours = cv.findContours(close_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        checkerboard_contour = None
        for c in contours:
            if checkerboard_contour is None or cv.contourArea(c) > cv.contourArea(checkerboard_contour):
                checkerboard_contour = c

        bounding_rect = cv.minAreaRect(checkerboard_contour)
        bounding_box = np.int32(cv.boxPoints(bounding_rect))

        # Crop the bounding box to one quadrant, hiding the marker —
        # the upper-right quadrant of a properly oriented image.
        top_left, top_right, bottom_right, _ = order_rectangle_corners(bounding_box)

        new_top_right = top_right
        new_top_left = np.int32(top_left + (top_right - top_left) / 2)
        new_bottom_right = np.int32(top_right + (bottom_right - top_right) / 2)
        new_bottom_left = np.int32(top_left + (bottom_right - top_left) / 2)

        bounding_box = np.array([new_top_left, new_top_right, new_bottom_right, new_bottom_left])

        mask = np.zeros(original_img.shape[:2], dtype="uint8")
        cv.drawContours(mask, [bounding_box], 0, (255, 255, 255), cv.FILLED)

        masked_img = np.copy(gray_img)
        masked_img[mask == 0] = [255]
        if debug:
            cv.imshow("masked", masked_img)
            cv.waitKey()

        features_x, features_y = (24, 24)
        _, corners = cv.findChessboardCornersSB(
            masked_img,
            (features_x, features_y),
            cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY,
        )

        return corners, masked_img

    @pyqtSlot(str, list, list, float, list, list)
    def calibrate_gripper(
        self,
        gripper_calibration_batch_directory: str,
        camera_matrix: list,
        distortion_coefficients: list,
        perspective_angle: float,
        perspective_starting_quad: list,
        perspective_corrected_quad: list,
    ) -> None:
        debug = False

        perspective_starting_quad_arr = np.float32(perspective_starting_quad)
        perspective_corrected_quad_arr = np.float32(perspective_corrected_quad)

        batch_dir = gripper_calibration_batch_directory
        start_image_path = posixpath.join(batch_dir, "start.jpg")
        camera_delta_image_path = posixpath.join(batch_dir, "camera_delta.jpg")
        gripper_delta_image_path = posixpath.join(batch_dir, "gripper_delta.jpg")
        batch_info_path = posixpath.join(batch_dir, "gripper_calibration_info.json")
        output_image_path = posixpath.join(batch_dir, "gripper_calibration_output.jpg")

        with open(batch_info_path) as jsonfile:
            batch_info = json.load(jsonfile)

        start_x_counts = batch_info["start_x"]
        start_y_counts = batch_info["start_y"]
        camera_delta_x_counts = batch_info["camera_delta_x"]
        camera_delta_y_counts = batch_info["camera_delta_y"]
        gripper_delta_x_counts = batch_info["gripper_delta_x"]
        gripper_delta_y_counts = batch_info["gripper_delta_y"]

        start_img = open_image_undistorted_and_rotated(start_image_path, camera_matrix, distortion_coefficients)
        camera_delta_img = open_image_undistorted_and_rotated(camera_delta_image_path, camera_matrix, distortion_coefficients)
        gripper_delta_img = open_image_undistorted_and_rotated(gripper_delta_image_path, camera_matrix, distortion_coefficients)

        start_img = correct_perspective(start_img, perspective_starting_quad_arr, perspective_corrected_quad_arr, crop=True)
        camera_delta_img = correct_perspective(camera_delta_img, perspective_starting_quad_arr, perspective_corrected_quad_arr, crop=True)
        gripper_delta_img = correct_perspective(gripper_delta_img, perspective_starting_quad_arr, perspective_corrected_quad_arr, crop=True)

        start_img = rotate_image(start_img, perspective_angle, crop=True)
        camera_delta_img = rotate_image(camera_delta_img, perspective_angle, crop=True)
        gripper_delta_img = rotate_image(gripper_delta_img, perspective_angle, crop=True)

        start_chessboard, _ = self.find_chessboard(start_img)
        camera_delta_chessboard, _ = self.find_chessboard(camera_delta_img)
        gripper_delta_chessboard, _ = self.find_chessboard(gripper_delta_img)

        blue_color = (255, 234, 0)
        green_color = (0, 255, 42)
        white_color = (255, 255, 255)
        pink_color = (192, 0, 255)
        circle_thickness = 5
        circle_radius = 10

        ####################################################################
        # MOTOR COUNTS PER PIXEL
        #
        # 1. Photo of the target ("start photo")
        # 2. Camera moved to a different motor position, another photo
        #    ("camera calibration" photo)
        # 3. For corresponding features we compute average pixel deltas and
        #    divide the motor counts moved by those deltas.

        for (point,) in start_chessboard:
            start_img = cv.circle(start_img, np.int32(point).tolist(), circle_radius, green_color, circle_thickness)
            if debug:
                cv.imshow("start", start_img)
                cv.waitKey(1)
        cv.imwrite(posixpath.join(batch_dir, "start-annotated.jpg"), start_img)

        for (point,) in camera_delta_chessboard:
            camera_delta_img = cv.circle(camera_delta_img, np.int32(point).tolist(), circle_radius, pink_color, circle_thickness)
            if debug:
                cv.imshow("camera_delta", camera_delta_img)
                cv.waitKey(1)
        cv.imwrite(posixpath.join(batch_dir, "camera-delta-annotated.jpg"), camera_delta_img)

        for (point,) in gripper_delta_chessboard:
            gripper_delta_img = cv.circle(gripper_delta_img, np.int32(point).tolist(), circle_radius, blue_color, circle_thickness)
            if debug:
                cv.imshow("gripper_delta", gripper_delta_img)
                cv.waitKey(1)
        cv.imwrite(posixpath.join(batch_dir, "gripper-delta-annotated.jpg"), gripper_delta_img)

        delta_x_px_sum = 0.0
        delta_y_px_sum = 0.0
        num_points = 0
        for (start_point,), (camera_delta_point,) in zip(start_chessboard, camera_delta_chessboard, strict=False):
            start_x, start_y = start_point
            camera_delta_x, camera_delta_y = camera_delta_point

            num_points += 1
            delta_x_px_sum += camera_delta_x - start_x
            delta_y_px_sum += camera_delta_y - start_y

        delta_x_px_avg = delta_x_px_sum / num_points
        delta_y_px_avg = delta_y_px_sum / num_points

        logger.debug("delta_x_px_avg: %s", delta_x_px_avg)
        logger.debug("delta_y_px_avg: %s", delta_y_px_avg)
        logger.debug("moved x counts from start to camera delta: %s", camera_delta_x_counts - start_x_counts)
        logger.debug("moved y counts from start to camera delta: %s", camera_delta_y_counts - start_y_counts)

        x_counts_per_px_x = (camera_delta_x_counts - start_x_counts) / delta_x_px_avg
        y_counts_per_px_y = (camera_delta_y_counts - start_y_counts) / delta_y_px_avg
        motor_counts_per_px = [x_counts_per_px_x, y_counts_per_px_y]

        logger.debug("x_counts_per_px_x: %s", x_counts_per_px_x)
        logger.debug("y_counts_per_px_y: %s", y_counts_per_px_y)

        ####################################################################
        # CAMERA → GRIPPER TRANSFORM
        #
        # 1. The gripper was moved to roughly the middle of the target and
        #    the target rotated by a small angle.
        # 2. The camera returned to the start position and took a third photo.
        # 3. For each feature we draw the perpendicular bisector between its
        #    positions in the start and gripper photos; all bisectors should
        #    intersect at the rotation center (= gripper pickup point).
        # 4. Real data never intersects perfectly, so a gradient descent on
        #    the sum-of-squared-distances cost finds the best rotation point.

        perpendicular_lines = []
        for (start_point,), (gripper_delta_point,) in zip(start_chessboard, gripper_delta_chessboard, strict=False):
            start_x, start_y = start_point
            gripper_delta_x, gripper_delta_y = gripper_delta_point

            slope = (gripper_delta_y - start_y) / (gripper_delta_x - start_x)
            midpoint = start_point + (gripper_delta_point - start_point) / 2
            midpoint_x, midpoint_y = midpoint

            perpendicular_slope = -1 / slope
            delta_x = 10
            delta_y = perpendicular_slope * delta_x
            perpendicular_endpoint = [midpoint_x + delta_x, midpoint_y + delta_y]
            perpendicular_lines.append([np.float32(midpoint), np.float32(perpendicular_endpoint)])

            if debug:
                circle_radius = 2
                circle_thickness = 5
                line_thickness = 2
                gripper_delta_img = cv.circle(gripper_delta_img, (int(start_x), int(start_y)), circle_radius, green_color, circle_thickness)
                gripper_delta_img = cv.circle(
                    gripper_delta_img, (int(gripper_delta_x), int(gripper_delta_y)), circle_radius, blue_color, circle_thickness
                )
                gripper_delta_img = cv.line(
                    gripper_delta_img, np.int32(start_point).tolist(), np.int32(gripper_delta_point).tolist(), white_color, line_thickness
                )
                gripper_delta_img = cv.line(
                    gripper_delta_img, np.int32(midpoint).tolist(), np.int32(perpendicular_endpoint).tolist(), pink_color, line_thickness
                )

                height, width = gripper_delta_img.shape[:2]
                p1, p2 = get_escape_points([midpoint, perpendicular_endpoint], width, height)
                gripper_delta_img = cv.line(gripper_delta_img, np.int32(p1).tolist(), np.int32(p2).tolist(), pink_color, 2)

                cv.imshow("gripper delta image", gripper_delta_img)
                cv.waitKey(10)

        if debug:
            cv.waitKey()
            cv.destroyAllWindows()

        # Optimize: find the point nearest to all perpendicular lines.
        # Initial guess: the intersection of the first two lines.
        rotation_point = get_intersect(perpendicular_lines[0], perpendicular_lines[1])

        current_cost = self.gradient_descent_cost(rotation_point, perpendicular_lines)
        print(f"gradient descent cost: {current_cost} for point: {rotation_point} - [iteration 0]")

        iterations = 0
        max_iterations = 100000
        minimum_cost_increment_threshold_per_line = 0.0001
        gradient_step_size = 1  # pixels; adaptive, shrinks as we converge

        while True:
            x, y = rotation_point
            iterations += 1

            cost_x1 = self.gradient_descent_cost([x - gradient_step_size, y], perpendicular_lines)
            cost_x2 = self.gradient_descent_cost([x + gradient_step_size, y], perpendicular_lines)
            next_guess_x = x - gradient_step_size if cost_x1 < cost_x2 else x + gradient_step_size

            cost_y1 = self.gradient_descent_cost([x, y - gradient_step_size], perpendicular_lines)
            cost_y2 = self.gradient_descent_cost([x, y + gradient_step_size], perpendicular_lines)
            next_guess_y = y - gradient_step_size if cost_y1 < cost_y2 else y + gradient_step_size

            rotation_point = (next_guess_x, next_guess_y)

            prior_cost = current_cost
            current_cost = self.gradient_descent_cost(rotation_point, perpendicular_lines)
            print(f"gradient descent cost: {current_cost} for point: {rotation_point} - [iteration {iterations}]")

            if iterations >= max_iterations:
                break

            # Worse than before → shrink the step and try again
            if (prior_cost - current_cost) < 0:
                gradient_step_size = gradient_step_size / 10
                continue

            if (prior_cost - current_cost) < minimum_cost_increment_threshold_per_line * len(perpendicular_lines):
                break

        if debug:
            x, y = np.int32(rotation_point)
            cv.circle(gripper_delta_img, (x, y), 10, (0, 255, 42), 5)
            cv.imshow("gripper delta image", gripper_delta_img)
            cv.imwrite(output_image_path, gripper_delta_img)
            cv.waitKey()

        # Final transform (see NOTES doc: 04/24/2024 - Gripper calibration method)
        rotation_point_x, rotation_point_y = rotation_point

        camera_to_gripper_transform = [
            (rotation_point_x * x_counts_per_px_x) + (gripper_delta_x_counts - start_x_counts),
            (rotation_point_y * y_counts_per_px_y) + (gripper_delta_y_counts - start_y_counts),
        ]

        self.gripper_calibration_results_ready.emit(motor_counts_per_px, camera_to_gripper_transform)

    @pyqtSlot()
    def exit_thread(self) -> None:
        self._thread_exiting = True

    def run(self) -> None:
        while not self._thread_exiting:
            self.msleep(100)
