import math
import sys

import cv2 as cv
import numpy as np

from puzzle_bot.utils import (
    estimate_linear_regression_coefficients,
    get_intersect,
    get_point_on_line,
    sort_corners,
)


def rotate_image(img: np.ndarray, angle_degrees: float, crop: bool = True) -> np.ndarray:
    image_center = tuple(np.array(img.shape[1::-1]) / 2)
    rot_mat = cv.getRotationMatrix2D(image_center, angle_degrees, 1.0)
    rotated_img = cv.warpAffine(img, rot_mat, img.shape[1::-1], flags=cv.INTER_NEAREST)

    counter_clockwise_rotation = angle_degrees > 0

    if crop:
        # https://docs.opencv.org/4.x/d4/d61/tutorial_warp_affine.html
        height, width, _ = img.shape
        ul = np.transpose(np.array([0, 0, 1]))
        ur = np.transpose(np.array([width, 0, 1]))
        lr = np.transpose(np.array([width, height, 1]))
        ll = np.transpose(np.array([0, height, 1]))

        new_ul = np.dot(rot_mat, ul)
        new_ur = np.dot(rot_mat, ur)
        new_lr = np.dot(rot_mat, lr)
        new_ll = np.dot(rot_mat, ll)

        crop_left_x = 0
        crop_top_y = 0
        crop_right_x = 0
        crop_bottom_y = 0

        if counter_clockwise_rotation:
            crop_left_x = new_ll[0]
            crop_right_x = new_ur[0]

            rotated_upper_edge = [new_ul, new_ur]
            new_ll_x_line = [(new_ll[0], 0), (new_ll[0], height)]
            intersection_point = get_intersect(rotated_upper_edge, new_ll_x_line)
            crop_top_y = intersection_point[1]

            rotated_lower_edge = [new_ll, new_lr]
            new_ur_x_line = [(new_ur[0], 0), (new_ur[0], height)]
            intersection_point = get_intersect(rotated_lower_edge, new_ur_x_line)
            crop_bottom_y = intersection_point[1]
        else:
            # For clockwise rotation the opposite corners define the crop
            crop_left_x = new_ul[0]
            crop_right_x = new_lr[0]

            rotated_upper_edge = [new_ul, new_ur]
            new_lr_x_line = [(new_lr[0], 0), (new_lr[0], height)]
            intersection_point = get_intersect(rotated_upper_edge, new_lr_x_line)
            crop_top_y = intersection_point[1]

            rotated_lower_edge = [new_ll, new_lr]
            new_ul_x_line = [(new_ul[0], 0), (new_ul[0], height)]
            intersection_point = get_intersect(rotated_lower_edge, new_ul_x_line)
            crop_bottom_y = intersection_point[1]

        crop_left_x = math.ceil(crop_left_x)
        crop_right_x = math.floor(crop_right_x)
        crop_top_y = math.ceil(crop_top_y)
        crop_bottom_y = math.floor(crop_bottom_y)

        rotated_img = rotated_img[crop_top_y:crop_bottom_y, crop_left_x:crop_right_x]

    return rotated_img


def calibrate_rotation(img1: np.ndarray, img2: np.ndarray) -> float:
    num_columns = 3  # corners in a horizontal row (findCorners finds them all)
    num_rows = 14  # corners in a vertical column (must all be visible)

    ret1, corners1 = cv.findChessboardCornersSB(
        img1,
        (num_rows, num_columns),
        flags=cv.CALIB_CB_LARGER + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY,
    )
    if not ret1:
        print("ERROR: Could not find corners in first image")
        sys.exit(-1)

    if len(corners1) % num_rows != 0:
        print("ERROR: Some columns are partially cut off")
        sys.exit(-1)

    ret2, corners2 = cv.findChessboardCornersSB(
        img2,
        (num_rows, num_columns),
        flags=cv.CALIB_CB_LARGER + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY,
    )
    if not ret2:
        print("ERROR: Could not find corners in second image")
        sys.exit(-1)

    if len(corners2) % num_rows != 0:
        print("ERROR: Some columns are partially cut off")
        sys.exit(-1)

    num_cols1 = len(corners1) // num_rows
    num_cols2 = len(corners2) // num_rows

    corners1 = corners1.squeeze().reshape(num_rows, num_cols1, 2)
    corners2 = corners2.squeeze().reshape(num_rows, num_cols2, 2)

    # Average the angle of the segments joining each corner pair
    angle_count = 0
    angle_sum = 0.0
    for row_idx, row in enumerate(corners1):
        for col_idx, point in enumerate(row):
            x1, y1 = point
            x2, y2 = corners2[row_idx][col_idx]

            delta_x = x1 - x2
            delta_y = y1 - y2
            angle_radians = math.atan(delta_y / delta_x)
            angle_degrees = math.degrees(angle_radians)
            angle_sum += angle_degrees
            print(angle_degrees)
            angle_count += 1

    average_angle_degrees = angle_sum / angle_count
    print(f"Average Angle Degrees: {average_angle_degrees}")

    return average_angle_degrees


def correct_perspective(
    img: np.ndarray,
    starting_quad: np.ndarray,
    corrected_quad: np.ndarray,
    crop: bool = True,
) -> np.ndarray:
    height, width, _ = img.shape
    correction_matrix = cv.getPerspectiveTransform(starting_quad, corrected_quad)
    img = cv.warpPerspective(
        img, correction_matrix, (width, height), flags=cv.INTER_NEAREST, borderMode=cv.BORDER_CONSTANT, borderValue=(0, 255, 0)
    )
    if crop:
        # Crop away warped-out regions: for every edge, crop to whichever
        # warped corner is closest to the image center.
        ul = np.transpose(np.array([0, 0, 1]))
        ur = np.transpose(np.array([width, 0, 1]))
        lr = np.transpose(np.array([width, height, 1]))
        ll = np.transpose(np.array([0, height, 1]))

        new_ul = np.dot(correction_matrix, ul)
        new_ur = np.dot(correction_matrix, ur)
        new_lr = np.dot(correction_matrix, lr)
        new_ll = np.dot(correction_matrix, ll)

        # Scale by z (see https://stackoverflow.com/questions/45010881/)
        new_ul /= new_ul[2]
        new_ur /= new_ur[2]
        new_lr /= new_lr[2]
        new_ll /= new_ll[2]

        crop_right_to_x = np.int32(np.floor(np.min([new_ur[0], new_lr[0]])))
        crop_left_to_x = np.int32(np.ceil(np.max([new_ul[0], new_ll[0]])))
        crop_top_to_y = np.int32(np.ceil(np.max([new_ul[1], new_ur[1]])))
        crop_bottom_to_y = np.int32(np.floor(np.min([new_ll[1], new_lr[1]])))

        img = img[crop_top_to_y:crop_bottom_to_y, crop_left_to_x:crop_right_to_x]

    return img


def calibrate_perspective(img: np.ndarray, checker_columns: int, checker_rows: int) -> tuple[np.ndarray, np.ndarray]:
    features_x = checker_columns - 1
    features_y = checker_rows - 1

    ret, corners = cv.findChessboardCornersSB(
        img,
        (features_x, features_y),
        flags=cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY,
    )
    if not ret:
        print("ERROR: Could not find all corners")
        sys.exit(-1)

    corners = corners.squeeze().reshape(features_y, features_x, 2)

    # Normalize corner order so the lower-left corner comes first
    corners = sort_corners(corners)

    # Fit lines to the four board edges
    top_edge = corners[-1]
    bottom_edge = corners[0]
    left_edge = np.array([row[0] for row in corners])
    right_edge = np.array([row[-1] for row in corners])

    top_intercept, top_slope = estimate_linear_regression_coefficients(top_edge)
    bottom_intercept, bottom_slope = estimate_linear_regression_coefficients(bottom_edge)
    left_intercept, left_slope = estimate_linear_regression_coefficients(left_edge)
    right_intercept, right_slope = estimate_linear_regression_coefficients(right_edge)

    height, width, _ = img.shape
    top_p1 = (0, top_intercept)
    bottom_p1 = (0, bottom_intercept)
    left_p1 = (0, left_intercept)
    right_p1 = (0, right_intercept)
    top_p2 = (width, top_intercept + top_slope * width)
    bottom_p2 = (width, bottom_intercept + bottom_slope * width)
    left_p2 = (width, left_intercept + left_slope * width)
    right_p2 = (width, right_intercept + right_slope * width)

    # Each quad corner is the intersection of two edge lines
    tl = get_intersect((top_p1, top_p2), (left_p1, left_p2))
    tr = get_intersect((top_p1, top_p2), (right_p1, right_p2))
    br = get_intersect((bottom_p1, bottom_p2), (right_p1, right_p2))
    bl = get_intersect((bottom_p1, bottom_p2), (left_p1, left_p2))

    # The corrected quad keeps the top edge; side edges become perpendicular
    # to it and proportionally long per the checkerboard aspect ratio.
    top_edge_length = np.linalg.norm(np.array(tr) - np.array(tl))
    checker_ratio = (features_x - 1) / (features_y - 1)  # width/height ratio
    side_edge_length = top_edge_length / checker_ratio

    top_edge_slope = (tr[1] - tl[1]) / (tr[0] - tl[0])
    side_edge_slope = -1 / top_edge_slope

    left_edge_arbitrary_point_y = tl[1] + height
    left_edge_arbitrary_point_x = ((left_edge_arbitrary_point_y - tl[1]) / side_edge_slope) + tl[0]
    new_bl = get_point_on_line([tl, (left_edge_arbitrary_point_x, left_edge_arbitrary_point_y)], side_edge_length)

    right_edge_arbitrary_point_y = tr[1] + height
    right_edge_arbitrary_point_x = ((right_edge_arbitrary_point_y - tr[1]) / side_edge_slope) + tr[0]
    new_br = get_point_on_line([tr, (right_edge_arbitrary_point_x, right_edge_arbitrary_point_y)], side_edge_length)

    starting_quad = np.float32([tl, tr, br, bl])
    corrected_quad = np.float32([tl, tr, new_br, new_bl])

    return starting_quad, corrected_quad
