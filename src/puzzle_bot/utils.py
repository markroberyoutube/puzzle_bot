import logging
from collections.abc import Iterable, Sequence

import cv2 as cv
import exif
import numpy as np
from scipy import stats

logger = logging.getLogger(__name__)

# Number of border pixels cropped off undistorted photos to remove the
# black frame produced by cv.undistort.
LEFT_CROP_PIXELS = 13
BOTTOM_CROP_PIXELS = 11
RIGHT_CROP_PIXELS = 19
TOP_CROP_PIXELS = 13

Point = Sequence[float]


def get_point_on_line(line: tuple[Point, Point], distance: float) -> tuple[float, float]:
    initial_point, terminal_point = line
    v = np.array(initial_point, dtype=float)
    u = np.array(terminal_point, dtype=float)
    n = v - u
    n /= np.linalg.norm(n, 2)
    point = v - distance * n
    return tuple(point)  # type: ignore[return-value]


def sort_corners(corners: np.ndarray) -> np.ndarray:
    # See if we need to flip the column order
    row_0_start_x, _ = corners[0][0]
    row_0_end_x, _ = corners[0][-1]
    if row_0_start_x > row_0_end_x:
        corners = np.fliplr(corners)

    # See if we need to flip the row order
    _, col_0_start_y = corners[0][0]
    _, col_0_end_y = corners[-1][0]
    if col_0_start_y < col_0_end_y:
        corners = np.flipud(corners)

    return corners


def manhattan_distance(point1: Point, point2: Point) -> float:
    x1, y1 = point1
    x2, y2 = point2
    return abs(x2 - x1) + abs(y2 - y1)


def anchor_order(pieces: Iterable[Point]) -> list[Point]:
    pieces = list(pieces)
    if len(pieces) == 0:
        return []

    max_x, max_y = max(pieces)
    pieces_in_anchor_order: list[Point] = []
    total_pieces_to_place = (max_x + 1) * (max_y + 1)

    distance = 0
    origin = [0, 0]
    pieces_in_anchor_order.append(origin)

    while len(pieces_in_anchor_order) < total_pieces_to_place:
        distance += 1
        pieces_at_distance = [p for p in pieces if manhattan_distance(origin, p) == distance]
        pieces_at_distance.sort(key=lambda p: p[0])
        pieces_in_anchor_order.extend(pieces_at_distance)

    return pieces_in_anchor_order


def open_image_undistorted_and_rotated(
    image_path: str,
    camera_matrix: Sequence[Sequence[float]],
    distortion_coefficients: Sequence[Sequence[float]],
) -> np.ndarray:
    img = cv.imread(image_path)

    # Rotate before undistorting: the camera matrix and distortion parameters
    # were computed on images in normal horizontal orientation.
    try:
        with open(image_path, "rb") as image_file:
            exif_img = exif.Image(image_file)
            orientation = exif_img.orientation
            if orientation == 1:  # Normal orientation, no rotation
                pass
            elif orientation == 3:  # Rotate 180
                img = cv.rotate(img, cv.ROTATE_180)
            elif orientation == 6:  # Rotated 90 CCW, counteract that
                img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
            elif orientation == 8:  # Rotated 270 CCW, counteract that
                img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
            else:
                logger.error("[open_image_undistorted_and_rotated] Unsupported jpg orientation: %s", orientation)
    except Exception as e:
        logger.error("[open_image_undistorted_and_rotated] Error reading exif: %s", e)

    # The phone may output a portrait-format photo while flagging the
    # orientation as normal; rotate 90 CW when height exceeds width.
    height, width, _ = img.shape
    if height > width:
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)

    camera_matrix_arr = np.float32(camera_matrix)
    distortion_arr = np.float32(distortion_coefficients)

    # Undistort (a black border appears and is cropped below)
    img = cv.undistort(img, camera_matrix_arr, distortion_arr)

    height, width, _ = img.shape
    img = img[TOP_CROP_PIXELS : height - BOTTOM_CROP_PIXELS, LEFT_CROP_PIXELS : width - RIGHT_CROP_PIXELS]

    return img


def parse_int(data: str) -> int | None:
    result_chars: list[str] = []
    for c in data:
        if c.isdigit() or c == "-":
            result_chars.append(c)
        elif result_chars:
            break
    return int("".join(result_chars)) if result_chars else None


def estimate_linear_regression_coefficients(points: Iterable[Point]) -> tuple[float, float]:
    x_data = [p[0] for p in points]
    y_data = [p[1] for p in points]
    slope, intercept, *_ = stats.linregress(x_data, y_data)
    return (intercept, slope)


def minimum_distance(point: Point, line: tuple[Point, Point]) -> np.floating:
    point_arr = np.float32(point)
    line_arr = np.float32(line)
    p1, p2 = line_arr
    return np.cross(p2 - p1, point_arr - p1) / np.linalg.norm(p2 - p1)


def get_escape_points(line: tuple[Point, Point], width: int, height: int) -> list[list[float]]:
    escape_points: list[list[float]] = []
    p1, p2 = line

    edges = [
        ([p1, p2], [(0, 0), (width, 0)], "top"),
        ([p1, p2], [(width, 0), (width, height)], "right"),
        ([p1, p2], [(width, height), (0, height)], "bottom"),
        ([p1, p2], [(0, height), (0, 0)], "left"),
    ]
    for _, edge, side in edges:
        x, y = get_intersect((p1, p2), edge)
        if side in ("top", "bottom"):
            if 0 < x < width:
                escape_points.append([x, y])
        elif 0 < y < height:
            escape_points.append([x, y])
    return escape_points


def get_intersect(line_a: tuple[Point, Point], line_b: tuple[Point, Point]) -> tuple[float, float]:
    a1, a2 = line_a
    b1, b2 = line_b
    s = np.vstack([a1, a2, b1, b2])  # s for stacked
    h = np.hstack((s, np.ones((4, 1))))  # h for homogeneous
    l1 = np.cross(h[0], h[1])  # first line
    l2 = np.cross(h[2], h[3])  # second line
    x, y, z = np.cross(l1, l2)  # point of intersection
    if z == 0:  # lines are parallel
        return (float("inf"), float("inf"))
    return (x / z, y / z)


def order_rectangle_corners(points: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype="int32")

    # The top-left point has the smallest sum, the bottom-right the largest.
    s = points.sum(axis=1)
    rect[0] = points[np.argmin(s)]
    rect[2] = points[np.argmax(s)]

    # The top-right point has the smallest difference, bottom-left the largest.
    diff = np.diff(points, axis=1)
    rect[1] = points[np.argmin(diff)]
    rect[3] = points[np.argmax(diff)]

    return rect
