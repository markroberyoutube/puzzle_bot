import numpy as np
import json
import cv2 as cv
from scipy import stats
import unicodedata

def open_image_undistorted_and_rotated(image_path, camera_matrix, distortion_coefficients):
    """Open image_path as an opencv image, undistort it, rotate if necessary, and return the image"""
    # Open the image using cv
    img = cv.imread(image_path)
    
    # Convert pre-calculated intrinsics to np arrays if they are not already np arrays
    camera_matrix = np.float32(camera_matrix)
    distortion_coefficients = np.float32(distortion_coefficients)

    # Undistort the image
    img = cv.undistort(img, camera_matrix, distortion_coefficients)

    # If exif orientation information exists, rotate img accordingly
    try:
        with open(image_path, 'rb') as image_file:
            exif_img = exif.Image(image_file)
            orientation = exif_img.orientation
            if orientation == 1: # Normal orientation, no rotation
                pass
            elif orientation == 3: # Rotate 180
                img = cv.rotate(img, cv.ROTATE_180)
            elif orientation == 6: # Rotated 90 CCW, counteract that
                img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
            elif orientation == 8: # Rotated 270 CCW, counteract that
                img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
            else:
                logger.error(f"[open_image_and_fix_orientation] Unsupported jpg orientation: {orientation}")
    except Exception as e:
        pass
    
    # In some scenarios the phone will output a photo in vertical format
    # but will set the orientation flag to normal. To counteract this,
    # rotate 90CW if the photo seems to be in vertical format (height > width)
    height, width, channels = img.shape
    if height > width:
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
    
    return img

def parse_int(data):
    """Parses the first contiguous integer from data"""    
    result_chars = []
    for c in data:
        if c.isdigit() or c == "-":
            result_chars.append(c)
        else:
            if result_chars:
                break
    if result_chars:
        return int("".join(result_chars))
    else:
        return None

def estimate_linear_regression_coefficients(points):
    """Return the coefficients (a and b) for a y=a+bx simple linear regression, given a list points (x,y tuples)"""
    x_data = [p[0] for p in points]
    y_data = [p[1] for p in points]
    slope, intercept, r, p, std_err = stats.linregress(x_data, y_data)
    return (intercept, slope)

def minimum_distance(point, line):
    """Returns the minimum distance between a point and a line (a tuple of two points p1 and p2 on that line)"""
    # Make sure types are numpy types
    point = np.float32(point)
    line = np.float32(line)
    
    # Unpack the points representing the line
    p1, p2 = line
    
    # See https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
    distance = np.cross(p2-p1, point-p1)/np.linalg.norm(p2-p1)
    return distance

def get_escape_points(line, width, height):
    """Returns the points at which the line (a tuple of two points p1 and p2) leaves an image of specified width and height"""
    escape_points = []
    
    # Unpack the points representing the line
    p1, p2, = line
    
    # See if line leaves the top edge of the image
    x,y = get_intersect([p1, p2], [(0,0), (width, 0)])
    if x > 0 and x < width:
        escape_points.append([x,y])
        
    # See if line leaves the right edge of the image
    x,y = get_intersect([p1, p2], [(width, 0), (width, height)])
    if y > 0 and y < height:
        escape_points.append([x,y])
    
    # See if line leaves the bottom edge of the image
    x,y = get_intersect([p1, p2], [(width, height), (0, height)])
    if x > 0 and x < width:
        escape_points.append([x,y])
    
    # See if line leaves the left edge of the image
    x,y = get_intersect([p1, p2], [(0, height), (0, 0)])
    if y > 0 and y < height:
        escape_points.append([x,y])
    
    return escape_points

def get_intersect(line_a, line_b):
    """ 
    Returns the point of intersection of the specified lines, where each line
    is represented by a tuple of points that line passes through. Meaning
    line_a is composed of points a1 and a2 while line_b is composed of 
    two points b1 and b2.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    # Algo from https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
    a1, a2 = line_a
    b1, b2 = line_b
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return (float('inf'), float('inf'))
    return (x/z, y/z)

def order_rectangle_corners(points):
    """Return the 4 rectangle corners re-ordered in our canonical order (top_left, top_right, bottom_right, bottom_left)"""
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype = "int32")

    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = points.sum(axis = 1)
    rect[0] = points[np.argmin(s)]
    rect[2] = points[np.argmax(s)]

    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(points, axis = 1)
    rect[1] = points[np.argmin(diff)]
    rect[3] = points[np.argmax(diff)]

    # return the ordered coordinates
    return rect
    