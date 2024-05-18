import numpy as np
import json
import cv2 as cv
from scipy import stats
import unicodedata
import logging
import sys
import exif

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

LEFT_CROP_PIXELS = 20
BOTTOM_CROP_PIXELS = 11
RIGHT_CROP_PIXELS = 9
TOP_CROP_PIXELS = 11

def manhattan_distance(point1, point2):
    x1,y1 = point1
    x2,y2 = point2
    return abs(x2-x1) + abs(y2-y1)
    
    
def spiral_order(pieces):
    """Returns the pieces in 'spiral' order, starting at the origin and extending out in a clockwise spiral."""

    # Make sure 'pieces' is a proper list and not merely an iterable,
    # because we need to iterate often and slice it and so forth.
    pieces = list(pieces)
    
    # If pieces is empty, return an empty list
    if len(pieces) == 0: return []

    # First find the number of rows and cols
    max_x, max_y = max(pieces)
    rows = max_y + 1
    cols = max_x + 1
    
    # Compute the list in lexical order (left to right, top to bottom, considering top left as origin), to help us later
    pieces_in_lexical_order = []
    for y in range(rows):
        for x in range(cols):
            pieces_in_lexical_order.append([x,y])
    
    # Reshape that into an array of arrays (of point tuples)
    pieces_in_lexical_order = np.array(pieces_in_lexical_order).reshape([rows, cols, 2])
    
    # Slice it in a spiral pattern
    pieces_in_spiral_order = []
    while (len(pieces_in_lexical_order) > 0):
        # Take the top row (the first array) and add it to the output
        row = pieces_in_lexical_order[0]
        pieces_in_spiral_order.extend(row)
        # Now remove that top row
        pieces_in_lexical_order = pieces_in_lexical_order[1:]
        # Now rotate the array counter-clockwise
        pieces_in_lexical_order = np.rot90(pieces_in_lexical_order) 
        
    return pieces_in_spiral_order

    
    
def anchor_order(pieces):
    """Returns the pieces in 'anchor' order, starting at the origin and extending out in a triangular fashion."""
    
    # Make sure 'pieces' is a proper list and not merely an iterable,
    # because we need to iterate often and slice it and so forth.
    pieces = list(pieces)
    
    # If pieces is empty, return an empty list
    if len(pieces) == 0: return []
    
    # First find the maximum x and y values
    max_x, max_y = max(pieces)

    pieces_in_anchor_order = []
    total_pieces_to_place = (max_x + 1) * (max_y + 1)

    # Put the origin piece (0,0) in the list to start things off
    distance = 0
    origin = [0,0]
    pieces_in_anchor_order.append(origin)

    while len(pieces_in_anchor_order) < total_pieces_to_place:
        # Find all the pieces at the current manhattan distance
        distance += 1
        pieces_at_distance = [p for p in pieces if manhattan_distance(origin, p) == distance]
        # Sort those pieces by x (ascending, starting at x=0)
        pieces_at_distance.sort(key=lambda p: p[0])
        # Add those pieces to the solution
        pieces_in_anchor_order.extend(pieces_at_distance)

    return pieces_in_anchor_order

def open_image_undistorted_and_rotated(image_path, camera_matrix, distortion_coefficients):
    """Open image_path as an opencv image, undistort it, rotate if necessary, and return the image"""
    # Open the image using cv
    img = cv.imread(image_path)

    # Rotate the image before we un-distort, because camera matrix and distortion parameters were
    # computed on images in the normal horizontal rotation, so we need to rotate to that orientation
    # before we undistort.

    # If exif orientation information exists, rotate img accordingly
    # Note that this rotation uses a matrix transformation so no information is lost
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
        logger.error(f"[open_image_and_fix_orientation] Error reading exif: {e}")
    
    # In some scenarios the phone will output a photo in vertical format
    # but will set the orientation flag to normal. To counteract this,
    # rotate 90CW if the photo seems to be in vertical format (height > width)
    height, width, channels = img.shape
    if height > width:
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
    
    # Convert pre-calculated intrinsics to np arrays if they are not already np arrays
    camera_matrix = np.float32(camera_matrix)
    distortion_coefficients = np.float32(distortion_coefficients)

    # Undistort the image (causes a black border to appear around the image.. will be cropped later in this function)
    img = cv.undistort(img, camera_matrix, distortion_coefficients)
    
    # Finally crop the image to get rid of the empty space caused by the un-distortion
    height, width, channels = img.shape # Get these again because rotate may have changed them!
    start_row = TOP_CROP_PIXELS
    end_row = height - BOTTOM_CROP_PIXELS
    start_col = LEFT_CROP_PIXELS
    end_col = width - RIGHT_CROP_PIXELS
    img = img[start_row:end_row, start_col:end_col]
    
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
    