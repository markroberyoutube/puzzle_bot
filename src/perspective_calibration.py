import numpy as np
import os, sys
import cv2 as cv
import math
from scipy import stats
from utils import open_image_undistorted_and_rotated, get_point_on_line, sort_corners, \
                  estimate_linear_regression_coefficients, get_intersect

def rotate_image(img, angle_degrees, crop=True):
    """Rotate img by angle_degrees, and optionally crop to remove undefined border pixels.
    
       Note: Angle should be positive for counter-clockwise and negative for clockwise.
    """
    
    # Rotate image about its center
    image_center = tuple(np.array(img.shape[1::-1]) / 2)
    scale = 1.0
    rot_mat = cv.getRotationMatrix2D(image_center, angle_degrees, scale)
    rotated_img = cv.warpAffine(img, rot_mat, img.shape[1::-1], flags=cv.INTER_NEAREST)
    
    # Determine rotation direction
    counter_clockwise_rotation = angle_degrees > 0
  
    # Crop out undefined pixels if requested
    if crop:
        # Math reference: https://docs.opencv.org/4.x/d4/d61/tutorial_warp_affine.html
        
        # Convert corners (upper right, bottom left, etc) to homogenous coordinates (add the ,1)
        # and transpose to get them into a vertical column
        height, width, channels = img.shape 
        ul = np.transpose(np.array([0,0,1]))
        ur = np.transpose(np.array([width, 0, 1]))
        lr = np.transpose(np.array([width, height, 1]))
        ll = np.transpose(np.array([0, height, 1]))
        
        # Rotate those corners about the center 
        new_ul = np.dot(rot_mat, ul)
        new_ur = np.dot(rot_mat, ur)
        new_lr = np.dot(rot_mat, lr)
        new_ll = np.dot(rot_mat, ll)
        
        # Determine which x and y values to crop to
        crop_left_x = 0
        crop_top_y = 0
        crop_right_x = 0
        crop_bottom_y = 0
        
        if counter_clockwise_rotation:
            crop_left_x = new_ll[0] # Crop left to rotated lower left X value
            crop_right_x = new_ur[0] # Crop right to rotated upper right X value
            
            # Crop top to where rotated upper edge intersects X=new_ll_x
            rotated_upper_edge = [new_ul, new_ur]
            new_ll_x_line = [(new_ll[0], 0), (new_ll[0], height)]
            intersection_point = get_intersect(rotated_upper_edge, new_ll_x_line)
            crop_top_y = intersection_point[1] # Crop top to intersection Y value
            
            # Crop bottom to where rotated lower edge intersects X=new_ur_x
            rotated_lower_edge = [new_ll, new_lr]
            new_ur_x_line = [(new_ur[0], 0), (new_ur[0], height)]
            intersection_point = get_intersect(rotated_lower_edge, new_ur_x_line)
            crop_bottom_y = intersection_point[1] # Crop bottom to intersection Y value
        else:
            # For clockwise rotation, basically the opposite points are selected as crop points
            
            crop_left_x = new_ul[0] # Crop left to rotated upper left X value
            crop_right_x = new_lr[0] # Crop right to rotated lower right X value
            
            # Crop top to where rotated upper edge intersects X=new_lr_x
            rotated_upper_edge = [new_ul, new_ur]
            new_lr_x_line = [(new_lr[0], 0), (new_lr[0], height)]
            intersection_point = get_intersect(rotated_upper_edge, new_lr_x_line)
            crop_top_y = intersection_point[1] # Crop top to intersection Y value
            
            # Crop bottom to where rotated lower edge intersects X=new_ul_x
            rotated_lower_edge = [new_ll, new_lr]
            new_ul_x_line = [(new_ul[0], 0), (new_ul[0], height)]
            intersection_point = get_intersect(rotated_lower_edge, new_ul_x_line)
            crop_bottom_y = intersection_point[1] # Crop bottom to intersection Y value
    
        # Round the crop values to ensure a full crop
        crop_left_x = math.ceil(crop_left_x)
        crop_right_x = math.floor(crop_right_x)
        crop_top_y = math.ceil(crop_top_y)
        crop_bottom_y = math.floor(crop_bottom_y)
        
        # Finally crop the image
        rotated_img = rotated_img[crop_top_y:crop_bottom_y, crop_left_x:crop_right_x]
  
    # Return the resulting image
    return rotated_img

def calibrate_rotation(img1, img2):
    """Given two images of a checkerboard, offset by a motor movement in X direction only,
       calculate the angle between the X motor axis and the X axis of the image."""
    
    num_columns = 3 # num corners in a horizontal row (can be undersized, findCorners will find all corners)
    num_rows = 14 # num corners in a vertical column (all of these should be visible for this code to work)
    
    # Find all corners in first image
    ret1, corners1 = cv.findChessboardCornersSB(
        img1, 
        (num_rows, num_columns), 
        flags=cv.CALIB_CB_LARGER + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY
    )
    if not ret1:
        print("ERROR: Could not find corners in first image")
        sys.exit(-1)
    
    # If any columns do not show all corners, exit with an error
    if len(corners1) % num_rows != 0:
        print("ERROR: Some columns are partially cut off")
        sys.exit(-1)
    
    # Find all corners in second image
    ret2, corners2 = cv.findChessboardCornersSB(
        img2, 
        (num_rows, num_columns), 
        flags=cv.CALIB_CB_LARGER + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY
    )
    if not ret2:
        print("ERROR: Could not find corners in second image")
        sys.exit(-1)

    # If any columns do not show all corners, exit with an error
    if len(corners2) % num_rows != 0:
        print("ERROR: Some columns are partially cut off")
        sys.exit(-1)

    # Slice up the array of corners from one long list into a 2D array of rows and columns
    num_cols1 = len(corners1) // num_rows
    num_cols2 = len(corners2) // num_rows
    
    corners1 = corners1.squeeze().reshape(num_rows, num_cols1, 2)
    corners2 = corners2.squeeze().reshape(num_rows, num_cols2, 2)
    
    # Compare the locations of the corners in the first image with the same corners found in the second image.
    # radius = 3
    # color1 = [0,255,0]
    # color2 = [255,0,0]
    # thickness = cv.FILLED
    angle_count = 0
    angle_sum = 0
    for row_idx, row in enumerate(corners1):
        for col_idx, point in enumerate(row):
            x1,y1 = point

            # find the corresponding point in the second image
            x2,y2 = corners2[row_idx][col_idx]
            
            # Draw a green circle around the corner in the first image, and a red
            # circle around the corner in the second image
            # img1 = cv.circle(img1, (int(x1),int(y1)), radius, color1, thickness)
            # img1 = cv.circle(img1, (int(x2),int(y2)), radius, color2, thickness)
            
            # Compute the angle between the X axis and a line segment connecting these two points
            delta_x = x1 - x2
            delta_y = y1 - y2
            angle_radians = math.atan(delta_y / delta_x)
            angle_degrees = math.degrees(angle_radians)
            angle_sum += angle_degrees
            print(angle_degrees)
            angle_count += 1
            
            # Increase the radius to make it easier to see the order the corners were found in
            #radius += 1

    # cv.imshow("rotated", rotate_image(img1, average_angle_degrees, True))
    # cv.waitKey(0)
    
    # Finally compute the average angle
    average_angle_degrees = angle_sum / angle_count
    print(f"Average Angle Degrees: {average_angle_degrees}")
    
    return average_angle_degrees

def correct_perspective(img, starting_quad, corrected_quad, crop=True):
    """Return img that has been perspective corrected using the supplied quads (sets of four corner points)."""
    height, width, channels = img.shape
    correction_matrix = cv.getPerspectiveTransform(starting_quad, corrected_quad)
    img = cv.warpPerspective(
        img, correction_matrix, (width, height), flags=cv.INTER_NEAREST, borderMode=cv.BORDER_CONSTANT, borderValue=(0,255,0)
    )
    if crop:
        # Determine crop values to eliminate invalid pixel areas. This is a two step process:
        # 1) Find warped positions of the original image corners
        # 2) Find the crop values each edge based on which warped corner is closest to the center of the image

        # First specify original corners in homogeneous coordinates
        ul = np.transpose(np.array([0,0,1]))
        ur = np.transpose(np.array([width,0,1]))
        lr = np.transpose(np.array([width,height,1]))
        ll = np.transpose(np.array([0,height,1]))
    
        # Now find the warped positions of those corners. 
        new_ul = np.dot(correction_matrix, ul)
        new_ur = np.dot(correction_matrix, ur)
        new_lr = np.dot(correction_matrix, lr)
        new_ll = np.dot(correction_matrix, ll)
    
        # Don't forget to scale by z as shown here: https://stackoverflow.com/questions/45010881/
        new_ul /= new_ul[2]
        new_ur /= new_ur[2]
        new_lr /= new_lr[2]
        new_ll /= new_ll[2]
    
        # Find crop values based on which corner on that edge is closest to the center of the image
        crop_right_to_x = np.int32(np.floor(np.min([new_ur[0], new_lr[0]])))
        crop_left_to_x = np.int32(np.ceil(np.max([new_ul[0], new_ll[0]])))
        crop_top_to_y = np.int32(np.ceil(np.max([new_ul[1], new_ur[1]])))
        crop_bottom_to_y = np.int32(np.floor(np.min([new_ll[1], new_lr[1]])))
    
        # Crop the image
        img = img[crop_top_to_y:crop_bottom_to_y, crop_left_to_x:crop_right_to_x]
    
    # Return the results
    return img

def calibrate_perspective(img, checker_columns, checker_rows):
    """Given an image of a full checkerboard, calculate the two quads needed
       to correct the perspective so that the parallax is eliminated (in other
       words, so the checkerboard appears as a perfect rectangle).
    """

    # Find corners in the images
    features_x = checker_columns - 1
    features_y = checker_rows - 1
    
    # Find all chessboard corners
    ret, corners = cv.findChessboardCornersSB(
        img, 
        (features_x, features_y), 
        flags=cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY
    )
    if not ret:
        print("ERROR: Could not find all corners")
        sys.exit(-1)

    # Slice up the array of corners from one long list into a 2D array of rows and columns
    corners = corners.squeeze().reshape(features_y, features_x, 2) # the 2 at the end is for a 2D point
    
    # Because cv.findChessboardCornersSB does not return the corners in a consistent order, we need to
    # reorder the corners so that the starting corner is in a consistent place (we'll pick the lower left 
    # corner as our starting corner)
    corners = sort_corners(corners)
    
    # Draw circles at each corner, growing by radius to make it easier to visualize if the 
    # order is correct (growing in size from left to right, and bottom to top)
    # radius = 5
    # color = (0,255,0)
    # thickness = 2
    # for row_idx, row in enumerate(corners):
    #     for col_idx, point in enumerate(row):
    #         x,y = point
    #         img = cv.circle(img, (int(x),int(y)), radius, color, thickness)
    #         radius += 1
    # cv.imshow("img", img)
    # cv.waitKey(0)

    # Do linear regression to find lines fitting the 4 edges (top, right, left, bottom)
    top_edge    = corners[-1]
    bottom_edge = corners[0]
    left_edge   = np.array([row[0] for row in corners])
    right_edge  = np.array([row[-1] for row in corners])
    
    top_intercept, top_slope = estimate_linear_regression_coefficients(top_edge)
    bottom_intercept, bottom_slope = estimate_linear_regression_coefficients(bottom_edge)
    left_intercept, left_slope = estimate_linear_regression_coefficients(left_edge)
    right_intercept, right_slope = estimate_linear_regression_coefficients(right_edge)
    
    # Find two points on each line, which we'll use to calculate intersections
    height, width, channels = img.shape
    top_p1    = (0, top_intercept)
    bottom_p1 = (0, bottom_intercept)
    left_p1   = (0, left_intercept)
    right_p1  = (0, right_intercept)
    top_p2    = (width, top_intercept + top_slope*width)
    bottom_p2 = (width, bottom_intercept + bottom_slope*width)
    left_p2   = (width, left_intercept + left_slope*width)
    right_p2  = (width, right_intercept + right_slope*width)

    # Find the quad where those lines intersect (ex: top left point is where top and left edge intersect)
    tl = get_intersect((top_p1, top_p2), (left_p1, left_p2))
    tr = get_intersect((top_p1, top_p2), (right_p1, right_p2))
    br = get_intersect((bottom_p1, bottom_p2), (right_p1, right_p2))
    bl = get_intersect((bottom_p1, bottom_p2), (left_p1, left_p2))
    
    # Calculate the corrected quad as follows:
    # The top edge should be the existing top edge
    new_tl = tl
    new_tr = tr
    # The side edge lengths should be proportionally correct to the top edge length
    # (given that we know the relative lengths from the checkerboard width/height ratio )
    top_edge_length = np.linalg.norm(np.array(tr)-np.array(tl))
    checker_ratio = (features_x-1) / (features_y-1) # width/height ratio
    side_edge_length = top_edge_length / checker_ratio
    # The side edges should be perpendicular to the top edge (i.e. negative slope from top edge)
    # So this is a 2 stop process: 1) calculate perpendicular slope, 2) find point on perpendicular line at desired distance
    # Calculate a point anywhere on the new left edge (used for the next computation)
    top_edge_slope = (tr[1]-tl[1]) / (tr[0]-tl[0])
    side_edge_slope = -1 / top_edge_slope # Perpendicular to top edge
    # Calculate location of new left bottom corner
    left_edge_arbitrary_point_y = tl[1] + height
    left_edge_arbitrary_point_x = ((left_edge_arbitrary_point_y - tl[1]) / side_edge_slope) + tl[0] # (y2-y1)/slope + x1
    new_bl = get_point_on_line([tl, (left_edge_arbitrary_point_x, left_edge_arbitrary_point_y)], side_edge_length)
    # Calculate location of new right bottom corner
    right_edge_arbitrary_point_y = tr[1] + height
    right_edge_arbitrary_point_x = ((right_edge_arbitrary_point_y - tr[1]) / side_edge_slope) + tr[0] # (y2-y1)/slope + x1
    new_br = get_point_on_line([tr, (right_edge_arbitrary_point_x, right_edge_arbitrary_point_y)], side_edge_length)
    
    # Assemble the starting and corrected quads
    starting_quad = np.float32([tl, tr, br, bl])
    corrected_quad = np.float32([tl, tr, new_br, new_bl])

    # Draw circles at each quad point
    # radius = 20
    # starting_quad_color = (255,0,0)
    # corrected_quad_color = (0,255,0)
    # thickness = 10
    # for p in starting_quad:
    #     img = cv.circle(img, np.int32(p), radius, starting_quad_color, thickness)
    # for p in corrected_quad:
    #     img = cv.circle(img, np.int32(p), radius, corrected_quad_color, thickness)
    # cv.imshow("img", img)
    # cv.waitKey(0)
    
    # Return the results
    return starting_quad, corrected_quad


def main():
    camera_matrix = [[1.38159463e+04, 0.00000000e+00, 1.99782976e+03],
                     [0.00000000e+00, 1.38159463e+04, 1.57020242e+03],
                     [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
    distortion_coefficients = [[0.27549, -2.26908, 0,  0.00023,  0.00480]]
    
    # The first image should show all the rows, and at least the first 3 columns of the checkerboard,
    # and the checkerboard should be off screen to the right. For each row, the same number of columns
    # should be visible.
    img1 = open_image_undistorted_and_rotated("start.jpg", camera_matrix, distortion_coefficients)
    
    # The second image should show the checkerboard having been moved to the left quite a bit so that 
    # all of it is visible.
    img2 = open_image_undistorted_and_rotated("camera_delta.jpg", camera_matrix, distortion_coefficients)

    # Specify the checkerboard size
    checker_columns = 24
    checker_rows = 15
    
    # Calculate the two quads needed to correct perspective
    starting_quad, corrected_quad = calibrate_perspective(img2, checker_columns, checker_rows)
    
    # Correct the perspective of both images
    img1 = correct_perspective(img1, starting_quad, corrected_quad)
    img2 = correct_perspective(img2, starting_quad, corrected_quad)
    
    # Calculate the rotation needed to make the image X axis parallel to the motor X axis
    angle_degrees = calibrate_rotation(img1, img2)
    
    # Rotate and crop the images
    img1 = rotate_image(img1, angle_degrees, crop=True)
    img2 = rotate_image(img2, angle_degrees, crop=True)
    
    # Write the final images to disk
    cv.imwrite("start_corrected.jpg", img1)
    cv.imwrite("camera_delta_corrected.jpg", img2)
