import sys, os, logging, tempfile, math, glob, posixpath
import cv2 as cv
import numpy as np
import json
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThread

from utils import minimum_distance, get_escape_points, get_intersect, order_rectangle_corners, open_image_undistorted_and_rotated
from perspective_calibration import correct_perspective, rotate_image

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

# from the Miranda motor datasheet: https://www.mouser.com/pdfDocs/201215miranda-manual-ovz20008_rev5.pdf
ROTATION_MOTOR_COUNTS_PER_DEGREE = 182

class GripperCalibration(QThread):
    """"Gripper Calibration

    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    """
    
    # Define QT Signals for inter-process communication
    trigger_gripper_calibration = pyqtSignal(str, list, list, float, list, list)
    gripper_calibration_results_ready = pyqtSignal(list, list)
    
    thread_closing = pyqtSignal() # Emit to this when you want the thread to exit
    
    def __init__(self, parent):
        # Superclass initializer
        QThread.__init__(self, parent)
        
        # Connect signals for inter-process communication
        self.trigger_gripper_calibration.connect(self.calibrate_gripper)
        self.thread_closing.connect(self.exit_thread)
        
        # Set to True to cause the Thread's main run loop to exit
        self._thread_exiting = False


    def gradient_descent_cost(self, point, lines):
        """Calculate the sum of squares of the minimum distances between the point and each line
    
        Note that lines are represented here by a tuple of two points (p1, p2)
    
        """
        # For each line, calculate the squared minimum distance between the line and the rotation
        cost = 0
        for line in lines:
            cost += minimum_distance(point, line)**2
        return cost

    def find_chessboard(self, original_img, debug=False):
        """Find and return the chessboard feature locations
        
        Returns corners, masked_img
        
        corners: chessboard corners np.array of float32, shape: (num_corners, 1, 2)
        masked_img: cv image showing the utilized quadrant of the chessboard and nothing else
        
        Note that our target has a weird marker in the middle that opencv's findChessboardCorners
        function errors out on, so what we do is use opencv to turn everything white in that image
        except for one quadrant of the chessboard (thus eliminating the marker) and then do our 
        chessboard recognition on that.
        
        """
    
        # Convert image to grayscale
        gray_img = cv.cvtColor(original_img, cv.COLOR_BGR2GRAY)

        # The following process is guided from advice from here:
        # https://stackoverflow.com/questions/55169645/square-detection-in-image

        # Median blur a LOT to eliminate noise, then sharpen
        blur_img = cv.medianBlur(gray_img, 45)
        if debug: cv.imshow('blur', blur_img)
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen_img = cv.filter2D(blur_img, -1, sharpen_kernel)
        if debug: cv.imshow('sharpen', sharpen_img)
    
        # Threshold to a binary image
        THRESHOLD = 100 # ICC changed from 200
        thresh_img = cv.threshold(sharpen_img, 100, 255, cv.THRESH_BINARY)[1]
        if debug: cv.imshow('thresh', thresh_img)
    
        # Morph close to eliminate sharpening noise
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
        close_img = cv.morphologyEx(thresh_img, cv.MORPH_CLOSE, kernel, iterations=2)
        if debug: cv.imshow('close', close_img)

        # Find contours and filter using threshold area
        contours = cv.findContours(close_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        # Find the largest contour and assume it's our checkerboard
        checkerboard_contour = None
        for c in contours:
            if checkerboard_contour is None or cv.contourArea(c) > cv.contourArea(checkerboard_contour):
                checkerboard_contour = c
            
        # Find a rotated bounding box representing the minimal bounding box
        bounding_rect = cv.minAreaRect(checkerboard_contour) # Find bounding rect
        bounding_box = cv.boxPoints(bounding_rect) # Get the 4 corner points of the rect
        bounding_box = np.int32(bounding_box) # Turn the cv points into numpy points
    
        # Since there's an undetectable marker in the middle of the chessboard, 
        # crop the bounding box to only show basically a quadrant that crops out the marker.
        # This is the upper right quadrant of a properly oriented (rotated) image
        ordered_points = order_rectangle_corners(bounding_box) # Order points in canonical order
        top_left, top_right, bottom_right, bottom_left = ordered_points
    
        new_top_right = top_right
        new_top_left = np.int32(top_left + (top_right - top_left)/2)
        new_bottom_right = np.int32(top_right + (bottom_right - top_right)/2)
        new_bottom_left = np.int32(top_left + (bottom_right - top_left)/2)
    
        bounding_box = np.array([new_top_left, new_top_right, new_bottom_right, new_bottom_left])
    
        # Create a mask out of that rect
        mask = np.zeros(original_img.shape[:2], dtype="uint8") # Create empty mask (all pixels black)
        rect_mask = cv.drawContours(
            mask, 
            [bounding_box], # list of contours
            0, # index into list of contours
            (255,255,255), # White color
            cv.FILLED # thickness, or cv2.FILLED to have a fill but no border
        )
    
        # Now use the mask to mask out the desired quadrant of the checkerboard in the grayscale image
        masked_img = np.copy(gray_img)
        masked_img[mask == 0] = [255] # If mask is black turn that pixel white in the output image
        if debug: cv.imshow('masked', masked_img)
        if debug: cv.waitKey()
            
        # Find the features
        features_x, features_y = (24, 24)
        retcode, corners = cv.findChessboardCornersSB(
            masked_img,
            (features_x, features_y),
            cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY
        )
    
        return corners, masked_img

    @pyqtSlot(str, list, list, float, list, list)
    def calibrate_gripper(self,
        gripper_calibration_batch_directory, 
        camera_matrix, 
        distortion_coefficients,
        perspective_angle,
        perspective_starting_quad,
        perspective_corrected_quad):

        # Set to True if you want to see a lot of imview windows pop up
        debug = False

        # Convert the quads to numpy arrays
        perspective_starting_quad = np.float32(perspective_starting_quad)
        perspective_corrected_quad = np.float32(perspective_corrected_quad)

        # Define data input paths
        batch_dir = gripper_calibration_batch_directory
        start_image_path = posixpath.join(batch_dir, "start.jpg")
        camera_delta_image_path = posixpath.join(batch_dir, "camera_delta.jpg")
        gripper_delta_image_path = posixpath.join(batch_dir, "gripper_delta.jpg")
        batch_info_path = posixpath.join(batch_dir, "gripper_calibration_info.json")
        
        # Define data output paths
        output_image_path = posixpath.join(batch_dir, "gripper_calibration_output.jpg")
    
        # Open the config file
        batch_info = None
        with open(batch_info_path, "r") as jsonfile:
            batch_info = json.load(jsonfile)
            
        # Unpack the config values
        start_x_counts = batch_info["start_x"]
        start_y_counts = batch_info["start_y"]
        camera_delta_x_counts = batch_info["camera_delta_x"]
        camera_delta_y_counts = batch_info["camera_delta_y"]
        gripper_delta_x_counts = batch_info["gripper_delta_x"]
        gripper_delta_y_counts = batch_info["gripper_delta_y"]

        # Open the images
        start_img = open_image_undistorted_and_rotated(start_image_path, camera_matrix, distortion_coefficients)
        camera_delta_img = open_image_undistorted_and_rotated(camera_delta_image_path, camera_matrix, distortion_coefficients)
        gripper_delta_img = open_image_undistorted_and_rotated(gripper_delta_image_path, camera_matrix, distortion_coefficients)

        # Correct the perspective of the images
        start_img = correct_perspective(start_img, perspective_starting_quad, perspective_corrected_quad, crop=True)
        camera_delta_img = correct_perspective(camera_delta_img, perspective_starting_quad, perspective_corrected_quad, crop=True)
        gripper_delta_img = correct_perspective(gripper_delta_img, perspective_starting_quad, perspective_corrected_quad, crop=True)

        # Rotate the images so the image X axis is parallel with the motor X axis
        start_img = rotate_image(start_img, perspective_angle, crop=True)
        camera_delta_img = rotate_image(camera_delta_img, perspective_angle, crop=True)
        gripper_delta_img = rotate_image(gripper_delta_img, perspective_angle, crop=True)

        # Find the chessboards
        start_chessboard, masked_start_img = self.find_chessboard(start_img)
        camera_delta_chessboard, masked_camera_delta_img = self.find_chessboard(camera_delta_img)
        gripper_delta_chessboard, masked_gripper_delta_img = self.find_chessboard(gripper_delta_img)
    
        # Annotation parameters
        blue_color = (255, 234, 0) # BGR, so this is blue
        green_color = (0, 255, 42) # BGR, so this is green
        white_color = (255, 255, 255)
        black_color = (0, 0, 0)
        pink_color = (192, 0, 255) # pink
        circle_thickness = 5
        circle_radius = 10
        
        ################################################################################################
        ## CALCULATE MOTOR COUNTS PER PX IN BOTH X AND Y USING THIS ALGORITHM:
        ##
        ## Prior to this function being called:
        ## 1. Take a photo of the target (this is called the "start photo")
        ## 2. Move the camera to a slightly different location (motor_counts_x and motor_counts_y) 
        ## 3. Take another photo (this is called the "camera calibration" photo)
        ## 
        ## In this function we finish the process by performing the following:
        ## 4. Find the checkerboard features in each image
        ## 5. For corresponding checkerboard features, calculate the average delta_x_pixels and 
        ##    average delta_y_pixels that the feature appeared to move from one photo to another.
        ## 6. Now simply divide the motor counts moved by the pixel deltas observersed, and
        ##    we have the relationship we wanted
    
        # Draw green circles on the start chessboard
        for point in start_chessboard:
            point = point[0] # Strip off the unnecessary outer array
            start_img = cv.circle(start_img, np.int32(point).tolist(), circle_radius, green_color, circle_thickness)
            if debug:
                cv.imshow('start', start_img)
                cv.waitKey(1)
        cv.imwrite(posixpath.join(batch_dir, "start-annotated.jpg"), start_img)
    
        # Draw blue circles on the camera delta chessboard
        for point in camera_delta_chessboard:
            point = point[0] # Strip off the unnecessary outer array
            camera_delta_img = cv.circle(camera_delta_img, np.int32(point).tolist(), circle_radius, pink_color, circle_thickness)
            if debug:
                cv.imshow('camera_delta', camera_delta_img)
                cv.waitKey(1)
        cv.imwrite(posixpath.join(batch_dir, "camera-delta-annotated.jpg"), camera_delta_img)
    
        # Draw pink circles on the gripper delta chessboard
        for point in gripper_delta_chessboard:
            point = point[0] # Strip off the unnecessary outer array
            gripper_delta_img = cv.circle(gripper_delta_img, np.int32(point).tolist(), circle_radius, blue_color, circle_thickness)
            if debug:
                cv.imshow('gripper_delta', gripper_delta_img)
                cv.waitKey(1)
        cv.imwrite(posixpath.join(batch_dir, "gripper-delta-annotated.jpg"), gripper_delta_img)
    
        # Calculate motor_counts / px in both x and y. We do this for x by averaging the delta_x values
        # for all corresponding points, and then taking delta_x_motor_counts / delta_x_px
        # We repeat the same process for y
        delta_x_px_sum = 0
        delta_y_px_sum = 0
        num_points = 0
        for start_point, camera_delta_point in zip(start_chessboard, camera_delta_chessboard):
            # Strip off the outer array that cv adds
            start_point = start_point[0]
            camera_delta_point = camera_delta_point[0]
        
            # Find the x,y coordinates
            start_x, start_y = start_point
            camera_delta_x, camera_delta_y = camera_delta_point
        
            # Sum the x and y differences
            num_points += 1
            delta_x_px_sum += camera_delta_x - start_x
            delta_y_px_sum += camera_delta_y - start_y
        
        # Find the averages
        delta_x_px_avg = delta_x_px_sum / num_points
        delta_y_px_avg = delta_y_px_sum / num_points

        logging.debug(f"delta_x_px_avg: {delta_x_px_avg}")
        logging.debug(f"delta_y_px_avg: {delta_y_px_avg}")
        
        logging.debug(f"moved x counts from start to camera delta: {camera_delta_x_counts - start_x_counts}")
        logging.debug(f"moved y counts from start to camera delta: {camera_delta_y_counts - start_y_counts}")
        
        # Finally compute motor counts per pixel counts in both X and Y
        x_counts_per_px_x = (camera_delta_x_counts - start_x_counts) / delta_x_px_avg
        y_counts_per_px_y = (camera_delta_y_counts - start_y_counts) / delta_y_px_avg
        motor_counts_per_px = [x_counts_per_px_x, y_counts_per_px_y]
        
        logging.debug(f"x_counts_per_px_x: {x_counts_per_px_x}")
        logging.debug(f"y_counts_per_px_y: {y_counts_per_px_y}")
    
    
        ################################################################################################
        ## CALCULATE GRIPPER X,Y TRANSFORM USING OUR ALGORITHM:
        ##
        ## Prior to this function being called:
        ## 1. Move the gripper roughly to the middle of the target and remember the x and y 
        ##    motor counts we've moved since the start photo.
        ## 2. Rotate the target an arbitrary amount (say, around 30 deg - definitely less than 45 degrees
        ##    otherwise we might have a hard time figuring out corresponding features between photos)
        ## 3. Move the camera back to the "start photo" position and take another photo 
        ##    (this is called the "gripper calibration photo")
        ## 
        ## In this function we finish the process by performing the following:
        ## 4. Find the checkerboard features in each image
        ## 5. Imagine a line going from a feature in the start_photo to the same feature
        ##    in the gripper calibration photo. At the midpoint of this line, calculate a
        ##    perpendicular line. Do this for all of the features in the checkerboard
        ## 6. Where those lines intersect should be the center of rotation, meaning the
        ##    gripper pickup location, which helps us find the relationship between the gripper and
        ##    the camera. However those lines are real data so the lines don't all interesect perfectly
        ##    at the same point. So, we use gradient descent to find the optimal guess as to the 
        ##    rotation point. To do that, we define a cost function that is the sum of squares of the
        ##    distances from the point to each of those lines. As an initial estimate we guess the
        ##    intersection of the first two lines. Then we perform gradient descent to move that guess
        ##    around until we minimize the cost function. For our specific situation this should find 
        ##    the global minima, not just a local one, because the cost function's surface should look
        ##    like an inverted cone that funnels us to the correct answer.
    
        # Now we'll calculate the gripper x,y transform using our method.
        perpendicular_lines = [] # Will be filled with (p1, p2) tuples representing line segments
        for start_point, gripper_delta_point in zip(start_chessboard, gripper_delta_chessboard):
            # Strip off the outer array that cv adds
            start_point = start_point[0]
            gripper_delta_point = gripper_delta_point[0]
        
            # Find the x,y coordinates
            start_x, start_y = start_point
            gripper_delta_x, gripper_delta_y = gripper_delta_point
        
            # Find the slope (dy/dx) and the midpoint
            slope = (gripper_delta_y - start_y) / (gripper_delta_x - start_x)
            midpoint = start_point + (gripper_delta_point - start_point)/2
            midpoint_x, midpoint_y = midpoint

            # Find a perpendicular line segment passing through the midpoint
            perpendicular_slope = -1/slope # perpendicular slope
            delta_x = 10 # Arbitrary
            delta_y = perpendicular_slope * delta_x
            perpendicular_endpoint = [midpoint_x + delta_x, midpoint_y + delta_y]
            perpendicular_lines.append([np.float32(midpoint), np.float32(perpendicular_endpoint)])
        
            # Annotation parameters
            circle_radius = 2
            circle_thickness = 5
            line_thickness = 2
            
            if debug:
                # Draw the start and gripper checkerboard features on the gripper image
                gripper_delta_img = cv.circle(gripper_delta_img, (int(start_x),int(start_y)), circle_radius, green_color, circle_thickness)
                gripper_delta_img = cv.circle(gripper_delta_img, (int(gripper_delta_x),int(gripper_delta_y)), circle_radius, blue_color, circle_thickness)
                # Draw white lines connecting each corresponding pair of features
                gripper_delta_img = cv.line(gripper_delta_img, np.int32(start_point).tolist(), np.int32(gripper_delta_point).tolist(), white_color, line_thickness)
                # Draw small pink lines perpendicular to those
                gripper_delta_img = cv.line(gripper_delta_img, np.int32(midpoint).tolist(), np.int32(perpendicular_endpoint).tolist(), pink_color, line_thickness)
        
                # Find where that perpendicular line segment hits a visible border of the image
                # Note this is for drawing purposes only
                height, width = gripper_delta_img.shape[:2]
                p1, p2 = get_escape_points([midpoint, perpendicular_endpoint], width, height)
        
                # Draw the longer perpendicular lines
                gripper_delta_img = cv.line(gripper_delta_img, np.int32(p1).tolist(), np.int32(p2).tolist(), pink_color, 2)
                
                # Show the in-progress result
                cv.imshow("gripper delta image", gripper_delta_img)
                cv.waitKey(10)

        if debug:
            cv.waitKey()
            cv.destroyAllWindows()

        # Finally, run an optimization process to find our rotation point!
        # This is the point that is nearest all of the lines.
        # For a starting guess, choose the intersection of the first two lines
        rotation_point = get_intersect(perpendicular_lines[0], perpendicular_lines[1])

        # Print the current cost before we iterate
        current_cost = self.gradient_descent_cost(rotation_point, perpendicular_lines)
        prior_cost = None
        print(f"gradient descent cost: {current_cost} for point: {rotation_point} - [iteration 0]")
    
        iterations = 0 # Keep track of how many optimizer loops we have run
        max_iterations = 100000 # Stop optimizing if we have done more than this many iterations
        minimum_cost_increment_threshold_per_line = 0.0001 # Stop optimizing if cost change is less than this per line
        gradient_step_size = 1 # in pixels. We're using adaptive step size so it will shrink as we get closer
    
        while True:
            x, y = rotation_point
            next_guess_x = None
            next_guess_y = None
        
            iterations += 1
        
            # Determine negative X gradient
            cost_x1 = self.gradient_descent_cost([x - gradient_step_size, y], perpendicular_lines)
            cost_x2 = self.gradient_descent_cost([x + gradient_step_size, y], perpendicular_lines)
            if cost_x1 < cost_x2:
                next_guess_x = x - gradient_step_size
            else:
                next_guess_x = x + gradient_step_size
        
            # Determine negative Y gradient
            cost_y1 = self.gradient_descent_cost([x, y - gradient_step_size], perpendicular_lines)
            cost_y2 = self.gradient_descent_cost([x, y + gradient_step_size], perpendicular_lines)
            if cost_y1 < cost_y2:
                next_guess_y = y - gradient_step_size
            else:
                next_guess_y = y + gradient_step_size
        
            # Make a new guess in the direction of the negative gradients
            rotation_point = (next_guess_x, next_guess_y)
        
            # Print the new cost
            prior_cost = current_cost
            current_cost = self.gradient_descent_cost(rotation_point, perpendicular_lines)
            print(f"gradient descent cost: {current_cost} for point: {rotation_point} - [iteration {iterations}]")

            # Check to see if we should stop the optimizer
            if iterations >= max_iterations:
                break
        
            # If this guess was worse than the last, shrink the step size and try again
            if (prior_cost - current_cost) < 0:
                gradient_step_size = gradient_step_size / 10
                continue
        
            # Also check to see if our step size is now too small
            if (prior_cost - current_cost) < minimum_cost_increment_threshold_per_line * len(perpendicular_lines):
                break

        if debug:
            # Draw a green circle at the final point
            x,y = np.int32(rotation_point)
            gripper_delta_image = cv.circle(gripper_delta_img, (x,y), 10, (0, 255, 42), 5)
    
            cv.imshow('gripper delta image', gripper_delta_img)
            cv.imwrite(output_image_path, gripper_delta_img)
            cv.waitKey()
        
        # Lastly, calculate the transform using our algorithm (see NOTES doc: 04/24/2024 - Gripper calibration method)
        c_x = x_counts_per_px_x # motor counts X per pixel X
        c_y = y_counts_per_px_y # motor counts Y per pixel Y
        rotation_point_x, rotation_point_y =  rotation_point # pixel location of rotation point
        
        camera_to_gripper_transform = [
            (rotation_point_x * c_x) + (gripper_delta_x_counts - start_x_counts),
            (rotation_point_y * c_y) + (gripper_delta_y_counts - start_y_counts)
        ]
        
        # Let other threads know the results
        self.gripper_calibration_results_ready.emit(motor_counts_per_px, camera_to_gripper_transform)
        

    @pyqtSlot()
    def exit_thread(self):
        """Emit to this slot to stop this thread."""
        # Ask the main run loop to exit
        self._thread_exiting = True

    def run(self):
        """Main thread loop"""

        # Currently the run loop does nothing other than sleep
        while (not self._thread_exiting):
            self.msleep(100) # Delay 100 ms using QThread's version of sleep

