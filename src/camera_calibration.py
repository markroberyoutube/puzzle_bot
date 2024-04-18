import sys, os, logging, tempfile, math, glob
import cv2 as cv
import numpy as np
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThread

from utils import estimate_linear_regression_coefficients

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

class CameraCalibration(QThread):
    """"Camera Calibration

    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    """
    
    # Define QT Signals for inter-process communication
    trigger_checkerboard_error_measurements = pyqtSignal(str, int, int, float, float)
    checkerboard_error_measurements_ready = pyqtSignal(float, float, float, float, str)
    
    trigger_camera_calibration = pyqtSignal(str, int, int, float, float, list, list)
    camera_calibration_results_ready = pyqtSignal(float, float, list, list)
    
    trigger_undistort_image = pyqtSignal(str, str, list, list)
    undistorted_image_ready = pyqtSignal()
    
    thread_closing = pyqtSignal() # Emit to this when you want the thread to exit
    
    def __init__(self, parent):
        # Superclass initializer
        QThread.__init__(self, parent)
        
        # Connect signals for inter-process communication
        self.trigger_checkerboard_error_measurements.connect(self.measure_checkerboard_error)
        self.trigger_camera_calibration.connect(self.calibrate_camera)
        self.trigger_undistort_image.connect(self.undistort_image)
        self.thread_closing.connect(self.exit_thread)
        
        # Set to True to cause the Thread's main run loop to exit
        self._thread_exiting = False
        
        # Create a tempdir 
        self.temp_dir = tempfile.mkdtemp()

    @pyqtSlot(str, str, list, list)
    def undistort_image(self, source_image_path, destination_image_path, camera_matrix, distortion_coefficients):
        # Make sure source path exists
        if not source_image_path or not os.path.exists(source_image_path):
            logging.error(f"[CameraCalibration.undistort_image] Source image does not exist at: {source_image_path}")
            return
        
        # Open the image
        img = cv.imread(source_image_path)
        
        # Convert intrinsics to np arrays
        camera_matrix = np.array(camera_matrix)
        distortion_coefficients = np.array(distortion_coefficients)
        
        logging.debug(f"[CameraCalibration.undistort_image] camera_matrix: {camera_matrix}")
        logging.debug(f"[CameraCalibration.undistort_image] distortion_coefficients: {distortion_coefficients}")
                
        # Optionally refine the camera matrix using a free scaling parameter. Causes "zoom out"
        # and the goal is to either hide or show the black area around the image created after we undistort the image
        # details: https://stackoverflow.com/questions/39432322/what-does-the-getoptimalnewcameramatrix-do-in-opencv
        # h, w = img.shape[:2]
        # refined_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w,h), 1, (w,h))
        #dst = cv.undistort(img, camera_matrix, distortion_coefficients, None, refined_matrix)
        
        # Undistort the image
        dst = cv.undistort(img, camera_matrix, distortion_coefficients)
        
        # Save the image
        cv.imwrite(destination_image_path, dst)
        
        # Tell other threads the undistorted image is ready
        self.undistorted_image_ready.emit()

    @pyqtSlot(str, int, int, float, float, list, list)
    def calibrate_camera(self, photo_directory, checker_rows, checker_cols, checker_width_mm, checker_height_mm, camera_matrix, distortion_coefficients):
        # Determine features (intersections) from checkerboard parameters
        features_x = checker_cols - 1
        features_y = checker_rows - 1
        
        # Convert lists into np arrays
        camera_matrix = np.array(camera_matrix)
        distortion_coefficients = np.array(distortion_coefficients) #k1,k2,p1,p2[,k3[k4,k5,k6[,s1,s2,s3,s4[,tx,ty]]]]
        
        # Make sure photo directory exists
        if not photo_directory or not os.path.exists(photo_directory) or not os.path.isdir(photo_directory):
            logging.error(f"[CameraCalibration.calibrate_camera] Calibration photo directory does not exist at: {photo_directory}")
            return
        
        # Make sure there are jpg files in that photo directory
        photos = glob.glob(os.path.join(photo_directory, "*.jpg"))
        if not photos or len(photos) == 0:
            logging.error(f"[CameraCalibration.calibrate_camera] No jpg files found in Calibration photo directory at: {photo_directory}")
            return
        
        # Arrays to store object points and image points from all the images.
        three_d_points = [] # Vector for 3D points (known from checkerboard dimensions)
        two_d_points = [] # Vector for 2D points (observed in the images)
        

        # Prepare (x,y,z) feature locations in mm, like (0,0,0), (5,0,0), (10,0,0) ....,(30,25,0)
        # 3D points of all features in checkerboard coordinate system, in the real world.
        # This presumes a perfectly flat checkerboard, with spacing given as function arguments.
        # This also presumes a square checker (checker_width_mm == checker_height_mm)
        objectp3d = np.zeros((1, features_x * features_y, 3), np.float32) 
        objectp3d[0, :, :2] = np.mgrid[0:features_x, 
                                       0:features_y].T.reshape(-1, 2)*checker_height_mm
        
        # Iterate through calibration photos and find features (corners, aka saddle points)
        checkerboards_found = 0
        img = None
        gray = None
        for image_path in photos:
            # Open the image and convert it to grayscale
            img = cv.imread(image_path)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
            # Find the chess board corners based on a sector-based approach defined in:
            # http://bmvc2018.org/contents/papers/0508.pdf
            retcode, corners = cv.findChessboardCornersSB(gray,(features_x, features_y),
                cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY)
            
            # Skip any images that we couldn't find all the features on
            if not retcode or corners is None or len(corners) < features_x*features_y:
                continue
            
            # Add object points and image points
            if retcode == True:
                checkerboards_found += 1
                three_d_points.append(objectp3d)            
                two_d_points.append(corners)
        
        logging.debug(f"[CameraCalibration.calibrate_camera] Found checkerboards: {checkerboards_found}")
        
        # Prepare parameters for the camera calibration routine
        img_height, img_width, img_depth = img.shape
        img_size = (img_width, img_height)
        flags = cv.CALIB_SAME_FOCAL_LENGTH + cv.CALIB_USE_INTRINSIC_GUESS
        
        # Configure termination criteria
        max_iterations = 1000
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, max_iterations, 0.00001)
        
        # Provide initial guesses if not given for f_x, f_y, c_x, and c_y
        # [[fx 0  cx]
        #  [0  fy cy]
        #  [0  0  1]]
        if camera_matrix[0][0] == 0: camera_matrix[0][0] = 10000 # f_x
        if camera_matrix[1][1] == 0: camera_matrix[1][1] = 10000 # f_y
        if camera_matrix[0][2] == 0: camera_matrix[0][2] = img_width/2 # c_x
        if camera_matrix[0][2] == 0: camera_matrix[1][2] = img_height/2 # c_y
        
        # Run camera calibration routine
        # fixed_point = 1
        # rms_reprojection_error, camera_matrix, distortion_coefficients, r_vecs, t_vecs, new_three_d_points = cv.calibrateCameraRO(
        #     three_d_points, two_d_points, img_size, fixed_point, camera_matrix, distortion_coefficients, None, None, None, flags, criteria
        # )
        # three_d_points = new_three_d_points
        
        
        
        # Run camera calibration routine
        rms_reprojection_error, camera_matrix, distortion_coefficients, r_vecs, t_vecs = cv.calibrateCamera(
            three_d_points, two_d_points, img_size, camera_matrix, distortion_coefficients, None, None, flags, criteria
        )
        
        logging.debug(f"[CameraCalibration.calibrate_camera] RMS RPE: {rms_reprojection_error}")
        logging.debug(f"[CameraCalibration.calibrate_camera] Camera matrix: {camera_matrix}")
        logging.debug(f"[CameraCalibration.calibrate_camera] Distortion coefficients: {distortion_coefficients}")
        
        # Compute error
        total_error = 0
        total_points = 0

        largest_error = 0
        for i in range(len(three_d_points)):
            reprojected_points, _ = cv.projectPoints(three_d_points[i], r_vecs[i], t_vecs[i], camera_matrix, distortion_coefficients)
            error = cv.norm(two_d_points[i], reprojected_points, cv.NORM_L2)
            if error > largest_error:
                largest_error = error
            total_error += error*error
            total_points += three_d_points[i].shape[1] # the number of checkerboard corners
        calculated_rms_rpe = math.sqrt(total_error / total_points)
        logging.debug(f"[CameraCalibration.calibrate_camera] RMS Error (re-calculated): {calculated_rms_rpe}")
        logging.debug(f"[CameraCalibration.calibrate_camera] Largest Error: {largest_error}")
        
        # Let the other threads know that camera calibration results are ready
        self.camera_calibration_results_ready.emit(
            calculated_rms_rpe, largest_error, camera_matrix.tolist(), distortion_coefficients.tolist()
        )
        
        
    @pyqtSlot(str, int, int, float, float)
    def measure_checkerboard_error(self, image_path, checker_rows, checker_cols, checker_width_mm, checker_height_mm):
        """Given a single photo, fit lines to the rows of features and measure the error of the fit"""
        # Determine features (intersections) from checkerboard parameters
        features_x = checker_cols - 1
        features_y = checker_rows - 1

        # Configure drawing parameters
        radius = 50
        line_color = (255, 255, 255) # BGR, so this is white
        circle_color = (243, 10, 213) # BGR, so this is pink
        thickness = 10
    
        # Make sure image path exists
        if not image_path or not os.path.exists(image_path):
            logging.error(f"[CameraCalibration.measure_checkerboard_error] Image file does not exist at: {image_path}")
            return
        
        # Find the chess board corners (saddle points) based on a sector-based approach defined in:
        # http://bmvc2018.org/contents/papers/0508.pdf
        img = cv.imread(image_path)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        retcode, corners = cv.findChessboardCornersSB(
            gray,
            (features_x, features_y), 
            cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY
        )
        if not retcode or corners is None or len(corners) < features_x*features_y:
            logging.error(f"[CameraCalibration.measure_checkerboard_error] Could not find all features in image at {image_path}")
            return
        features = [list(c[0]) for c in corners]
        
        # Draw circles around each feature
        for (x,y) in features:
            img = cv.circle(img, (int(x),int(y)), radius, circle_color, thickness)
        
        # Fit lines to each of the rows
        rows = zip(*(iter(features),) * features_x) # Split up the feature list into rows of features
        img_height, img_width, img_depth = img.shape
        max_error_px = 0
        error_squared_total_px = 0
        num_error_terms = 0
        for row in rows:
            # Run linear regression
            (a, b) = estimate_linear_regression_coefficients(row)
            logging.debug(f"[CameraCalibration.measure_checkerboard_error] Linear regression a={a}, b={b}")
            # Draw the lines
            start_point = (0, int(a)) # y intercept at x = 0 is y=a+bx => y=a
            end_point = (img_width, int(a+(b*img_width))) # end of line should be at edge of image, x = image width
            img = cv.line(img, start_point, end_point, line_color, thickness)
            # Compute error terms
            for point in row:
                x, y = point
                estimated_y = a + b*x
                abs_error_px = abs(y - estimated_y)
                error_squared_total_px += abs_error_px**2
                num_error_terms += 1
                if abs_error_px > max_error_px:
                    max_error_px = abs_error_px

        # Finally compute the Root Mean Square Error
        average_rms_error_px = math.sqrt(error_squared_total_px / num_error_terms)
        
        # Find the smallest spacing (in pixels) between features, useful to convert px to inches
        rows = zip(*(iter(features),) * features_x) # Split up the feature list (again!) into rows of features
        min_feature_spacing_x = None
        for row in rows:
            prev_x, prev_y = row[0]
            for point in row[1:]:
                cur_x, cur_y = point
                spacing_x = abs(cur_x - prev_x)
                if min_feature_spacing_x is None or spacing_x < min_feature_spacing_x:
                    min_feature_spacing_x = spacing_x
                prev_x = cur_x
                prev_y = cur_y
        
        # Convert the errors into mm and then inches
        px_per_mm = min_feature_spacing_x / checker_width_mm
        px_per_inch = px_per_mm * 25.4
        average_rms_error_inches = average_rms_error_px / px_per_inch
        max_error_inches = max_error_px / px_per_inch
        
        # Save the annotated image to our temp dir
        annotated_image_path = os.path.join(self.temp_dir, os.path.split(image_path)[-1])
        cv.imwrite(annotated_image_path, img)
        
        # Let other threads know that the results are ready
        self.checkerboard_error_measurements_ready.emit(
            average_rms_error_px, 
            max_error_px, 
            average_rms_error_inches,
            max_error_inches,
            annotated_image_path
        )

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

