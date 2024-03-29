from fileinput import filename
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
from numpy.linalg import inv

# Constants
CALIBRATION_FILE = "CALIBRATION_FILE_TWO.yaml"
TRANSFORMATION_MATRIX_FILE = "TRANSFORMATION_FILE_ONE.yaml"
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6x6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
FONT = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
BOLD_FONT = cv2.FONT_HERSHEY_DUPLEX  # font for displaying text (below)
MARKER_IDS = [0, 1, 2]
ITERATIONS = 10

# Initialize and warmup the camera
camera = PiCamera()
raw_capture = PiRGBArray(camera)
camera.resolution = (1296, 976)
time.sleep(0.1)

# Open the calibration file
cv_file_load = cv2.FileStorage(CALIBRATION_FILE, cv2.FILE_STORAGE_READ)
cv_file_load.release()

# Numpy matrix to hold temp point data
ball = np.zeros((3, 3))

# Iteration variable
for i in range(Iterations):
    # Take a photo
    print(f"Info: Capturing image for camera calibration, {i}")
    try:
        raw_capture = PiRGBArray(camera)
        camera.capture(raw_capture, format="bgr")
        image = raw_capture.array
        print("Info: Image Captured")

        # Save the image
        print(f"Info: Saving image {FILENAME}")
        cv2.imwrite(FILENAME, image)
        print("Info: Image Saved")

        # Modify the captured image
        gray_off = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.undistort(gray_off, mtx_load, dist_load, None, newcameramtx_load)

        # Perform Aruco detection
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        # Check detection for needed markers
        if all(marker in ids for marker in MARKER_IDS):
            print("Info: Captured Image contains needed markers")

            # Calculate the 3D position of the markers
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_load, dist_load)
            (rvec - tvec).any()

            # Calculate index
            index_a, index_b, index_c = [np.where(ids == marker)[0][0] for marker in MARKER_IDS]

            # Get the positions
            pos_a, pos_b, pos_c = [tvec[index][0] for index in (index_a, index_b, index_c)]
            print(f"Appending: {[pos_a, pos_b, pos_c]}")
            ball += np.array([pos_a, pos_b, pos_c])
            print(ball)

        # The captured image did not contain the required ArUco markers
        else:
            print(f"Error: Captured image did not contain the necessary ArUco markers for calibration, {i}")

    except:
        print(f"Error: FAILED #{i}")

    time.sleep(0.5)

