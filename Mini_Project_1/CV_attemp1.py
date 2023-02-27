'''
John Puryear
2/21/23
Seed Lab: Mini project

#Attemp 1, needs to be fixed
'''

import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import busio
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus import SMBus


# CONSTANTS
CALIBRATION_FILE = "camera_calibration.yaml"
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
CAMERA_RESOLUTION = (1296, 976)
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_BOLD = cv2.FONT_HERSHEY_DUPLEX
CENTER_X = CAMERA_RESOLUTION[0] / 2
CENTER_Y = CAMERA_RESOLUTION[1] / 2
LCD_COLUMNS = 16
LCD_ROWS = 2


def calculate_corner_center(corners, index):
    """
    Given an array of ArUco corners and an index, calculate the center of the corner at the given index.
    :param corners: Array of ArUco corners.
    :param index: Index of the corner to calculate the center of.
    :return: Tuple of x and y coordinates of the center of the corner.
    """
    corner_info = corners[index]
    c_x = int((corner_info[0][0][0] + corner_info[0][2][0]) / 2.0)
    c_y = int((corner_info[0][0][1] + corner_info[0][2][1]) / 2.0)
    return c_x, c_y


# LOAD CALIBRATION FILE
cv_file_load = cv2.FileStorage(CALIBRATION_FILE, cv2.FILE_STORAGE_READ)
mtx_load = cv_file_load.getNode("camera_matrix").mat()
dist_load = cv_file_load.getNode("dist_coeff").mat()
newcameramtx_load = cv_file_load.getNode("new_camera_matrix").mat()
cv_file_load.release()

# SET UP CAMERA
camera = PiCamera()
camera.resolution = CAMERA_RESOLUTION
raw_capture = PiRGBArray(camera, size=CAMERA_RESOLUTION)
time.sleep(0.1)

# LCD SETUP
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, LCD_COLUMNS, LCD_ROWS)
lcd.color = [0, 127, 255]
smile = chr(0b10111100)
lcd.message = f" {smile}    ArUco   {smile} \n     Testing     "
time.sleep(2)
lcd.clear()
red_color_iter = 0
blue_color_iter = 127
green_color_iter = 255

# I2C SETUP
bus = SMBus(1)
addr = 0x8
return_data = 0

# MAIN LOOP
quadrant = 0
quadrant_new = 0
for frame in camera.capture_continuous(raw_capture, format="rgb", use_video_port=True):
    try:
        # GET IMAGE AND CONVERT
        image = frame.array
        gray_off = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.undistort(gray_off, mtx_load, dist_load, None, newcameramtx_load)

        # RUN ARUCO DETECTION
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        # DRAW DETECTED MARKERS
                
        gray = cv2.aruco.drawDetectedMarkers(gray, corners, ids)

        # DRAW CENTER POINTS AND IDS
        if ids is not None:
            for i in range(len(ids)):
                center_x, center_y = calculate_corner_center(corners, i)
                cv2.circle(gray, (center_x, center_y), 2, (255, 0, 0), -1)
                cv2.putText(gray, str(ids[i][0]), (center_x + 10, center_y + 10), FONT, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

                # UPDATE QUADRANT BASED ON CENTER POINT
                if center_x < CENTER_X and center_y < CENTER_Y:
                    quadrant_new = 0
                elif center_x >= CENTER_X and center_y < CENTER_Y:
                    quadrant_new = 1
                elif center_x >= CENTER_X and center_y >= CENTER_Y:
                    quadrant_new = 2
                else:
                    quadrant_new = 3

        # IF A NEW QUADRANT HAS BEEN DETECTED, UPDATE LCD AND SEND DATA TO I2C
        if quadrant_new != quadrant:
            quadrant = quadrant_new
            if quadrant == 0:
                lcd.color = [0, 127, 255]  # Blue
            elif quadrant == 1:
                lcd.color = [255, 255, 0]  # Yellow
            elif quadrant == 2:
                lcd.color = [255, 0, 0]  # Red
            else:
                lcd.color = [0, 255, 0]  # Green

            lcd.clear()
            lcd.message = f"  Quadrant: {quadrant}  "

            # SEND DATA TO I2C
            try:
                bus.write_byte(addr, quadrant)
                return_data = bus.read_byte(addr)
            except:
                print("Error communicating with I2C device")

        # DISPLAY IMAGE
        cv2.imshow("ArUco Detection", gray)
        key = cv2.waitKey(1) & 0xFF

        # CLEAR THE STREAM
        raw_capture.truncate(0)

        # BREAK LOOP IF 'q' KEY IS PRESSED
        if key == ord("q"):
            break

    except KeyboardInterrupt:
        break

# CLEAN UP
cv2.destroyAllWindows()
camera.close()

