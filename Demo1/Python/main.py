from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import math
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd






#LCD Setup
# initialize a character LCD display 
# that is connected to the Raspberry Pi's I2C interface.
lcd_columns = 16 # Specifies the size of the display
lcd_rows = 2 # Specifies the size of the display
i2c = busio.I2C(board.SCL, board.SDA) #initializes an I2C bus object for communication with the LCD.
#represents the display
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#sets the color of the backlight to blue (RGB values [0, 127, 255]).
lcd.color = [0, 127, 255]
time.sleep(2) #Pauses the execution of the code
lcd.clear() # clears LCD
#creates a color gradient
redColorIter = 0
blueColorIter = 127
greenColorIter = 255

# Camera calibration
CALIBRATION_FILE = "TRANSFORMATION_FILE_TWO.yaml"
calib_file = cv2.FileStorage(CALIBRATION_FILE, cv2.FILE_STORAGE_READ)
calib_file.release()

# Transformation
TRANSFORMATION_FILE = "TRANSFORMATION_FILE.yaml"
trans_file = cv2.FileStorage(TRANSFORMATION_FILE, cv2.FILE_STORAGE_READ)
transformation = trans_file.getNode("TRANSFORMATION_FILE_MATRIX").mat()
trans_file.release()
print(transformation)

# Constants for ArUco detection
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6x6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
FONT = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)

# Set camera parameters
camera = PiCamera()
camera.resolution = (1317, 976)
raw_capture = PiRGBArray(camera, size=(1296, 976))

## Settings for Camera
camera.exposure_mode = 'sports'
camera.image_denoise = False
camera.iso = 0
camera.brightness = 53
camera.contrast = 10

# Camera to warm up
print("Starting Camera...")
time.sleep(0.1)

for frame in camera.capture_continuous(raw_capture, format="rgb", use_video_port=True):
    # Convert image
    image = frame.array
    gray_off = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.undistort(gray_off, mtx, dist, None, newcam_mtx)
    # ArUco detection
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
    # Check if any marker was found
    if len(corners) > 0:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        (rvec-tvec).any() 
        
        # Loop through all the markers
        for index, corner_info in enumerate(corners):
            # Calculate center of marker
            c_x = int((corner_info[0][0][0] + corner_info[0][2][0]) / 2.0)
            c_y = int((corner_info[0][0][1] + corner_info[0][2][1]) / 2.0)

            ###### DRAW & CALCULATE ANGLE #####
            
            # Calculate angle and distance
            point = tvec[index][0]
            fix = np.matmul(point ,transformation)
            angle = math.degrees(math.atan(fix[0]/fix[1]))
            distance = math.sqrt(fix[0] ** 2 + fix[1] ** 2)
            
            # Draw text on the image
            phi_write = str(-1*round(angle,1)) + "'"
            distance_write = str(round(distance,1)) + "In"
            cv2.putText(gray, phi_write, (c_x+8, c_y-4), FONT,
            cv2.putText(gray, phi_write, (c_x+8, c_y-4), FONT, 1, (255,255,255), 1, cv2.LINE_AA)
            cv2.putText(gray, distance_write, (c_x+8, c_y+20), FONT, 1, (255,255,255), 1, cv2.LINE_AA)
            Degree = char(1011 0000)
            lcd.message = "Angle: " + str(Angle) + Degree


        # Draw line from center of image to center of marker
        cv2.line(gray, (int(gray.shape[1]/2), int(gray.shape[0]/2)), (c_x, c_y), (0, 255, 0), 2)

        # Print the angle and distance to the terminal
        print("Angle: {:.2f} degrees, Distance: {:.2f} inches".format(angle, distance))

        # Show the image
        cv2.imshow("ArUco 3D Detection", gray)
        
 # Wait for a key press
 key = cv2.waitKey(1) & 0xFF

# If the 'q' key was pressed, break from the loop
if key == ord("q"):
    break
    
cv2.destroyAllWindows()
camera.close()

