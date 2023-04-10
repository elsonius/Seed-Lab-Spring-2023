import time
import math
import busio
import board
import smbus
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
from board import D18 as TRIG_PIN, D24 as ECHO_PIN
import RPi.GPIO as GPIO
import character_lcd

# Set up LCD screen
lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [0, 0, 100]
time.sleep(1)
bus = smbus.SMBus(1)
address = 4
size = 2

# Write I2C value
def writeNumber(value):
    bus.write_i2c_block_data(address, 0, value)
    return -1

# Read I2C value
def readNumber():
    number = bus.read_i2c_block_data(address, 0, size)
    return number

def createMarkers():
    # Run this program then print the markers that are saved in this program's directory
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    Mk1 = aruco.drawMarker(aruco_dict, 1, 700)
    Mk2 = aruco.drawMarker(aruco_dict, 2, 700)
    cv2.imwrite('marker1.jpg', Mk1)
    cv2.imwrite('marker2.jpg', Mk2)
    cv2.imshow('Marker 1', Mk1)
    cv2.imshow('Marker 2', Mk2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    # Set GPIO mode to BCM
    GPIO.setmode(GPIO.BCM)
    # Set up GPIO pins
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    # Set trigger pin low
    GPIO.output(TRIG_PIN, False)
    time.sleep(2)

    try:
        while True:
            # Send a 10us pulse to trigger
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TRIG_PIN, False)

            # Wait for echo to go high and then low again
            while GPIO.input(ECHO_PIN) == 0:
                pulse_start = time.time()
            while GPIO.input(ECHO_PIN) == 1:
                pulse_end = time.time()

            # Calculate distance in cm
            pulse_duration = pulse_end - pulse_start
            distance = round(pulse_duration * 17150, 2)

            # Calculate angle in degrees
            angle = round(math.degrees(2 * math.asin(distance / (2 * 25))), 2)

            # Print distance and angle values to LCD
            lcd.clear()
            lcd.message('Distance: {} cm\nAngle: {} deg'.format(distance, angle))
            time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()

state = 0  # state 0 is start
camera = PiCamera()
focalLen = 1745.95  # pixels (calculated)
markerHeight = 185  # mm (measured) marker size must be 700. use create marker function
print("Calibrating Camera...")
camera.resolution = (1920, 1088)  # resolution of computer monitor
time.sleep(2)
camera.exposure

