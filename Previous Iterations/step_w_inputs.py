'''Wiring based on DRV8825:
GND across from DIR -> ground pin on pi, eg pin 9
2A -> blue wire from motor
1A -> black wire from motor
1B -> green wire from motor
2B -> red wire from motor
GND -> ground (black wire) from the battery
VMOT -> positive (red wire) from battery
EN -> GPIO 22, pin 15
RST and SLP -. 5V pin on pi, eg pin 4
STP -> GPIO 21, pin 40
DIR -> GPIO 20, pin 38'''

import RPi.GPIO as GPIO
from time import sleep

# Define GPIO pins
DIR = 20  # Direction GPIO Pin
STEP = 21  # Step GPIO Pin
ENABLE = 22  # Enable GPIO Pin

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(ENABLE, GPIO.OUT)

# Enable the driver
GPIO.output(ENABLE, GPIO.LOW)  # Set LOW to enable the driver

def rotate_motor(steps):
    #Rotate the motor a specified number of steps.
    if steps > 0:
        GPIO.output(DIR, 1)  # CW
    else:
        GPIO.output(DIR, 0)  # CCW

    steps = abs(steps)  # Use absolute value for the loop
    for _ in range(steps):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(0.01)  # Adjust delay for speed
        GPIO.output(STEP, GPIO.LOW)
        sleep(0.01)

try:
	while True:
    # Get user input
		user_input = input("Enter the number of steps (positive for CW, negative for CCW) or 'exit' to stop: ")
    
		if user_input.lower() == 'exit':
			break #exit the loop
		
		try:
			steps = int(user_input) #steps will be ints only
			rotate_motor(steps)
		except ValueError:
			print("Invalid number. Please enter an integer or 'exit'.")

finally:
    # Disable the driver and cleanup GPIO settings
    GPIO.output(ENABLE, GPIO.HIGH)  # Set HIGH to disable the driver
    GPIO.cleanup()
