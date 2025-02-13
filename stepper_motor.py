"""Raspberry Pi GPIO 18 (STEP)  -> Driver STEP
Raspberry Pi GPIO 23 (DIR)   -> Driver DIR
Raspberry Pi GPIO 24 (ENABLE) -> Driver ENABLE (or GND)
Power Supply V+               -> Driver V+
Power Supply GND              -> Driver GND"""

import RPi.GPIO as GPIO
import time

# GPIO pin configuration
STEP_PIN = 18
DIR_PIN = 23
ENABLE_PIN = 24

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

# Enable the driver
GPIO.output(ENABLE_PIN, GPIO.LOW)

# Function to move the stepper motor
def move_stepper(steps, direction):
    GPIO.output(DIR_PIN, direction)  # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)  # Step
        time.sleep(0.001)  # Adjust speed here
        GPIO.output(STEP_PIN, GPIO.LOW)   # Step
        time.sleep(0.001)  # Adjust speed here

try:
    while True:
        # Move clockwise
        print("Moving clockwise")
        move_stepper(200, GPIO.HIGH)  # Move 200 steps clockwise
        time.sleep(1)

        # Move counterclockwise
        print("Moving counterclockwise")
        move_stepper(200, GPIO.LOW)  # Move 200 steps counterclockwise
        time.sleep(1)

except KeyboardInterrupt:
    pass  # Exit on Ctrl+C

# Clean up GPIO settings
GPIO.cleanup()

"""import RPi.GPIO as GPIO
import time

# GPIO pin configuration
STEP_PIN = 18
DIR_PIN = 23
ENABLE_PIN = 24

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

# Enable the driver
GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the driver (LOW)

# Function to move the stepper motor
def move_stepper(steps, direction):
    GPIO.output(DIR_PIN, direction)  # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)  # Step
        time.sleep(0.001)  # Adjust speed here
        GPIO.output(STEP_PIN, GPIO.LOW)   # Step
        time.sleep(0.001)  # Adjust speed here

try:
    while True:
        # Move clockwise
        print("Moving clockwise")
        move_stepper(200, GPIO.HIGH)  # Move 200 steps clockwise
        time.sleep(1)

        # Move counterclockwise
        print("Moving counterclockwise")
        move_stepper(200, GPIO.LOW)  # Move 200 steps counterclockwise
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by User")

finally:
    # Clean up GPIO settings
    GPIO.cleanup()"""
