# Initial code to control stepper motor
# ** Generated with the help of AI **

import RPi.GPIO as GPIO
import time

# Define GPIO pins connected to ULN2003
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22
step_pins = [IN1, IN2, IN3, IN4]

# Step sequence (Half-Step Mode)
step_sequence = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
]

# GPIO setup
GPIO.setmode(GPIO.BCM)
for pin in step_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

def stepper_motor(steps, delay=0.005):
    for _ in range(steps):
        for step in step_sequence:
            for pin in range(4):
                GPIO.output(step_pins[pin], step[pin])
            time.sleep(delay)

try:
    stepper_motor(512)  # Rotate stepper motor
finally:
    GPIO.cleanup()