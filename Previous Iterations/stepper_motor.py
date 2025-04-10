'''Wiring based on DRV8825:
GND across from DIR -> ground pin on pi, eg pin 9
"2A -> blue wire from motor
1A -> black wire from motor
1B -> green wire from motor
2B -> red wire from motor
GND -> ground (black wire) from the battery
VMOT -> positive (red wire) from battery
EN -> GPIO 22, pin 15
RST and SLP -. 5V pin on pi, eg pin 4
STP -> GPIO 21, pin 40
DIR -> GPIO 20, pin 40'''

import time
import RPi.GPIO as GPIO

DIR = 20 
STEP = 21 
ENABLE = 22
CW = 1
CCW = 0
SPR = 200 #number of steps

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(ENABLE, GPIO.OUT)

GPIO.output(ENABLE, GPIO.LOW)
GPIO.output(DIR, CW)

#completes 1 revolution cw
for x in range(SPR):
    GPIO.output(STEP, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(STEP, GPIO.LOW)
    time.sleep(0.01)

time.sleep(0.5)

GPIO.output(DIR, CCW)
#completes 1 revolution ccw
for x in range(SPR):
    GPIO.output(STEP, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(STEP, GPIO.LOW)
    time.sleep(0.01)

GPIO.output(ENABLE, GPIO.HIGH) #disable driver
GPIO.cleanup() #cleanup GPIO settings