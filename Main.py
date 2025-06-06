'''
Wiring for the BNO055 IMU: 
Vin -> 3V3 Power (pin 1)
GND -> Ground (pin 9)
SDA -> GPIO 2 (pin 3)
SCL -> GPIO 3 (pin 5)

Wiring for the linear actuator:
--- L298N Motor Driver ---
IN1 -> GPIO 12 (pin 32)
IN2 -> GPIO 17 (pin 38)
5V power to the motor driver
GND -> Ground (pin 9)
12V battery power to the motor driver
12V battery ground to the motor driver

Wiring for the foot sensors

Wiring for the stepper motor:
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
DIR -> GPIO 20, pin 38

Main function for overall leg control
Takes in data from the three IMU sensors and the two foot sensors and uses that data to determine the optimal position of the leg and the foot
Users will have the option of saving data into a .json file and a .csv if desired

Ensure all pin connections are correct and the Raspbeerry Pi is powered on before testing
'''
# INITIALIZATION AND SETUP
# ----------------------------------------------------------------------------------------------
# Import the necessary libraries
import time
import board
import busio
import adafruit_bno055
import json
import RPi.GPIO as GPIO
import time
import math
import csv
import pandas as pd
#import spidev
i2c = busio.I2C(board.SCL, board.SDA)
sensor1 = adafruit_bno055.BNO055_I2C(i2c, address = 0x28)
sensor2 = adafruit_bno055.BNO055_I2C(i2c, address = 0x29)

# Setup GPIO pins for the stepper motor
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

last_val = 0xFFFF

spi0 = spidev.SpiDev()
spi0.open(0,0)
spi0.max_speed_hz = 1350000

spi1 = spidev.SpiDev()
spi1.open(0,1)
spi1.max_speed_hz = 1350000

spi2 = spidev.SpiDev()
spi2.open(0,2)
spi2.max_speed_hz = 1350000

spi3 = spidev.SpiDev()
spi3.open(0,3)
spi3.max_speed_hz = 1350000



imu_set = [sensor1, sensor2]
foot_data1 = []

imu_data = {
        "timestamp": [[] for x in imu_set],
        "Accelerometer": [[] for x in imu_set],
        "Magnetometer": [[] for x in imu_set],
        "Gyroscope": [[] for x in imu_set],
        "Euler": [[] for x in imu_set],
        "Quaternion": [[] for x in imu_set],
        "Linear Acceleration": [[] for x in imu_set],
        "Gravity": [[] for x in imu_set]
    }

foot_data1 = []
foot_data2 = []
prev_foot_avg1 = 0
current_angle = 0
last_foot_change_time = time.time()
gait_phase = 'STANCE'

accel_dif = 0.0
previous_accel = 0.0
previous_quat = (0.0, 0.0, 0.0, 0.0)
# ----------------------------------------------------------------------------------------------

# SENSOR OUTPUT FUNCTIONS
# ----------------------------------------------------------------------------------------------
def temperature():
    global last_val  
    result = sensor1.temperature
    if abs(result - last_val) == 128:
        result = sensor1.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

def sensorData (sensor):
    ts = int(round(time.time() * 1000)) #time
    return{
        "ts": int(round(time.time() * 1000)),
        "t": sensor.temperature, #temp
        "d": {
            "x": sensor.euler[0],
            "y": sensor.euler[1],
            "z": sensor.euler[2],
        }
    }

def read_channel(spi_bus, channel):
# if channel < 0 or channel > 7:
#     return -1
    adc = spi_bus.xfer2([1, (8+channel)<<4, 0])
    foot_data1 = ((adc[1]&3) << 8) + adc[2]
    return foot_data1

# ----------------------------------------------------------------------------------------------

# HARDWARE CONTROL FUNCTIONS
# ----------------------------------------------------------------------------------------------
def move_actuator(move_distance, direction):
    IN1 = 12
    IN2 = 17
    # distance to move equals: num_seconds * 0.39
    sleep_time = move_distance / 0.39
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    if direction == 'extend':
        print('Moving forward')
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif direction == 'retract':
        print('Moving backward')
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    elif direction == 'stop':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
    time.sleep(sleep_time)
    
def rotate_motor(angle):
    #ex: 200 steps = 360 degrees
    # each step would be 1.8 degrees
    # steps * 1.8 = angle
    steps = angle / 1.8 
    #Rotate the motor a specified number of steps.
    if steps > 0:
        GPIO.output(DIR, 1)  # CW rotation
    else:
        GPIO.output(DIR, 0)  # CCW rotation

    steps = abs(steps)  # Use absolute value for the loop
    for x in range(steps):
        GPIO.output(STEP, GPIO.HIGH)
        time.sleep(0.01)  # Adjust delay for speed
        GPIO.output(STEP, GPIO.LOW)
        time.sleep(0.01)
# ----------------------------------------------------------------------------------------------

# DATA STORAGE
# ----------------------------------------------------------------------------------------------
def save_data(sensors):
    # Save values for the .json file
    for idx, sensor in enumerate(sensors):
        imu_data["timestamp"][idx].append(int(round(time.time() * 1000)))
        imu_data["Accelerometer"][idx].append(sensor.acceleration)
        imu_data["Magnetometer"][idx].append(sensor.magnetic)
        imu_data["Gyroscope"][idx].append(sensor.gyro)
        imu_data["Euler"][idx].append(sensor.euler)
        imu_data["Quaternion"][idx].append(sensor.quaternion)
        imu_data["Linear Acceleration"][idx].append(sensor.linear_acceleration)
        imu_data["Gravity"][idx].append(sensor.gravity)
    # Save to a csv file for the first IMU
    with open("IMU_data.csv", "w", newline='') as csvfile:
        fieldnames = ["timestamp", "Accelerometer", "Magnetometer", "Gyroscope", "Euler", "Quaternion", "Linear Acceleration", "Gravity"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(len(imu_data["timestamp"])):
            writer.writerow({
                "timestamp": imu_data["timestamp"][i],
                "Accelerometer": imu_data["Accelerometer"][i],
                "Magnetometer": imu_data["Magnetometer"][i],
                "Gyroscope": imu_data["Gyroscope"][i],
                "Euler": imu_data["Euler"][i],
                "Quaternion": imu_data["Quaternion"][i],
                "Linear Acceleration": imu_data["Linear Acceleration"][i],
                "Gravity": imu_data["Gravity"][i]
            })
# ----------------------------------------------------------------------------------------------

# MAIN LOOP
# ----------------------------------------------------------------------------------------------
try:
    while True:
        current_time = time.time()
        '''print("Temperature: {} degrees C".format(sensor.temperature))'''
    
        print(
        #   "Temperature: {} degrees C".format(temperature()) # Use to monitor sensor health
        ) 
        #print("Accelerometer (m/s^2): {}".format(sensor.acceleration)) # Right decreases, left increases, Up Increases, Down Decreases, Forward increases, Backward decreases
        #print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        #print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        print("Euler angle: {}".format(sensor1.euler)) # Supposed to change as you rotate the sensor
        #print("Quaternion: {}".format(sensor1.quaternion))
        q_w, q_x, q_y, q_z = sensor1.quaternion
        #print(f"sensor.quaternion: w={q_w}, x={q_x}, y={q_y}, z={q_z}")
        print()
        print("Euler angle2: {}".format(sensor2.euler))
        #print("Quaternion2: {}".format(sensor2.quaternion))
        q_w2, q_x2, q_y2, q_z2 = sensor2.quaternion
        #print(f"sensor.quaternion: w={q_w2}, x={q_x2}, y={q_y2}, z={q_z2}")
        #print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration)) # Left right up and down movement
        #print("Gravity (m/s^2): {}".format(sensor.gravity)) # About 9.81 whatever axis is pointing down
        print()

        #message construction for positional data
        message = json.dumps ({
            "sensor": sensorData(sensor1)
        })
        #print(message)
        print('---------------------------------------------')
        save_data(imu_set)
        
        print('First MCP3008 Reading:')
        for channel in range(8):
            print(f"Channel {channel}: {read_channel(spi0, channel)}", end="\t")
            foot_data1.append(read_channel(spi0, channel))
            print('\n')

        print('Second MCP3008 Reading:')
        for channel in range(8):
            print(f"Channel {channel+8}: {read_channel(spi1, channel)}", end="\t")
            foot_data1.append(read_channel(spi1, channel))
            print('\n\n\n\n\n\n')
        
        print('Third MCP3008 Reading:')
        for channel in range(8):
            print(f"Channel {channel}: {read_channel(spi2, channel)}", end="\t")
            foot_data2.append(read_channel(spi2, channel))
            print('\n')

        print('Fourth MCP3008 Reading:')
        for channel in range(8):
            print(f"Channel {channel+8}: {read_channel(spi3, channel)}", end="\t")
            foot_data2.append(read_channel(spi3, channel))
            print('\n\n\n\n\n\n')
        print('-----------------------------------')
        time.sleep(1)

        # Decide when the actuator should extend or retract
        foot_avg1 = sum(foot_data1) / len(foot_data1)
        foot_avg2 = sum(foot_data2) / len(foot_data2)
        foot_data1 = [] # Reset foot data for the next iteration
        foot_data2 = [] # Reset foot data for the next iteration
        foot_avg1_change = foot_avg1 - prev_foot_avg1
        prev_foot_avg1 = foot_avg1
        if foot_avg1_change > 100:
            last_foot_change_time = time.time()
            change_duration = current_time-last_foot_change_time
            if change_duration > 1:
                gait_phase = 'SWING'
        elif foot_avg1_change < 100:
            last_foot_change_time = time.time()
            change_duration = current_time-last_foot_change_time
            if change_duration > 1:
                gait_phase = 'STANCE'
             
        hip_accel_diff = sensor1.acceleration[1] - sensor2.acceleration[1]
        if (foot_avg1 < 100) and (hip_accel_diff > 30) and gait_phase == 'SWING':
            move_actuator(0.5, 'extend')
        elif (foot_avg1 < 100) and (hip_accel_diff < -30) and gait_phase == 'SWING':
            move_actuator(0.5, 'retract')
        else:
            move_actuator('stop')
        # Decide when the stepper motor should rotate
        if (foot_avg1 < 100) and (gait_phase == 'SWING') and (current_angle < 48) and (current_angle > -48) and (foot_avg1 > foot_avg2): # find the optimal value for this statement
            rotate_motor(8) # Move the motor to the right
            current_angle += 8
        elif (foot_avg1 < 100) and (gait_phase == 'SWING') and (current_angle < 48) and (current_angle > -48) and (foot_avg1 < foot_avg2):
            rotate_motor(-8) # Move the motor to the left. Negative val might not work
            current_angle -= 8
        else:
            rotate_motor(0)

        # Find the change in the euler angles to determine extension or retraction needs
        quat_dot = (q_w * previous_quat[0] + q_x * previous_quat[1] + q_y * previous_quat[2] + q_z * previous_quat[3])
        quat_dot = max(min(quat_dot, 1.0), -1.0)
        quat_diff = math.degrees(2*math.acos(quat_dot))
        previous_quat = (q_w, q_x, q_y, q_z)
        print('Quat diff is: ', quat_diff)

        
except KeyboardInterrupt:
    print('Data collection stopped')
    with open("imu_data.json", "w") as file:
        json.dump(imu_data, file, indent=1)
    print('Saved the data in imu_data.json')
    # Disable the driver and cleanup GPIO settings
    GPIO.output(ENABLE, GPIO.HIGH)  # Set HIGH to disable the driver
    GPIO.cleanup()
    spi0.close()
    spi1.close()
    spi2.close()
    spi3.close()
# ----------------------------------------------------------------------------------------------

# Sources
# ----------------------------------------------------------------------------------------------
#https://github.com/land-boards/lb-boards/blob/85bca65a309e9e72ca20b445a434affc8d03e97d/Projects/QTPy/adafruit-circuitpython-bundle-7.x-mpy-20220601/examples/bno055_i2c_gpio_simpletest.py
# ----------------------------------------------------------------------------------------------