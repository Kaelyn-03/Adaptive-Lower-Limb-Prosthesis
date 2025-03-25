''' IMU -> Rasp Pi
Vin -> 3V3 Power (pin 1)
GND -> Ground (pin 9)
SDA -> GPIO 2 (pin 3)
SCL -> GPIO 3 (pin 5)'''

#I2C initialization
import time
import board
import busio
import adafruit_bno055
import json
import RPi.GPIO as GPIO
import time
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

imu_data = {
        "timestamp": [],
        "Accelerometer": [],
        "Magnetometer": [],
        "Gyroscope": [],
        "Euler": [],
        "Quaternion": [],
        "Linear Acceleration": [],
        "Gravity": []
    }

accel_dif = 0.0

def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
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

def move_actuator(num_seconds):
    IN1 = 12
    IN2 = 17

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)

    def move_forward():
        print('Moving forward')
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

    def stop():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    def move_backward():
        print('Moving backward')
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)

    try:
       move_forward()
       time.sleep(num_seconds)
       stop()
       time.sleep(num_seconds)
       move_backward()  
       time.sleep(num_seconds)
       stop()
    finally: 
        GPIO.cleanup()
try:
    while True:
        '''print("Temperature: {} degrees C".format(sensor.temperature))'''
    
        print(
           "Temperature: {} degrees C".format(temperature())
        )  # Uncomment if using a Raspberry Pi
        #imu_data["timestamp"].append(int(round(time.time() * 1000)))
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration)) # Right decreases, left increases, Up Increases, Down Decreases, Forward increases, Backward decreases
        #imu_data["Accelerometer"].append(sensor.acceleration)
        print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        #imu_data["Magnetometer"].append(sensor.magnetic)
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        #imu_data["Gyroscope"].append(sensor.gyro)
        print("Euler angle: {}".format(sensor.euler))
        #imu_data["Euler"].append(sensor.euler)
        print("Quaternion: {}".format(sensor.quaternion))
        #imu_data["Quaternion"].append(sensor.quaternion)
        print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration)) # Left right up and down movement
        #imu_data["Linear Acceleration"].append(sensor.linear_acceleration)
        print("Gravity (m/s^2): {}".format(sensor.gravity)) #About 9.81 whateveer axis is pointing down
        #imu_data["Gravity"].append(sensor.gravity)
        print()

        # Find the change in the y linear acceleration
        accel_dif =float(sensor.acceleration[1])-float(accel_dif)

        #message construction for positional data
        message = json.dumps ({
            "sensor": sensorData(sensor)
        })
        #print(message)
        print('---------------------------------------------')
        time.sleep(3)

        #if the change in y linear acceleration is large enough move the actuator
        if accel_dif > 8:
             move_actuator(2)
            #accel_dif = sensor.acceleration[1]
        else:
            continue
except KeyboardInterrupt:
    print('Data collection stopped')
    #with open("imu_data.json", "w") as file:
    #    json.dump(imu_data, file, indent=1)
    print('Saved the data in imu_data.json')