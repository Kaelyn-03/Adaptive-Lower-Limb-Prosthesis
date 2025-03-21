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
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

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

while True:
    '''print("Temperature: {} degrees C".format(sensor.temperature))'''
    
    print(
        "Temperature: {} degrees C".format(temperature())
    )  # Uncomment if using a Raspberry Pi
    
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))
    print("Quaternion: {}".format(sensor.quaternion))
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    print("Gravity (m/s^2): {}".format(sensor.gravity))
    print()

    #message construction for positional data
    message = json.dumps ({
        "sensor": sensorData(sensor)
    })
    print(message)

    time.sleep(1)