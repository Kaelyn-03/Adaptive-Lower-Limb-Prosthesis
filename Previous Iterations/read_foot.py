# read_foot.py
# Code to take in data from the foot sensors

import spidev
import time

spi0 = spidev.SpiDev()
spi0.open(0,0)
spi0.max_speed_hz = 1350000

spi1 = spidev.SpiDev()
spi1.open(0,1)
spi1.max_speed_hz = 1350000

def read_channel(spi_bus, channel):
   # if channel < 0 or channel > 7:
   #     return -1
    adc = spi_bus.xfer2([1, (8+channel)<<4, 0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

try:
    while True:
        print('First MCP3008 Reading:')
        for channel in range(8):
            print(f"Channel {channel}: {read_channel(spi0, channel)}", end="\t")
        print('\n')

        print('Second MCP3008 Reading:')
        for channel in range(8):
            print(f"Channel {channel+8}: {read_channel(spi1, channel)}", end="\t")
        print('\n\n\n\n\n\n')
        print('-----------------------------------')
        time.sleep(3)        
except KeyboardInterrupt:
    spi0.close()
    spi1.close()
    print('SPI closed!')

