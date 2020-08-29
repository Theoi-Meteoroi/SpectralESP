
import usocket as socket
from time import sleep
from machine import I2C, Pin
from as7265x import *

import network
import esp
esp.osdebug(None)

import gc
gc.collect()

i2c = I2C(scl=Pin(22),sda=Pin(21))
sensor = AS7265X(i2c)

ssid = 'your-ssid'
password = 'your-wpa2-password'

station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
  pass

print('Connection successful')
print(station.ifconfig())
