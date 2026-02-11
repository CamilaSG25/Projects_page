# blink_xiao_esp32s3.py
from machine import Pin
import time

# En la XIAO ESP32S3 el LED integrado es activo-bajo
LED = Pin(21, Pin.OUT)

while True:
    LED.value(0)      # ON
    time.sleep(0.5)
    LED.value(1)      # OFF
    time.sleep(0.5)
