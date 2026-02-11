# blink_xiao_rp2040.py
from machine import Pin
import time

# En la XIAO RP2040 el LED integrado es activo-bajo
LED = Pin(17, Pin.OUT)

while True:
    LED.value(0)      # ON
    time.sleep(0.5)
    LED.value(1)      # OFF
    time.sleep(0.5)
