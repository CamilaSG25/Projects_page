# contador_leds_xiao.py
from machine import Pin
import time

# Pines (ajusta si usas otros GPIO reales)
led0 = Pin(0, Pin.OUT)
led1 = Pin(1, Pin.OUT)
led2 = Pin(2, Pin.OUT)
button = Pin(3, Pin.IN, Pin.PULL_UP)  # botón a GND

pulseCount = 0
lastButtonState = 1
debounceDelay = 50  # ms
lastDebounceTime = 0

# Apagar todo al inicio
led0.value(0)
led1.value(0)
led2.value(0)

while True:
    reading = button.value()

    # Antirrebote
    if reading != lastButtonState:
        lastDebounceTime = time.ticks_ms()

    if time.ticks_diff(time.ticks_ms(), lastDebounceTime) > debounceDelay:
        if reading == 0 and lastButtonState == 1:  # flanco de bajada
            pulseCount += 1

            if pulseCount == 1:
                led0.value(1)
            elif pulseCount == 2:
                led1.value(1)
            elif pulseCount == 3:
                led2.value(1)
            elif pulseCount == 4:
                # RESET
                led0.value(0)
                led1.value(0)
                led2.value(0)
                pulseCount = 0

    lastButtonState = reading
    time.sleep_ms(10)
