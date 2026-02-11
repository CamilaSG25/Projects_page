# cod de esp32

from machine import Pin
from time import sleep
import sys
import uselect

===== Hardware =====
btn = Pin(4, Pin.IN, Pin.PULL_UP)   # Botón físico en D4 (a GND)
led = Pin(3, Pin.OUT)               # LED en D3 (lo controla la interfaz)

===== Estados =====
estado_btn = 0
estado_led = 0
last = 1

Serial no bloqueante (lee comandos de la PC)
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

print("LISTO")

while True:
    # ---- 1) Botón físico -> ON/OFF (solo imprime) ----
    now = btn.value()
    if last == 1 and now == 0:
        estado_btn ^= 1
        print("ON" if estado_btn else "OFF")
        sleep(0.25)  # antirrebote
    last = now

    # ---- 2) Comandos desde interfaz -> LED D3 ----
    if poll.poll(0):
        cmd = sys.stdin.readline().strip()

        if cmd == "LED3_ON":
            estado_led = 1
            led.value(1)
            print("LED3=ON")

        elif cmd == "LED3_OFF":
            estado_led = 0
            led.value(0)
            print("LED3=OFF")

        elif cmd == "LED3_TOGGLE":
            estado_led ^= 1
            led.value(estado_led)
            print("LED3=ON" if estado_led else "LED3=OFF")

    sleep(0.01)
