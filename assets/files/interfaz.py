import tkinter as tk
import serial
import time

PORT = "COM8"   # <-- cambia al COM real de tu XIAO
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(1)

root = tk.Tk()
root.title("XIAO: Botón + LED")

canvas = tk.Canvas(root, width=220, height=220)
canvas.pack(padx=10, pady=10)

circulo = canvas.create_oval(40, 40, 180, 180, fill="red")

lbl_led = tk.Label(root, text="LED D3: OFF")
lbl_led.pack()

def enviar(cmd):
    ser.write((cmd + "\n").encode())

# Botón de la interfaz -> controla el LED en D3
tk.Button(root, text="Toggle LED D3", command=lambda: enviar("LED3_TOGGLE")).pack(pady=10)

def leer_serial():
    if ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        # Estado del botón físico (D4)
        if line == "ON":
            canvas.itemconfig(circulo, fill="green")
        elif line == "OFF":
            canvas.itemconfig(circulo, fill="red")

        # Estado del LED D3 (confirmación)
        elif line == "LED3=ON":
            lbl_led.config(text="LED D3: ON")
        elif line == "LED3=OFF":
            lbl_led.config(text="LED D3: OFF")

    root.after(50, leer_serial)

leer_serial()
root.mainloop()
