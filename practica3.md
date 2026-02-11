---
layout: default
title: Tkinter comunicado con ESP32
nav_order: 4
---

# Tkinter comunicado con ESP32

  En la presente práctica se realizó la comunicación serial entre una computadora personal y una placa XIAO ESP32S3 utilizando MicroPython y una interfaz gráfica desarrollada en Python con Tkinter. El objetivo principal fue establecer un sistema de interacción bidireccional que permitiera tanto la lectura de un botón físico conectado a la placa como el control de un actuador (LED) desde una interfaz gráfica en la computadora.

---

## Codigo generado en Thonny con Tkinter para interfaz gráfica

- **Código**  

        # Interfaz
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

El código presentado implementa una interfaz gráfica desarrollada en Python utilizando la biblioteca Tkinter, cuya función principal es establecer comunicación serial con una placa XIAO ESP32S3 para el monitoreo y control de entradas y salidas digitales.

Inicialmente, se importan las bibliotecas necesarias: tkinter para la creación de la interfaz gráfica, serial para la comunicación a través del puerto USB y time para introducir retardos que aseguren la correcta inicialización del puerto serial. Posteriormente, se configura el puerto de comunicación (COM8) y la velocidad de transmisión de datos a 115200 baudios, estableciendo la conexión serial con la placa.

Una vez inicializada la comunicación, se crea la ventana principal de la interfaz gráfica, en la cual se incorporan elementos visuales como un canvas que contiene un círculo, el cual funciona como indicador del estado del botón físico conectado a la placa. Dicho círculo cambia de color de rojo a verde dependiendo de los mensajes recibidos por el puerto serial (OFF u ON).

Adicionalmente, se incluye una etiqueta de texto que muestra el estado del LED conectado al pin D3, permitiendo al usuario conocer si este se encuentra encendido o apagado. Para el control del LED, se implementa un botón en la interfaz gráfica que, al ser presionado, envía un comando serial a la placa para alternar el estado del LED.

La función encargada de la lectura de datos seriales se ejecuta de manera periódica mediante el método after() de Tkinter, lo cual permite recibir información de la placa sin bloquear la ejecución de la interfaz. De esta manera, se asegura una comunicación continua y en tiempo real entre la computadora y el sistema embebido.

---

## Codigo generado en Thonny para la programación del ESP32

  - **Código** 

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

    El código implementa el control y monitoreo de una XIAO ESP32S3 utilizando MicroPython, permitiendo la interacción simultánea con un botón físico y un LED mediante comunicación serial con una computadora.

    En primer lugar, se configuran los pines de entrada y salida digital: un botón físico conectado al pin D4, el cual utiliza una resistencia pull-up interna para la correcta detección de pulsaciones, y un LED conectado al pin D3, cuyo estado es controlado desde una interfaz gráfica en la computadora. Para la gestión de estos elementos se emplean variables de estado que permiten identificar cambios en el botón y en el LED.

    Posteriormente, se implementa un mecanismo de lectura serial no bloqueante, el cual permite verificar de manera continua si existen datos entrantes desde la computadora sin detener la ejecución principal del programa. Este tipo de lectura evita que el microcontrolador quede esperando indefinidamente la llegada de datos por el puerto serial, lo que permite que otras tareas, como la detección del botón físico, sigan ejecutándose de forma simultánea y eficiente.

    Durante el ciclo principal del programa, el estado del botón físico es evaluado constantemente y, al detectarse una pulsación, se envían mensajes ON u OFF a través del puerto serial para su visualización en la interfaz gráfica. Al mismo tiempo, el microcontrolador recibe y procesa comandos enviados desde la computadora, los cuales permiten encender, apagar o alternar el estado del LED conectado al pin D3.

---

## Ejecución del programa en CMD y dependencias necesarias

  - **Ejecución del programa** 
    
    Para ejecutar la interfaz gráfica desarrollada en Python desde el Símbolo del sistema (CMD), es necesario indicar correctamente la ubicación del archivo .py. Cuando el archivo se encuentra en una carpeta con espacios o dentro de directorios específicos (por ejemplo, en OneDrive), se recomienda utilizar la ruta completa (ruta absoluta) entre comillas para evitar errores de lectura en la terminal. Esto permite que el comando identifique sin ambigüedades el archivo a ejecutar.

    Adicionalmente, para que el programa funcione correctamente es indispensable contar con las librerías necesarias instaladas en el entorno de Python. En particular, debido a que la interfaz se comunica con la XIAO ESP32S3 mediante puerto serial, se requiere instalar el módulo pyserial, ya que este permite abrir el puerto COM y leer/escribir datos desde Python. La instalación se realiza desde CMD mediante:

      pip install pyserial

    Una vez instalada la librería, el programa puede ejecutarse y visualizarse correctamente. En caso de no contar con dicha dependencia, el sistema mostrará errores del tipo “No module named serial”, indicando que el módulo requerido no está disponible.

    Finalmente, se debe considerar que el puerto COM solo puede ser utilizado por una aplicación a la vez; por ello, antes de ejecutar la interfaz se recomienda cerrar Thonny u otros monitores seriales para evitar conflictos en la conexión.

---

## Video funcional

- **Video funcionando**  
    <video controls width="640">
      <source src="{{ '/assets/img/p3_video.mp4' | relative_url }}" type="video/mp4">
      Tu navegador no soporta video HTML5.
    </video>

---
## Siguiente sección

[Primer archivo G-code (.nc)](primer-gcode.md)
