---
layout: default
title: Práctica 2
nav_order: 3
---

# Práctica 2

En la presente práctica se realizó la evaluación de distintos **protocolos de comunicación** entre plataformas de sistemas embebidos, con el objetivo de analizar su desempeño en términos de **velocidad y latencia**. Se implementaron pruebas de comunicación enviando **1000 mensajes** entre dispositivos para posteriormente registrar y analizar el tiempo de respuesta de cada protocolo.

Los protocolos evaluados fueron:

- UART  
- I2C  
- SPI  

Para cada protocolo se documentan las **conexiones físicas**, los **códigos cargados en cada microcontrolador** y los **resultados obtenidos mediante gráficas de latencia**.

---

- # UART

   - **Conexión**

      Se realizó la conexión física entre el Arduino UNO y el dispositivo receptor utilizando comunicación UART.  
      Se compartió una referencia común de tierra (GND) y se conectaron los pines TX y RX correspondientes.

      ![UART Arduino UNO conexión 1](assets/img/uart_arduino_conexion_1.jpg)

      ![UART Arduino UNO conexión 2](assets/img/uart_arduino_conexion_2.jpg)

      ## UART Arduino UNO

       -**Medir latencia de comunicacion UART desde Arduino UNO

         - **Código (Arduino UNO )**


       // PEGAR AQUÍ CÓDIGO UART ARDUINO UNO EMISOR

         - **Código (XIAO ESP32-S3 )**


       // PEGAR AQUÍ CÓDIGO UART ARDUINO UNO EMISOR

## Resultados

## UART Arduino UNO


---

## Siguiente sección

[Calibración](calibracion.md)
