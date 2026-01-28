---
layout: default
title: Práctica 1
nav_order: 2
---

# Práctica 1

  En la presente práctica se realizó la implementación del programa básico Blink en distintas placas de desarrollo con el objetivo de verificar su correcto funcionamiento, así como comprobar la comunicación entre el entorno de desarrollo y el microcontrolador. El programa Blink consiste en el encendido y apagado periódico de un LED, lo cual permite validar de manera sencilla las salidas digitales, la carga del código y la correcta configuración de la placa utilizada.

  Para el desarrollo de la práctica se emplearon diversas plataformas de control, entre las que se incluyen Arduino Uno y Arduino Nano (ATmega328), ESP32 DevKit V1 (ESP32-WROOM-32), XIAO ESP32S3 Sense y XIAO RP2040. Cada una de estas placas presenta características particulares en cuanto a arquitectura, capacidad de procesamiento y método de comunicación, lo que permitió comparar su comportamiento y proceso de configuración dentro del entorno de programación.

---

## Blink con Arduino Uno

- **Conexión**  
  La comunicación entre la computadora y la placa Arduino Uno/Nano se realizó mediante comunicación serial por USB. Este tipo de comunicación permite la transferencia de datos y programas

  Para establecer correctamente la comunicación entre la computadora y la placa Arduino, se realizaron los siguientes pasos dentro del Arduino IDE:

  Se seleccionó el modelo de la placa utilizada (Arduino Uno o Arduino Nano) desde el menú Herramientas → Placa.

  Se identificó y seleccionó el puerto de comunicación correspondiente desde Herramientas → Puerto, el cual aparece como un puerto serial (por ejemplo, COM8).

  Una vez configurados la placa y el puerto, el programa fue compilado y cargado correctamente en el microcontrolador mediante el botón de carga del entorno de desarrollo.

  ![arduino uno conexión](assets/img/conexion_arduino.jpeg)

- **Código** 
Dentro de la función setup(), se configuró el pin del LED como salida mediante la instrucción pinMode. En la función loop(), se implementó la lógica de encendido y apagado del LED utilizando las instrucciones digitalWrite, acompañadas de retardos temporales de 500 milisegundos mediante la función delay()


  ![Arduino uno programa](assets/img/Programa_arduino.jpeg)

- **Video funcionando**  
    <video controls width="640">
      <source src="{{ '/assets/img/arduino.mp4' | relative_url }}" type="video/mp4">
      Tu navegador no soporta video HTML5.
    </video>

---

## Arduino nano

  - **Conexión** 

    Al igual que en el Arduino Uno, la comunicación USB es convertida internamente a comunicación serial UART mediante un convertidor USB–Serial integrado en la placa Arduino Nano

    Para realizar la programación de la placa Arduino Nano, se llevaron a cabo los siguientes pasos en el entorno de desarrollo Arduino IDE:

    Se seleccionó la placa Arduino Nano desde el menú Herramientas → Placa.

    Se eligió el puerto serial correspondiente desde Herramientas → Puerto, identificado como un puerto USB (por ejemplo, COM5).

    Una vez configurados la placa y el puerto, se procedió a compilar y cargar el programa en el microcontrolador.

    ![arduino nano conexión](assets/img/conexion_nano.jpeg)

    Durante la programación del Arduino Nano, fue necesario ajustar la configuración del procesador debido a que se utilizó una versión antigua de la placa. Para ello, en el Arduino IDE se seleccionó la opción ATmega328P (Old Bootloader) desde el menú Herramientas → Procesador.

    ![arduino nano conexión](assets/img/nano_viejo.jpeg)

  - **Código** 

    El programa desarrollado tiene como finalidad controlar el encendido y apagado del LED integrado en la placa Arduino Nano, identificado como LED_BUILTIN.

    En la función setup(), se configuró el pin del LED como salida digital mediante la instrucción pinMode. Posteriormente, en la función loop(), se implementó una secuencia cíclica en la cual el LED se enciende y apaga utilizando la instrucción digitalWrite, incorporando retardos de tiempo mediante la función delay() para controlar la velocidad del parpadeo.

  ![arduino nano código](assets/img/cod nano.png)

- **Video funcionando**  
    <video controls width="640">
      <source src="{{ '/assets/img/nano.mp4' | relative_url }}" type="video/mp4">
      Tu navegador no soporta video HTML5.
    </video>

---

## ESP32‑WROOM‑32

  -**Conexión**

    Para la correcta programación de la placa ESP32-WROOM-32, se realizaron los siguientes pasos en el Arduino IDE

    Se seleccionó la placa ESP32-WROOM-DA Module desde el menú Herramientas → Placa, correspondiente al módulo utilizado en la práctica.

    Se eligió el puerto de comunicación serial asignado a la placa desde Herramientas → Puerto, identificado como un puerto USB (por ejemplo, COM10).

    Una vez configurados la placa y el puerto, se procedió a compilar y cargar el programa en el microcontrolador.

    ![SP32‑WROOM‑32 conexión](assets/img/wroom.jpeg)

  - **Código** 
  El programa desarrollado tiene como objetivo realizar el parpadeo del LED integrado en la placa ESP32-WROOM-32, el cual se encuentra conectado al pin digital 2.

  Debido a que en algunas configuraciones del ESP32 el pin del LED integrado no se encuentra definido por defecto, se utilizó una directiva de preprocesador (#ifdef) para definir el pin correspondiente (LED_BUILTIN) únicamente en caso de que no estuviera previamente declarado.

  En la función setup(), el pin del LED se configuró como salida digital mediante la instrucción pinMode. Posteriormente, en la función loop(), se implementó una secuencia cíclica en la que el LED se enciende y apaga de manera alternada utilizando la instrucción digitalWrite, incorporando retardos de 500 milisegundos con la función delay() para controlar la frecuencia del parpadeo

  ![wroom cod](assets/img/wroom_cod.jpeg)

  - **Video funcionando**  
    <video controls width="640">
      <source src="{{ '/assets/img/video_wroom.mp4' | relative_url }}" type="video/mp4">
      Tu navegador no soporta video HTML5.
    </video>


---

## 3. Conexión de motores

- Conecta cada motor NEMA 17 a su respectivo conector en la CNC Shield:
  - **Eje X** → Ejemplo: carro con banda dentada.
  - **Eje Y** → Ejemplo: mesa / base con husillo.
  - **Eje Z** → Ejemplo: mecanismo de cremallera, husillo o el eje vertical de tu herramienta.

![Maquina ejemplo CNC](assets/img/cnc.jpg)

### 3.1. Motores con conector estándar

Si estás usando motores NEMA con **conector estándar** (como los que suelen traer cable preensamblado):

- Normalmente basta con insertar el conector en el puerto correspondiente (X, Y, Z) de la shield.
- En muchos cables, el **cable rojo** queda en la parte superior del conector en la shield (pero esto puede variar según fabricante; revisa el datasheet si es posible).

![Motor con conector estándar](assets/img/motor_estandar.jpg)
![Motor con conector estándar2](assets/img/cables_motor.jpg)

### 3.2. Motores bipolares de 4 cables sin conector estándar

Si tus motores son bipolares de **4 cables sueltos**, primero debes identificar las **bobinas**:

1. Con un multímetro en modo continuidad o resistencia:
   - Encuentra qué pares de cables forman cada bobina (tendrán resistencia de unos pocos ohms).
   - Por ejemplo:
     - Bobina A → cables (rojo, azul).
     - Bobina B → cables (verde, negro).

2. Conecta las bobinas al driver (en el conector de la shield) en el siguiente orden de arriba a abajo:

   ```text
   A+   A−   B+   B−
   ```

3. Si al hacer un jog el motor **tiembla pero no gira**, probablemente los cables estén cruzados o las bobinas invertidas:
   - Cambia el orden de los cables (por ejemplo intercambiando A y B) hasta que el giro sea suave y continuo.

> ⚠️ No conectes ni desconectes los motores con la fuente encendida; puedes dañar tanto el driver como el motor.

---

## 4. Finales de carrera (opcional)

Los finales de carrera mejoran la seguridad y permiten hacer **homing** automático.

- Tipo recomendado: **microswitch mecánico con palanca**, usados en modo **NC** (normalmente cerrado).

  ![Finales de carrera](assets/img/switch.jpg)

- Conexión típica en el CNC Shield (conector X-, Y-, Z-):
  - `C` del switch → **GND (G)**.
  - `NC` del switch → **S (Signal)**.
  - Deja sin conectar el pin de **+5 V**.

![Conexión de finales de carrera](assets/img/finales_c.jpg)

Usar NC tiene varias ventajas:

- Si se corta un cable o se desconecta un switch, GRBL lo detecta como fallo.
- Reduce el riesgo de que la máquina se mueva fuera de límites sin detectar el error.

Más adelante, en GRBL, se activan:

- **Homing** (`$22=1`).
- **Límites duros** (`$21=1`) y/o **límites suaves** (`$20=1`).

Los detalles de configuración de homing y límites se describen en la sección de [Calibración](calibracion.md).

---

Con estos pasos, la parte de **hardware y conexiones físicas** queda lista para pasar a:

- Cargar GRBL en el Arduino.
- Configurar parámetros básicos.
- Empezar a probar movimientos desde el software.

---

## Siguiente sección

[Software (GRBL + OpenBuilds)](software.md)
