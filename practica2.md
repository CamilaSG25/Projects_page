---
layout: default
title: Práctica 2
nav_order: 3
---

# Práctica 2

En la presente práctica se realizó la evaluación de distintos protocolos de comunicación entre plataformas de sistemas embebidos, con el objetivo de analizar su desempeño en términos de velocidad y latencia. Se implementaron pruebas de comunicación enviando 1000 mensajes entre dispositivos en este caso entre ATMEGA328p(Arduino UNO) y el XIAO-ESP32-S3 para posteriormente registrar y analizar el tiempo de respuesta de cada protocolo en cada dispositivo.

Los protocolos evaluados fueron:

- **UART**  
- **I2C**  
- **SPI**  

Para cada protocolo se documentan las **conexiones físicas**, los **códigos** cargados en cada microcontrolador y los resultados obtenidos mediante **gráficas de latencia**.

---

# UART

  - **Conexión**

  Se realizó la conexión física entre el Arduino UNO y el dispositivo receptor utilizando comunicación UART.  
  Se compartió una referencia común de tierra (GND) y se conectaron los pines TX y RX correspondientes.
  Para este caso se ocupó un divisor de voltaje con una resistencia de 1kΩ y 460Ω para eviatr 5v en la XIAO,
  gracias al divisor se obtuvo un voltaje seguro de ~3.4v


  ![UART conexión ](assets/img/Conexion_UART1.jpeg)

  ![UART conexión 2](assets/img/Conexion_UART2.jpeg)

  --- 

  ## UART Arduino UNO

  - **Medir latencia de comunicacion UART desde Arduino UNO**

  - **Código (Arduino UNO )**
  
   <details>
   <summary><strong>Código Arduino UNO (UART desde Arduino UNO)</strong></summary>

   ```cpp

   // =====  UART - Arduino UNO - MASTER  =====

   #define BAUD_RATE 115200
   #define N_MSG     1000

   String rxLine = "";

   void setup() {
   Serial.begin(BAUD_RATE);   // UART hardware: D0(RX) / D1(TX)
   delay(300);

   // Header CSV (sale por UART hacia el XIAO)
   Serial.println("idx,rtt_us,latency_us");
   }

  void loop() {
  static bool done = false;
  if (done) return;

  for (int i = 0; i < N_MSG; i++) {
    unsigned long t0 = micros();

    // Enviar PING
    Serial.print("P,");
    Serial.print(i);
    Serial.print("\n");

    // Esperar ACK A,<id>
    rxLine = "";
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
          // Línea completa recibida
          if (rxLine.startsWith("A,")) {
            int id = rxLine.substring(2).toInt();
            if (id == i) {
              unsigned long t1 = micros();
              unsigned long rtt = t1 - t0;
              unsigned long lat = rtt / 2;

              // CSV (también sale por UART hacia el XIAO)
              Serial.print(i);
              Serial.print(",");
              Serial.print(rtt);
              Serial.print(",");
              Serial.println(lat);
              break;
            }
          }
          rxLine = "";
        } else {
          if (rxLine.length() < 40) rxLine += c;
        }
      }
    }
  }

  done = true;
  }

  void setup(){}

 - **Código (XIAO ESP32-S3 )**

   <details>
   <summary><strong>Código XIAO (UART desde Arduino UNO)</strong></summary>

   ```cpp
  // Tu código aquí
  void setup() {
   }
  // ===== UART - XIAO ESP32-S3 - SLAVE  =====

   #define UART_RX_PIN D7
   #define UART_TX_PIN D6
   #define BAUD_RATE   115200

   String buf = "";

   bool isDigit(char c) { return (c >= '0' && c <= '9'); }

   // valida exactamente: digits,digits,digits
   bool isCsvRow3(const String &s) {
  if (s.length() < 5) return false;      // mínimo "0,0,0"
  if (!isDigit(s[0])) return false;      // debe iniciar con número
  int commas = 0;

  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c == ',') commas++;
    else if (!isDigit(c)) return false;  // solo dígitos o comas
  }
  return commas == 2;
  }

  void setup() {
  Serial.begin(115200);  // USB -> aquí copias el CSV limpio
  delay(200);

  Serial1.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Marca para saber que este sketch está corriendo
  Serial.println("XIAO: listo (CSV limpio)");
  }

  void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      // quitar CR si llega \r\n
      if (buf.length() > 0 && buf[buf.length() - 1] == '\r') {
        buf.remove(buf.length() - 1);
      }

      // 1) Responder a P,<id>
      if (buf.startsWith("P,")) {
        int id = buf.substring(2).toInt();
        Serial1.print("A,");
        Serial1.print(id);
        Serial1.print("\n");
      }

      // 2) Imprimir SOLO CSV limpio al USB
      if (buf == "idx,rtt_us,latency_us" || isCsvRow3(buf)) {
        Serial.println(buf);
      }

      buf = "";
    } else {
      if (buf.length() < 140) buf += c;
      else buf = ""; // si se desborda, descartamos
    }
  }
  }

## Resultados
      A continuación se presentará la gráfica de resultado sobre el tiempo de respuesta y latencia del protocolo de comunicación UART en Arduino UNO

![UART grafica ARDUINO](assets/img/Grafica_UART_Arduino.jpeg)

      En la gráfica se observa que al inicio la latencia de los mensajes UART es menor, pero cambia hasta llegar a un valor  estable cercano a 1200 µs. Este comportamiento ocurre porque, en los primeros mensajes, el sistema acaba de arrancar y los buffers de comunicación todavía están vacíos, por lo que los mensajes se procesan más rápido.

      Conforme se envían más mensajes, la comunicación UART entra en un funcionamiento continuo ,entonces, los buffers se llenan, el microcontrolador empieza a esperar respuestas de forma repetitiva y el tiempo de transmisión se vuelve constante. Esto provoca que la latencia aumente ligeramente y luego se mantenga casi igual para el resto de los mensajes.
     

   ## UART XIAO ESP32-S3

    **Medir latencia de comunicacion UART desde XIAO ESP32-S3**
      
  - **Código (Arduino UNO )**

   <details>
   <summary><strong>Código Arduino UNO (UART desde XIAO ESP32-S3)</strong></summary>

   ```cpp
  // Tu código aquí

  // ======== UART - Arduino UNO - SLAVE ========

  // RX = D0, TX = D1 (Serial hardware)
  // =======================

  const uint32_t BAUD = 115200;  

  void setup() {
  Serial.begin(BAUD);
  }

  void loop() {
  if (Serial.available()) {
  int b = Serial.read();
  if (b >= 0) {
    Serial.write((uint8_t)b);   // eco inmediato
      }
    }
  }

  void setup(){}

 - **Código (XIAO ESP32-S3 )**

  <details>
   <summary><strong>Código  XIAO (UART desde XIAO ESP32-S3)</strong></summary>

   ```cpp

  // ======== UART - XIAO ESP32-S3 - MASTER =====

  // TX = D6, RX = D7 (Serial1)
  // USB Serial -> CSV: idx,rtt_us,latency_us
  // =======================

  #include <Arduino.h>

  const uint32_t BAUD = 115200;   // Cambia aquí si subes baud (igual en UNO)

  // Pines UART en XIAO ESP32-S3
  const int TX_PIN = D6;
  const int RX_PIN = D7;

  const uint16_t N_SAMPLES = 2000;     // Muestras (puedes dejar 1000 si quieres)
  const uint32_t TIMEOUT_US = 30000;   // 30 ms (suficiente incluso si hay algún fallo)

  static inline bool read_one_byte(uint8_t &out, uint32_t timeout_us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < timeout_us) {
  int n = Serial1.available();
  if (n > 0) {
    int b = Serial1.read();
    if (b >= 0) { out = (uint8_t)b; return true; }
  }
  // sin delays: lo más rápido posible
  }
  return false;
  }

  void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // Header CSV
  Serial.println("idx,rtt_us,latency_us");
  } 

  void loop() {
  for (uint32_t i = 0; i < N_SAMPLES; i++) {

  // Limpia basura previa
  while (Serial1.available()) (void)Serial1.read();

  uint8_t tx = (uint8_t)(i & 0xFF);

  uint32_t t0 = micros();
  Serial1.write(tx); // NO flush

  uint8_t rx = 0;
  bool ok = read_one_byte(rx, TIMEOUT_US);
  uint32_t t1 = micros();

  if (!ok || rx != tx) {
    Serial.print(i);
    Serial.println(",nan,nan");
    continue;
  }

  uint32_t rtt = t1 - t0;
  uint32_t latency = rtt / 2;

  Serial.print(i);
  Serial.print(",");
  Serial.print(rtt);
  Serial.print(",");
  Serial.println(latency);
  }

  while (true) delay(1000);
  }
  
  void setup(){}


## Resultados
      A continuación se presentará la gráfica de resultado sobre el tiempo de respuesta y latencia del protocolo de comunicación UART en XIAO ESP32-S3

  ![UART Grafica XIAO](assets/img/Grafica_UART_XIAO.jpeg)

    En esta práctica se evaluó el protocolo UART en la XIAO ESP32-S3 mediante comunicación ping-pong. Los resultados muestran una latencia promedio de ~149 µs y una baja variación (1.89 µs), lo que indica una comunicación rápida y estable.

    La mayoría de las mediciones se concentran en un rango estrecho, confirmando la confiabilidad del sistema. En comparación con el Arduino UNO, la XIAO ESP32-S3 presenta un mejor desempeño, por lo que UART en esta plataforma es adecuado para aplicaciones embebidas que requieren baja latencia.

# I2C

    **Conexión**

    Para el armado se hizo una conexión de pull up a 3.3v para evitar daños en el xiao
    Con 2 resistencias de 470ohms, asó como se puede observar en las siguientes imágenes, aquí las conexiones van "directas " no cruzadas es decir cada pin va uno a uno (SDA y SCL).


   ![I2C conexión ](assets/img/Conexion_I2C1.jpeg)

   ![I2C conexión 2](assets/img/Conexion_I2C2.jpeg)

    ## I2C Arduino UNO

    **Medir latencia de comunicacion I2C desde Arduino UNO**

  - **Código (Arduino UNO )**

   <details>
   <summary><strong>Código Arduino(I2C desde Arduino UNO)</strong></summary>

   ```cpp

    //======= I2C - Arduino UNO - MASTER =========

    #include <Wire.h>

    #define SLAVE_ADDR 0x08

    volatile uint8_t last_token = 0;
    volatile uint32_t t_rx = 0;

    void onReceive(int n) {
      while (Wire.available()) {
        last_token = Wire.read();
      }
      t_rx = micros();
    }

    void onRequest() {
      uint32_t latency = micros() - t_rx;

      Wire.write(last_token);
      Wire.write((uint8_t)(latency & 0xFF));
      Wire.write((uint8_t)((latency >> 8) & 0xFF));
      Wire.write((uint8_t)((latency >> 16) & 0xFF));
      Wire.write((uint8_t)((latency >> 24) & 0xFF));
    }

    void setup() {
      Wire.begin(SLAVE_ADDR);
      Wire.onReceive(onReceive);
      Wire.onRequest(onRequest);

      // Mantener bus a 3.3V: pull-ups internos OFF
      PORTC &= ~((1 << PC4) | (1 << PC5));  // A4/A5 sin pull-up
      pinMode(A4, INPUT);
      pinMode(A5, INPUT);
      digitalWrite(A4, LOW);
      digitalWrite(A5, LOW);
    }

    void loop() {}

    void setup(){}

 - **Código ( XIAO ESP32-S3 )**

   <details>
   <summary><strong>Código  XIAO (I2C desde Arduino UNO)</strong></summary>

   ```cpp

  //========= I2C - XIAO SP32-S3 -SLAVE===========

  #include <Wire.h>

  #define SLAVE_ADDR 0x08

  const int SDA_PIN = 5;  // D4/SDA = GPIO5
  const int SCL_PIN = 6;  // D5/SCL = GPIO6

  // Cambia entre 100000 y 400000 para comparar velocidad
  const uint32_t I2C_HZ = 100000;

  // Cuántas mediciones
  const uint32_t N = 2000;

  // Pausa entre mediciones (0 para máximo estrés)
  const uint32_t PAUSE_US = 200;

  uint32_t readU32LE() {
    uint32_t v = 0;
    v |= (uint32_t)Wire.read();
    v |= (uint32_t)Wire.read() << 8;
    v |= (uint32_t)Wire.read() << 16;
    v |= (uint32_t)Wire.read() << 24;
    return v;
  }

  void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_HZ);

    Serial.println("idx,rtt_us,echo_ok,arduino_latency_us");
  }

  void loop() {
    static uint32_t idx = 0;
    if (idx >= N) while (1) {}

    uint8_t token = (uint8_t)(idx & 0xFF);

    uint32_t t0 = micros();

    // WRITE 1 byte
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(token);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
      uint32_t rtt = micros() - t0;
      Serial.printf("%lu,%lu,%d,%d\n", idx, rtt, 0, -1);
      idx++;
      delayMicroseconds(PAUSE_US);
      return;
    }

    // READ 5 bytes
    uint8_t got = Wire.requestFrom(SLAVE_ADDR, (uint8_t)5);

    uint8_t echo = 0xFF;
    uint32_t latency = 0xFFFFFFFF;

    if (got == 5) {
      echo = Wire.read();
      latency = readU32LE();
    }

    uint32_t rtt = micros() - t0;
    uint8_t ok = (echo == token) ? 1 : 0;

    Serial.printf("%lu,%lu,%d,%ld\n",
                  idx, rtt, ok,
                  (latency == 0xFFFFFFFF) ? -1L : (long)latency);

    idx++;
    delayMicroseconds(PAUSE_US);
  }

   void setup(){}

 ## Resultados
      A continuación se presentará la gráfica de resultado sobre el tiempo de respuesta y latencia del protocolo de comunicación I2C en XIAO ESP32-S3

  ![I2C grafica arduino](assets/img/Grafica_I2C_Arduino.jpeg)

    En la gráfica se observa que al inicio la latencia de los mensajes UART es menor, pero cambia hasta llegar a un valor  estable cercano a 1200 µs. Este comportamiento ocurre porque, en los primeros mensajes, el sistema acaba de arrancar y los buffers de comunicación todavía están vacíos, por lo que los mensajes se procesan más rápido.

    Conforme se envían más mensajes, la comunicación UART entra en un funcionamiento continuo ,entonces, los buffers se llenan, el microcontrolador empieza a esperar respuestas de forma repetitiva y el tiempo de transmisión se vuelve constante. Esto provoca que la latencia aumente ligeramente y luego se mantenga casi igual para el resto de los mensajes.
  
   ## I2C XIAO ESP32-S3

   **Medir latencia de comunicacion I2C desde XIAO ESP32-S3**
      
  - **Código (Arduino UNO )**

    <details>
    <summary><strong>Código Arduino(I2C desde XIAO ESP32-S3)</strong></summary>

   ```cpp
    // ====== I2C - Arduino UNO - SLAVE======

    #include <Wire.h>

    #define SLAVE_ADDR 0x08
    volatile uint8_t lastByte = 0;

    void receiveEvent(int howMany) {
      if (Wire.available()) {
        lastByte = Wire.read();
      }
    }

    void requestEvent() {
      Wire.write(lastByte); // responde el mismo dato
    }

    void setup() {
      Wire.begin(SLAVE_ADDR);
      Wire.onReceive(receiveEvent);
      Wire.onRequest(requestEvent);
    }

    void loop() {
    }

    void setup(){}

 - **Código (XIAO ESP32-S3 )**

   <details>
   <summary><strong>Código XIAO(I2C desde XIAO ESP32-S3)</strong></summary>

   ```cpp

    // =========== I2C - XIAO ESP3 - MASTER ==============

    #include <Wire.h>

    #define SLAVE_ADDR 0x08
    #define MAX_MSGS 1000

    uint32_t idx = 0;

    void setup() {
      Serial.begin(115200);
      Wire.begin();              // XIAO como master
      Wire.setClock(400000);     // 400 kHz

      Serial.println("idx,rtt_us");
    }

    void loop() {

      if (idx >= MAX_MSGS) {
        // Detener experimento
        while (1) {
          delay(1000);
        }
      }

      uint32_t t0, t1;

      t0 = micros();

      // Enviar dato
      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write((uint8_t)(idx & 0xFF));
      Wire.endTransmission();

      // Pedir respuesta
      Wire.requestFrom(SLAVE_ADDR, 1);
      if (Wire.available()) {
        Wire.read();
      }

      t1 = micros();

      uint32_t rtt = t1 - t0;

      Serial.print(idx);
      Serial.print(",");
      Serial.println(rtt);

      idx++;

      delay(5); // puedes quitarlo si quieres saturar el bus
    }

   void setup(){}

 ## Resultados
  A continuación se presentará la gráfica de resultado sobre el tiempo de respuesta y latencia del protocolo de comunicación I2C en XIAO ESP32-S3

  ![I2C grafica XIAO](assets/img/Grafica_I2C_XIAO.jpeg)

  En esta práctica se evaluó el protocolo UART en la XIAO ESP32-S3 mediante comunicación ping-pong. Los resultados muestran una latencia promedio de ~222 µs y una baja variación (0.55 µs), lo que indica una comunicación rápida y estable.

  La mayoría de las mediciones se concentran en un rango estrecho, confirmando la confiabilidad del sistema. En comparación con el Arduino UNO, la XIAO ESP32-S3 presenta un mejor desempeño, por lo que UART en esta plataforma es adecuado para aplicaciones embebidas que requieren baja latencia.


# SPI
   **Conexión**

    Para el armado se hicieron conexiones deirectas, cada pin se conecto al otro del microcontrolador contrario.
    De esta manera no se perdieron datos y se obtuvieron buenos resultados, aunque alguna configuración para bahar el voltaje hubiera sido una buena gorma de evitar daños.

   ![I2C conexión ](assets/img/Conexion_SPI1.jpeg)

   ![I2C conexión 2](assets/img/Conexion_SPI2.jpeg)

    ## I2C Arduino UNO

    **Medir latencia de comunicacion SPI desde Arduino UNO**

  - **Código (Arduino UNO )**

   <details>
   <summary><strong>Código Arduino(SPI desde Arduino)</strong></summary>

   ```cpp

    // ========== SPI - ARDUINO UNO - MAESTRO ==========

    #include <SPI.h>

    const uint8_t SS_PIN = 10;
    const uint8_t CMD    = 0xA5;

    const uint16_t MAX_SAMPLES = 1000;
    uint16_t idx = 0;

    void setup() {
      Serial.begin(115200);

      pinMode(SS_PIN, OUTPUT);
      digitalWrite(SS_PIN, HIGH);

      SPI.begin();
      SPI.setDataMode(SPI_MODE0);
      SPI.setBitOrder(MSBFIRST);

      // Velocidad segura (UNO 5V → XIAO 3.3V)
      SPI.setClockDivider(SPI_CLOCK_DIV64); // ~250 kHz

      Serial.println("idx,spi_time_us,rx0,rx1");
    }

    void loop() {
      // 🔒 Límite de 1000 muestras
      if (idx >= MAX_SAMPLES) {
        // Detenemos el loop para que no mande más datos
        while (true) {
          // aquí se queda para siempre
        }
      }

      uint32_t t0 = micros();

      digitalWrite(SS_PIN, LOW);
      uint8_t rx0 = SPI.transfer(CMD);
      uint8_t rx1 = SPI.transfer(0x00);
      digitalWrite(SS_PIN, HIGH);

      uint32_t t1 = micros();

      Serial.print(idx);
      Serial.print(",");
      Serial.print(t1 - t0);
      Serial.print(",");
      Serial.print(rx0);
      Serial.print(",");
      Serial.println(rx1);

      idx++;

      delay(5); // pequeño delay para estabilidad (puedes bajarlo o quitarlo)
    }

    void setup(){}


 - **Código (XIAO ESP32-S3 )**

   <details>
   <summary><strong>Código XIAO(SPI desde Arduino)</strong></summary>

   ```cpp

    // =========== SPI - XIAO ESP32-S3 - SLAVE ===========0

    #include <Arduino.h>

    extern "C" {
      #include "driver/spi_slave.h"
      #include "driver/gpio.h"
    }

    static const int PIN_SS   = 7;   // XIAO D7  (CS/SS)
    static const int PIN_SCK  = 8;   // XIAO D8
    static const int PIN_MOSI = 10;  // XIAO D10
    static const int PIN_MISO = 9;   // XIAO D9

    static const uint8_t CMD = 0xA5;

    static uint8_t tx_buf[2];
    static uint8_t rx_buf[2];

    static uint8_t counter = 0;

    void setup() {
      // Configuración del bus SPI para esclavo
      spi_bus_config_t buscfg = {};
      buscfg.mosi_io_num = PIN_MOSI;
      buscfg.miso_io_num = PIN_MISO;
      buscfg.sclk_io_num = PIN_SCK;
      buscfg.quadwp_io_num = -1;
      buscfg.quadhd_io_num = -1;

      spi_slave_interface_config_t slvcfg = {};
      slvcfg.spics_io_num = PIN_SS;
      slvcfg.flags = 0;
      slvcfg.queue_size = 1;
      slvcfg.mode = 0; // SPI_MODE0

      // Inicializa SPI esclavo (usa SPI2_HOST en ESP32/ESP32-S3)
      esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

      // Si falla, nos quedamos aquí para que lo notes
      if (ret != ESP_OK) {
        while (true) { delay(1000); }
      }

      tx_buf[0] = 0;
      tx_buf[1] = 0;
      rx_buf[0] = 0;
      rx_buf[1] = 0;
    }

    void loop() {
      // Prepara respuesta: contador
      tx_buf[0] = counter;
      tx_buf[1] = 0;

      spi_slave_transaction_t t = {};
      t.length = 2 * 8;            // 2 bytes en bits
      t.tx_buffer = tx_buf;
      t.rx_buffer = rx_buf;

      // Espera a que el maestro haga una transacción (bloqueante)
      esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
      if (ret == ESP_OK) {
        if (rx_buf[0] == CMD) {
          counter++;
        }
      }
    }

    void setup (){}

 ## Resultados
      A continuación se presentará la gráfica de resultado sobre el tiempo de respuesta y latencia del protocolo de comunicación SPI en Arduino UNO

  ![I2C grafica arduino](assets/img/Grafica_SPI_Arduino.jpeg)

    A partir de la gráfica de latencia de mensajes SPI en Arduino, se observa que el protocolo SPI presenta un comportamiento altamente estable y consistente durante la transmisión de 1000 mensajes entre los dispositivos.

    La latencia promedio medida fue de aproximadamente 77 µs, con una desviación estándar de 1.71 µs, lo cual indica una muy baja variabilidad entre las muestras. La mayoría de los valores de latencia se concentran dentro del rango delimitado por ±1 desviación estándar (75 µs a 78 µs), evidenciando que el sistema mantiene tiempos de respuesta prácticamente constantes a lo largo de toda la prueba.
 
   ## I2C XIAO ESP32-S3

   **Medir latencia de comunicacion I2C desde XIAO ESP32-S3**
      
   - **Código (Arduino UNO )**

   <details>
   <summary><strong>Código Arduino(SPI desde XIAO ESP32-S3)</strong></summary>

   ```cpp

    // ========== SPI - ARDUINO UNO - SLAVE ==========

      #include <SPI.h>

    volatile uint8_t replyByte = 0;
    volatile uint8_t lastReceived = 0;

    ISR(SPI_STC_vect) {
      lastReceived = SPDR;
      SPDR = replyByte;
      if (lastReceived == 0xA5) replyByte++;
    }

    void setup() {
      pinMode(MISO, OUTPUT);
      pinMode(SS, INPUT_PULLUP);
      SPCR |= _BV(SPE);
      SPCR |= _BV(SPIE);
      SPDR = replyByte;
    }

    void loop() {}

    void setup(){}

 - **Código (XIAO ESP32-S3 )**

   <details>
   <summary><strong>Código XIAO(SPI desde XIAO ESP32-S3)</strong></summary>

   ```cpp

    // ========= SPI - XIAO ESP32-S3 - MASTER ========

    #include <SPI.h>

    static const int PIN_SS   = 7;
    static const int PIN_SCK  = 8;
    static const int PIN_MISO = 9;
    static const int PIN_MOSI = 10;

    static const uint8_t CMD = 0xA5;
    static const uint16_t MAX_SAMPLES = 1000;

    void setup() {
      Serial.begin(115200);
      while (!Serial) {}

      pinMode(PIN_SS, OUTPUT);
      digitalWrite(PIN_SS, HIGH);

      SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

      Serial.println("idx,rtt_us,rx");
    }

    void loop() {
      static uint16_t idx = 0;
      if (idx >= MAX_SAMPLES) while (true) {}

      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

      uint32_t t0 = micros();

      digitalWrite(PIN_SS, LOW);

      SPI.transfer(CMD);        // el UNO recibe CMD y prepara respuesta
      SPI.transfer(0x00);       // ciclo extra para que se cargue bien SPDR
      uint8_t rx = SPI.transfer(0x00); // aquí lees respuesta estable

      digitalWrite(PIN_SS, HIGH);

      uint32_t t1 = micros();

      SPI.endTransaction();

      Serial.print(idx++);
      Serial.print(",");
      Serial.print(t1 - t0);
      Serial.print(",");
      Serial.println(rx);
    }

    void setup(){}

 ## Resultados
  A continuación se presentará la gráfica de resultado sobre el tiempo de respuesta y latencia del protocolo de comunicación SPI en XIAO ESP32-S3

  ![I2C grafica XIAO](assets/img/Grafica_SPI_XIAO.jpeg)

  Los resultados obtenidos al medir la latencia de mensajes SPI con el XIAO como maestro muestran un comportamiento estable y de baja latencia durante la transmisión de 1000 mensajes. La latencia promedio fue de aproximadamente 33 µs,con una desviación estándar de 1.09 µs, lo que indica una variación mínima entre las muestras.
  En comparación con las mediciones realizadas con el Arduino UNO como maestro, el XIAO presenta una menor latencia, lo cual se explica por su mayor capacidad de procesamiento y manejo más eficiente del protocolo SPI. En conclusión, SPI demuestra ser un protocolo rápido y confiable, especialmente cuando se utiliza una plataforma de mayor rendimiento como el XIAO.

---

## Siguiente sección

[Calibración](calibracion.md)