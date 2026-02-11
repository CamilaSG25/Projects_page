// blink_esp32_devkitv1.ino

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2   // En ESP32 DevKit el LED suele estar en GPIO 2
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);               // Encendido 500 ms
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);               // Apagado 500 ms
}
