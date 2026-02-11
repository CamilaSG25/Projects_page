// blink_uno_nano_100ms.ino

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);                 // LED encendido 100 ms
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);                 // LED apagado 100 ms
}
