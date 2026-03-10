#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

// ====== WIFI ======
const char* ssid = "IZZI-59A2_EXT";
const char* pass = "D4AB82FA59A2";

// ====== RENDER API ======
const char* URL_STATE = "https://leds-esp32-back-render.onrender.com/api/state";

// ====== NEOPIXEL ======
#define LED_PIN 4
#define NUM_LEDS 8
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

WiFiClientSecure client;

// "#RRGGBB" -> uint32_t
uint32_t hexToColor(const String& hex) {
  if (hex.length() < 7 || hex[0] != '#') return strip.Color(0,0,0);
  long number = strtol(hex.substring(1).c_str(), NULL, 16);
  uint8_t r = (number >> 16) & 0xFF;
  uint8_t g = (number >> 8) & 0xFF;
  uint8_t b = number & 0xFF;
  return strip.Color(r, g, b);
}

void applyLeds(uint32_t color, int count) {
  if (count < 0) count = 0;
  if (count > NUM_LEDS) count = NUM_LEDS;

  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, (i < count) ? color : 0);
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Init tira
  strip.begin();
  strip.setBrightness(80);
  strip.show(); // apaga

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(300);

  Serial.println("\nConectando WiFi...");
  WiFi.begin(ssid, pass);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    Serial.print(".");
    delay(400);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("NO conectó (timeout). OJO: la red debe ser 2.4GHz.");
    // Indicador: rojo (1 LED) si falla
    applyLeds(strip.Color(255,0,0), 1);
    return;
  }

  Serial.println("WiFi listo!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // HTTPS sin validar certificado (práctica)
  client.setInsecure();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Se desconectó WiFi.");
    delay(2000);
    return;
  }

  HTTPClient http;
  http.setTimeout(20000);

  if (!http.begin(client, URL_STATE)) {
    Serial.println("http.begin() falló");
    delay(2000);
    return;
  }

  int code = http.GET();
  Serial.print("HTTP code = "); Serial.println(code);

  if (code == 200) {
    String body = http.getString();
    Serial.println("Body:");
    Serial.println(body);

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, body);

    if (!err) {
      String colorHex = doc["color"] | "#7682CF";
      int count = doc["count"] | 0;

      applyLeds(hexToColor(colorHex), count);

      Serial.print("Aplicado -> Color: ");
      Serial.print(colorHex);
      Serial.print(" Count: ");
      Serial.println(count);
    } else {
      Serial.println("Error JSON");
    }
  } else {
    Serial.println("GET falló");
  }

  http.end();
  delay(2000);
}