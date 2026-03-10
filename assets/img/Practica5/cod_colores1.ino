#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

// ====== CONFIG WIFI ======
const char* ssid = "Primavera26";
const char* pass = "Ib3r02026pR1m";

// IP de tu compu donde corre Flask
// EJ: "http://172.22.54.156:5000/api/state"
String SERVER_STATE = "http://172.22.54.74:5000/api/state";

// ====== TIRA LED ======
#define LED_PIN 4          // GPIO4 -> DI
#define NUM_LEDS 8         // AJUSTA a tu tira real
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Convierte "#RRGGBB" -> uint32_t
uint32_t hexToColor(const String& hex) {
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

  strip.begin();
  strip.show(); // apaga todo
  strip.setBrightness(80); // ajusta brillo (0-255)

  WiFi.begin(ssid, pass);
  Serial.print("Conectando WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi listo!");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(SERVER_STATE);
    int code = http.GET();

    if (code == 200) {
      String body = http.getString();
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, body);

      if (!err) {
        String colorHex = doc["color"] | "#7682CF";
        int count = doc["count"] | 0;

        uint32_t color = hexToColor(colorHex);
        applyLeds(color, count);

        Serial.print("Color: "); Serial.print(colorHex);
        Serial.print("  Count: "); Serial.println(count);
      } else {
        Serial.println("Error JSON");
      }
    } else {
      Serial.print("HTTP error: ");
      Serial.println(code);
    }

    http.end();
  }

  delay(500); // consulta cada 0.5s
}