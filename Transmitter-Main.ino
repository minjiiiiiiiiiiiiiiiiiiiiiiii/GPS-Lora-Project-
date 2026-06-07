#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// =========================
// PATROL UNIT SETTINGS
// =========================
#define CAR_ID "CAR1"   // Change to the car name if you want multiple devices.

// =========================
// WiFi & API SETTINGS
// =========================
const char* WIFI_SSID = "Enter your Wi-Fi credentials";
const char* WIFI_PASSWORD = "Enter your Wi-fi password";
const char* API_URL = "Paste Here your backend API link here";

// =========================
// GPS PINS
// =========================
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// =========================
// LORA PINS
// =========================
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_FREQ 433E6

// =========================
// OLED SETTINGS (128x32)
// =========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C
#define OLED_SDA 21
#define OLED_SCL 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// =========================
// OBJECTS
// =========================
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// =========================
// TIMING & CONNECTION
// =========================
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;

unsigned long popupUntil = 0;
String popupMessage = "BOOT";
bool gpsFixAnnounced = false;

// Connection Management
bool wifiConnected = false;
bool usingLoRa = false;
unsigned long lastWifiCheck = 0;
unsigned long wifiFailStartTime = 0;
const unsigned long wifiCheckInterval = 10000;
const unsigned long WIFI_FAIL_TIMEOUT = 20000; // 20 seconds before switching to LoRa

// =========================
// FUNCTIONS
// =========================
String buildPacket();
void sendLoRa(const String& message);
bool sendToAPI(const String& message);
void drawOLED();
void showPopup(const String& msg, unsigned long durationMs);
bool hasFixApprox();
String gpsStatusText();
void connectToWiFi();
void checkWiFiConnection();
String escapeJson(const String& input);

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 10);
  display.println("PATROL " CAR_ID);
  display.display();
  delay(1200);

  // Connect to WiFi
  connectToWiFi();

  // Initialize LoRa (backup) - MUST MATCH RECEIVER SETTINGS
  SPI.begin();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    showPopup("LORA FAIL", 3000);
  } else {
    // MUST MATCH RECEIVER
    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    Serial.println("LoRa initialized as backup");
  }

  Serial.println("Patrol transmitter started.");
  Serial.println("WiFi primary (API), LoRa secondary (20s failover)");
  showPopup("TX READY", 1500);
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (hasFixApprox() && !gpsFixAnnounced) {
    gpsFixAnnounced = true;
    Serial.println("GPS fix successful");
    showPopup("FIX OK", 2000);
  }

  if (!gps.location.isValid()) {
    gpsFixAnnounced = false;
  }

  // Check WiFi connection periodically
  checkWiFiConnection();

  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    if (gps.location.isValid()) {
      String packet = buildPacket();
      bool sent = false;

      // Try WiFi first (API) if not already in LoRa mode
      if (!usingLoRa && WiFi.status() == WL_CONNECTED) {
        sent = sendToAPI(packet);
        if (sent) {
          Serial.println("Sent via WiFi/API: " + packet);
          showPopup("API SENT", 800);
          // Reset fail timer on successful send
          wifiFailStartTime = 0;
        } else {
          Serial.println("API send failed");
          // Start fail timer if this is the first failure
          if (wifiFailStartTime == 0) {
            wifiFailStartTime = millis();
            Serial.println("WiFi failure detected, will switch to LoRa after 20 seconds");
            showPopup("WiFi ERR", 1200);
          }
        }
      } 
      
      // Check if we should use LoRa (either already using it or WiFi failed for 20 seconds)
      if (!sent && (usingLoRa || (wifiFailStartTime > 0 && (millis() - wifiFailStartTime >= WIFI_FAIL_TIMEOUT)))) {
        if (!usingLoRa) {
          // Switch to LoRa mode after 20 seconds
          usingLoRa = true;
          Serial.println("Switching to LoRa mode (WiFi failed for 20 seconds)");
          showPopup("SWITCH LORA", 2000);
        }
        sendLoRa(packet);
        Serial.println("Sent via LoRa: " + packet);
        showPopup("LORA SENT", 800);
      } else if (!sent && wifiFailStartTime > 0 && (millis() - wifiFailStartTime < WIFI_FAIL_TIMEOUT)) {
        // Still in grace period, don't send anything
        unsigned long remaining = (WIFI_FAIL_TIMEOUT - (millis() - wifiFailStartTime)) / 1000;
        Serial.print("Waiting ");
        Serial.print(remaining);
        Serial.println(" seconds before switching to LoRa");
        showPopup("WAIT " + String(remaining) + "s", 1000);
      }
    } else {
      Serial.println("No valid GPS fix yet.");
      showPopup("WAIT GPS", 1200);
    }
  }

  drawOLED();
}

void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  showPopup("WiFi CONN", 1500);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    showPopup("WiFi OK", 1500);
    wifiConnected = true;
    usingLoRa = false;
    wifiFailStartTime = 0;
  } else {
    Serial.println("\nWiFi connection failed! Starting with LoRa.");
    showPopup("WiFi FAIL", 1500);
    wifiConnected = false;
    usingLoRa = true;
  }
}

void checkWiFiConnection() {
  if (millis() - lastWifiCheck >= wifiCheckInterval) {
    lastWifiCheck = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiConnected) {
        Serial.println("WiFi connection lost!");
        showPopup("WiFi LOST", 1500);
        wifiConnected = false;
        // Start the 20 second timer
        if (wifiFailStartTime == 0 && !usingLoRa) {
          wifiFailStartTime = millis();
          Serial.println("WiFi lost, will switch to LoRa after 20 seconds");
        }
      }
      // Try to reconnect
      WiFi.reconnect();
    } else {
      if (!wifiConnected) {
        wifiConnected = true;
        Serial.println("WiFi reconnected!");
        showPopup("WiFi BACK", 1500);
        // Reset LoRa mode and fail timer
        usingLoRa = false;
        wifiFailStartTime = 0;
      }
    }
  }
}

bool sendToAPI(const String& message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }
  
  // Create JSON payload matching your receiver's format
  String jsonPayload = "{";
  jsonPayload += "\"packet\":\"" + escapeJson(message) + "\",";
  jsonPayload += "\"source\":\"wifi_direct\",";
  jsonPayload += "\"rssi\":0";
  jsonPayload += "}";
  
  Serial.println("Sending JSON to API: " + jsonPayload);
  
  // Create secure client for HTTPS
  WiFiClientSecure client;
  client.setInsecure(); // Skip SSL verification for testing
  
  HTTPClient http;
  http.begin(client, API_URL);
  http.setTimeout(10000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", "close");
  
  int httpResponseCode = http.POST(jsonPayload);
  
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println("Response: " + response);
    http.end();
    return (httpResponseCode == 200);
  } else {
    Serial.print("Error on HTTP request: ");
    Serial.println(httpResponseCode);
    http.end();
    return false;
  }
}

String buildPacket() {
  double lat = gps.location.lat();
  double lng = gps.location.lng();

  double speedKmph = 0.0;
  if (gps.speed.isValid()) {
    speedKmph = gps.speed.kmph();
    if (speedKmph < 3.0) speedKmph = 0.0;
  }

  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  String timestamp = "0";
  if (gps.date.isValid() && gps.time.isValid()) {
    char buf[25];
    snprintf(
      buf, sizeof(buf),
      "%04d-%02d-%02d %02d:%02d:%02d",
      gps.date.year(),
      gps.date.month(),
      gps.date.day(),
      gps.time.hour(),
      gps.time.minute(),
      gps.time.second()
    );
    timestamp = String(buf);
  }

  // 6-field format (MATCHES RECEIVER EXPECTATION):
  // CAR1,lat,lng,speed,sats,timestamp
  String packet = String(CAR_ID) + "," +
                  String(lat, 6) + "," +
                  String(lng, 6) + "," +
                  String(speedKmph, 2) + "," +
                  String(sats) + "," +
                  timestamp;

  return packet;
}

void sendLoRa(const String& message) {
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}

bool hasFixApprox() {
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  return gps.location.isValid() && sats >= 4;
}

String gpsStatusText() {
  if (!gps.location.isValid()) return "NO FIX";
  if (hasFixApprox()) return "FIX OK";
  return "WEAK";
}

void showPopup(const String& msg, unsigned long durationMs) {
  popupMessage = msg;
  popupUntil = millis() + durationMs;
}

String escapeJson(const String& input) {
  String out = "";
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    if (c == '\"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }
  return out;
}

void drawOLED() {
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  double speedKmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  if (speedKmph < 3.0) speedKmph = 0.0;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print(CAR_ID);
  
  // Show connection status
  display.setCursor(32, 0);
  if (usingLoRa) {
    display.print("LoRa");
  } else if (WiFi.status() == WL_CONNECTED) {
    display.print("WiFi");
  } else {
    display.print("FAIL");
  }

  display.setCursor(68, 0);
  display.print("SAT:");
  display.print(sats);

  display.setCursor(108, 0);
  display.print((int)speedKmph);
  display.print("K");

  display.setCursor(0, 10);
  display.print("GPS:");
  display.print(gpsStatusText());

  display.setCursor(0, 20);
  if (millis() < popupUntil) {
    display.print("MSG:");
    display.print(popupMessage);
  } else {
    if (gps.location.isValid()) {
      display.print(gps.location.lat(), 2);
      display.print(",");
      display.print(gps.location.lng(), 2);
    } else {
      display.print("LAT/LNG: --");
    }
  }

  display.display();
}
