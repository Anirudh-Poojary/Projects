#include <WiFi.h>
#include <FirebaseESP32.h>
#include <TinyGPSPlus.h>

// Wi‑Fi credentials
#define WIFI_SSID "MAHE-MIT"
#define WIFI_PASSWORD "abcd1234"

// Firebase credentials
#define FIREBASE_HOST "real-time-data-44026-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyD3OKeeZjg7-xEJ3dwh9DsjwlvIm-6Ynko"

// Firebase objects
FirebaseData firebaseData;
FirebaseJson json;
FirebaseAuth auth;
FirebaseConfig config;

// GPS objects
TinyGPSPlus gps;
#define gpsSerial Serial2          // GPS on UART2
const int GPS_RX_PIN = 16;        // ESP32 RX2 — connect to GPS TX
const int GPS_TX_PIN = 17;        // ESP32 TX2 — connect to GPS RX

// STM32 UART (UART0) — GPIO1 = TX0, GPIO3 = RX0
// Note: UART0 is the default Serial port on ESP32. Connecting external UART devices to GPIO1/GPIO3
// will share the same hardware UART used by USB-to-Serial bridge. Ensure wiring and usage expectations.
#define stmSerial Serial           // STM32 -> ESP32 using UART0 (GPIO3 RX0); ESP32 TX0 is GPIO1

// Buffers and timing
String stm32Buffer = "";          // accumulate incoming chars from STM32
volatile bool stm32Updated = false;
unsigned long lastSend = 0;
const unsigned long sendInterval = 1000; // ms

void setup() {
  // Initialize Serial (UART0) for STM32 data and optional debug (shared)
  stmSerial.begin(115200); // UART0 on GPIO1 (TX0) & GPIO3 (RX0)

  // Start GPS serial (NEO-6M usually 9600) on UART2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Wi‑Fi connect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  stmSerial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    stmSerial.print(".");
    delay(500);
  }
  stmSerial.println("\nConnected to Wi‑Fi");

  // Configure Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  stmSerial.println("Firebase initialized");
}

void loop() {
  // 1) Feed GPS parser from UART2
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2) Read incoming chars from STM32 on UART0 (Serial)
  while (stmSerial.available() > 0) {
    char c = (char)stmSerial.read();
    // accumulate until newline or buffer limit
    if (c == '\n' || c == '\r') {
      if (stm32Buffer.length() > 0) {
        stm32Buffer.trim();
        stm32Updated = true;
      }
    } else {
      if (stm32Buffer.length() < 200) stm32Buffer += c; // prevent runaway growth
    }
  }

  // 3) Prepare and send to Firebase at interval
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    json.clear();

    // Only set lat/lon when valid; otherwise send nulls or skip keys
    if (gps.location.isValid()) {
      double latitude = gps.location.lat();
      double longitude = gps.location.lng();
      json.set("/LSET_VOL", latitude);
      json.set("/FARM_VOL", longitude);
      stmSerial.print("GPS valid: ");
      stmSerial.print(latitude, 6);
      stmSerial.print(", ");
      stmSerial.println(longitude, 6);
    } else {
      // preserve previous values in Firebase by not setting them, or set as invalid marker
      json.set("/LSET_VOL", "invalid");
      json.set("/FARM_VOL", "invalid");
      stmSerial.println("GPS invalid; not sending numeric coordinates.");
    }

    // STM32 data: send latest received string; if not updated, you may send previous or empty
    if (stm32Updated) {
      json.set("/STM32_DATA", stm32Buffer);
      stmSerial.print("STM32 data to send: ");
      stmSerial.println(stm32Buffer);
      // clear update flag (but keep buffer if you want it repeated)
      stm32Updated = false;
      stm32Buffer = ""; // clear after sending; remove this line if you want to retain last value
    } else {
      json.set("/STM32_DATA", ""); // send empty if no new data; adjust as desired
    }

    // Send combined JSON to Firebase under /Sensor
    if (Firebase.setJSON(firebaseData, "/Sensor", json)) {
      stmSerial.println("Data sent to Firebase");
    } else {
      stmSerial.print("Firebase error: ");
      stmSerial.println(firebaseData.errorReason());
    }
  }

  delay(10);
}
