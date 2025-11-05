#  Smart Assistive Stick for Visually Impaired

This project implements a **Smart Assistive Stick** that uses **STM32**, **ESP32**, **GPS**, and **Firebase** to help visually impaired users navigate safely while providing real-time location tracking and sensor data logging.

---

##  Overview

The system combines two powerful microcontrollers:

- **STM32** — handles sensor interfacing (e.g., ultrasonic, PIR, buzzers)
- **ESP32** — manages **Wi-Fi**, **Firebase data upload**, and **GPS tracking**

The STM32 continuously collects environment and obstacle data, which is then sent to the ESP32 via UART.  
The ESP32 reads this data, combines it with GPS coordinates, and uploads everything to **Firebase Realtime Database** for remote monitoring.

---

##  Hardware Components

| Component | Purpose |
|------------|----------|
| **ESP32** | Wi-Fi, Firebase communication, GPS interface |
| **STM32 (e.g., STM32F103C8T6)** | Sensor data acquisition, control logic |
| **NEO-6M GPS Module** | Provides latitude and longitude data |
| **Ultrasonic / PIR Sensors** | Detect obstacles |
| **Buzzer / Vibrating Motor** | Alerts the user about nearby obstacles |
| **Firebase Realtime Database** | Cloud platform to store live sensor and GPS data |

---

## 🔗 System Architecture


- STM32 reads sensor data and sends it as a serial string.
- ESP32 receives it through UART0.
- ESP32 reads GPS data via UART2.
- ESP32 pushes combined data (sensor + GPS) to Firebase every second.

---

##  Key Functionalities

###  GPS Integration
- Uses the **TinyGPSPlus** library on ESP32.
- Reads coordinates (latitude and longitude) from the NEO-6M GPS module.
- Validates GPS fix before sending to Firebase.

###  STM32 ↔ ESP32 Communication
- UART0 is used for serial communication.
- STM32 transmits processed sensor readings (e.g., obstacle distance, alert status).
- ESP32 receives and adds it to the Firebase JSON payload.

###  Firebase Upload
- Firebase Realtime Database stores:
  - GPS latitude (`/LSET_VOL`)
  - GPS longitude (`/FARM_VOL`)
  - STM32 sensor data (`/STM32_DATA`)

---

##  Software Setup

### 1. **Arduino IDE Configuration**
- Select **ESP32 Dev Module** board.
- Install required libraries:
  - `WiFi.h`
  - `FirebaseESP32.h`
  - `TinyGPSPlus.h`

### 2. **Firebase Setup**
- Create a **Firebase Realtime Database** project.
- Copy your database URL (e.g., `https://your-project-id.firebaseio.com/`)
- Get your **Web API Key** from Firebase console.
- Replace them in the ESP32 code:
  ```cpp
  #define FIREBASE_HOST "your-db-url"
  #define FIREBASE_AUTH "your-api-key"



