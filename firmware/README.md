# Dimmo ESP32 Firmware

The official firmware for the Dimmo IoT Smart Dimmer system, engineered in C++ for the ESP32 using the PlatformIO framework.

---

## 📋 Required Hardware

- **Microcontroller**: ESP32-WROOM-32 / ESP32 DevKit v1
- **Zero-Cross Detector**: H11AA1 optocoupler with AC input rectifier & noise filter
- **TRIAC Module**: BTA16-600B TRIAC driven by MOC3021 optocoupler
- **Voltage Sensor**: ZMPT101B active AC voltage transformer module
- **Current Sensor**: ACS712-05B Hall-effect current sensor module
- **Display**: 16x2 I2C LCD Display (PCF8574 I2C adapter at address `0x27`)

---

## 📌 Pin Mapping

| Peripheral | ESP32 DevKit Pin (Default) | PCB Alternate Pin | Function Description |
| ---------- | -------------------------- | ----------------- | -------------------- |
| **Zero-Cross Input** | **GPIO 17** | GPIO 34 | Hardware interrupt input for AC zero-crossing detection |
| **TRIAC Gate Drive** | **GPIO 4** | GPIO 26 | Output pin to drive MOC3021 optocoupler TRIAC gate |
| **ZMPT101B Voltage** | **GPIO 36** (VP) | GPIO 35 | Analog input for true-RMS voltage sampling |
| **ACS712 Current** | **GPIO 39** (VN) | GPIO 32 | Analog input for true-RMS current sampling |
| **I2C SDA (LCD)** | **GPIO 21** | GPIO 21 | I2C Data line for 16x2 LCD display |
| **I2C SCL (LCD)** | **GPIO 22** | GPIO 22 | I2C Clock line for 16x2 LCD display |

---

## 🏗️ Firmware Directory Layout

The firmware logic is organized between `src/` (main execution script) and `lib/Secrets/` (private credentials header):

```text
firmware/
├── lib/
│   └── Secrets/                  --> Private network & security credentials
│       ├── Secrets.h             --> User-configured header (gitignored)
│       └── Secrets.h.example     --> Public configuration template
├── src/
│   └── main.cpp                  --> Main firmware application (Zero-cross ISR, TRIAC control, RMS sampling, TLS MQTT, OTA)
├── README.md                     --> Subsystem documentation
└── platformio.ini                --> PlatformIO environment configuration & library dependencies
```

---

## 💻 PlatformIO Setup & Build

### Prerequisites
- Install [VS Code](https://code.visualstudio.com/) with the [PlatformIO IDE extension](https://platformio.org/platformio-ide).
- Alternatively, install PlatformIO CLI: `pip install platformio`.

### Configuration Steps (`Secrets.h`)
To protect sensitive credentials from being committed into public git repositories, all credentials, hostnames, passwords, and SSL certificates are isolated inside `lib/Secrets/Secrets.h`.

1. Navigate to `firmware/lib/Secrets/` and copy `Secrets.h.example` to create `Secrets.h`:
   ```bash
   cd firmware/lib/Secrets
   cp Secrets.h.example Secrets.h
   ```

2. Edit `Secrets.h` to supply your Wi-Fi credentials, MQTT broker details, topic definitions, and SSL/TLS CA certificate:
   ```cpp
   #ifndef SECRETS_H
   #define SECRETS_H

   // ==================== Local Wireless Network Credentials ====================
   #define WIFI_SSID         "YOUR_WIFI_NETWORK_NAME"
   #define WIFI_PASSWORD     "YOUR_WIFI_WPA2_PASSPHRASE"

   // ==================== Over-The-Air Deployment Variables ====================
   #define OTA_HOSTNAME      "Dimmo-Edge-Node-v3"
   #define OTA_PASSWORD      "your_encrypted_ota_flash_phrase"
   #define OTA_PORT          8266

   // ==================== Encrypted MQTT Broker Profiles ====================
   #define MQTT_BROKER       "your-broker-address.com"  // Hostname or IPv4 Address
   #define MQTT_PORT         8883                       // TLS Encrypted Port
   #define MQTT_USERNAME     "your_mqtt_client_user"
   #define MQTT_PASSWORD     "your_secure_broker_token"

   // Message Broker Topic Definitions
   #define MQTT_TOPIC        "devices/dev/command"
   #define TELEMETRY_TOPIC   "devices/dev/telemetry"
   #define STATUS_TOPIC      "devices/dev/status"
   #define DEBUG_TOPIC       "devices/dev/debug"

   // ==================== TLS 1.2 Certificate Authority Vector ==================
   const char* CERTIFICATE_AUTHORITY = R"EOF(
   -----BEGIN CERTIFICATE-----
   YOUR_ROOT_CA_CERTIFICATE_HERE
   -----END CERTIFICATE-----
   )EOF";

   #endif // SECRETS_H
   ```

### Build & Upload Commands

- **Compile Firmware**:
  ```bash
  pio run
  ```
- **Upload via USB Serial**:
  ```bash
  pio run --target upload
  ```
- **Open Serial Monitor (115200 baud)**:
  ```bash
  pio device monitor
  ```
- **Over-The-Air (OTA) Upload**:
  Uncomment the `upload_protocol = espota` section in `platformio.ini`, configure `upload_port` with your ESP32's local IP address, then execute:
  ```bash
  pio run --target upload
  ```

---

## 📡 MQTT Topics & Payloads

### 1. Command Topic (`devices/{deviceId}/command`)
Subscribed by the ESP32 to receive power and dimming commands from the Flutter app or MQTT clients.

```json
{
  "brightness": 75,
  "power": true
}
```

### 2. Device Status Topic (`devices/{deviceId}/status`)
Published by the ESP32 (with `retain=true`) on connection/state changes and via LWT on disconnect.

```json
{
  "state": "online",
  "power": true,
  "brightness": 75
}
```

### 3. Telemetry Topic (`devices/{deviceId}/telemetry`)
Published periodically by the ESP32 every 5 seconds containing True-RMS electrical readings.

```json
{
  "voltage": 230.0,
  "current": 350.0,
  "power": 80.5,
  "frequency": 50.0,
  "powerConsumption": 12.45,
  "powerFactor": 0.98,
  "brightness": 75,
  "rssi": -45
}
```

---

## 🔧 Troubleshooting & References

- **Compilation error missing `Secrets.h`**: Ensure `Secrets.h` was copied from `Secrets.h.example` inside `firmware/lib/Secrets/`.
- **Display not working**: Verify I2C address (`0x27` or `0x3F`) using an I2C scanner sketch and adjust contrast potentiometer on the PCF8574 adapter.
- **Erratic dimming / flicker**: Check ZC input connection on GPIO 17 (or GPIO 34 on PCB) and ensure zero-cross optocoupler circuit is powered correctly.
- **PlatformIO Documentation**: [Library Dependency Finder (LDF) Guide](https://docs.platformio.org/page/librarymanager/ldf.html) | [platformio.ini Specification](https://docs.platformio.org/page/projectconf/index.html)