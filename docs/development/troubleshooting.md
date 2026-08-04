# Troubleshooting & FAQs

This document compiles solutions for common hardware, firmware, mobile app, MQTT messaging, and simulation issues across the Dimmo ecosystem.

---

## ⚡ Hardware & Firmware Troubleshooting

### 1. `fatal error: Secrets.h: No such file or directory`
* **Symptom**: PlatformIO compilation fails immediately on line `#include <Secrets.h>`.
* **Cause**: `Secrets.h` is gitignored to protect sensitive Wi-Fi and MQTT credentials.
* **Fix**:
  ```bash
  cd firmware/lib/Secrets
  cp Secrets.h.example Secrets.h
  ```
  Edit `Secrets.h` to supply your Wi-Fi credentials, MQTT broker details, topic definitions, and SSL CA certificate.

---

### 2. Telemetry Shows `230V` / `50Hz` During Dry-Run (No Hardware Connected)
* **Question**: *Why does telemetry show 230V and 50Hz when testing the ESP32 on USB without AC mains connected?*
* **Explanation**: When no physical AC hardware (ZMPT101B voltage transformer / H11AA1 zero-crossing detector) is connected, pin sampling returns `0V` and `0Hz`. The firmware includes safe fallback defaults (`230.0V` / `50.0Hz`) so Flutter and MQTT clients render correctly without throwing divide-by-zero errors.
* **Hardware Behavior**: As soon as physical AC hardware is connected, live measured AC voltage (e.g., `218.4V`) and line frequency (e.g., `50.01Hz`) are published automatically.

---

### 3. TRIAC Load Flickering / No Dimming Response
* **Symptom**: The AC bulb stays fully ON, fully OFF, or flickers erratically.
* **Fix**:
  1. Check zero-crossing interrupt pin wiring: **GPIO 17** (Default DevKit) or **GPIO 34** (PCB layout).
  2. Verify that the H11AA1 optocoupler input is connected to AC mains via current-limiting resistors and that the collector pull-up resistor (10kΩ to 3.3V) is securely wired.
  3. Verify TRIAC gate trigger output pin: **GPIO 4** (Default DevKit) or **GPIO 26** (PCB layout).

---

### 4. 16x2 LCD Display Blank or Displaying Solid Blocks
* **Symptom**: LCD backlight turns on, but no characters or status text appear.
* **Fix**:
  1. Adjust the blue contrast potentiometer on the PCF8574 I2C daughterboard using a small screwdriver until text becomes clear.
  2. Ensure I2C bus wiring: **SDA $\rightarrow$ GPIO 21**, **SCL $\rightarrow$ GPIO 22**.
  3. Verify the I2C address in `main.cpp` (default `0x27`; use an I2C scanner sketch if your module uses `0x3F`).

---

## 📱 Flutter Mobile App & MQTT Troubleshooting

### 1. MQTT Connection Failed / TLS Handshake Timeout
* **Symptom**: Flutter app or ESP32 displays `MQTT Connection Failed: rc=-2` or times out.
* **Fix**:
  1. Ensure outbound TCP port **`8883`** is not blocked by local network firewalls or router settings.
  2. Double-check broker hostname, port (`8883`), username, and password in `Secrets.h` and the Flutter Login screen.
  3. Verify that the Root CA certificate matching your EMQX cluster is supplied in `Secrets.h`.

---

### 2. App Slider Moves but ESP32 Does Not Respond
* **Symptom**: Moving the brightness slider in the mobile app updates the UI, but the ESP32 serial output does not log incoming messages.
* **Fix**:
  1. Verify topic alignment: The Flutter app publishes to `devices/{deviceId}/command` (e.g. `devices/dev/command`).
  2. Ensure the JSON command payload format matches:
     ```json
     {
       "brightness": 75,
       "power": true
     }
     ```
  3. Ensure the ESP32 and Flutter app are configured with matching `deviceId` values (e.g. `"dev"`).

---

## 🧪 Testing & Simulation Tools

### 1. Running Offline Schema Validation
Validate all JSON command, telemetry, and status payload schemas without requiring an MQTT broker:
```bash
python3 tools/mqtt/test_mqtt_payloads.py --offline
```

### 2. Running Live MQTT Integration Tests
Publish test command payloads and assert live telemetry responses over TLS:
```bash
python3 tools/mqtt/test_mqtt_payloads.py --live --broker HOST --port 8883 --user USER --pass PASS
```

### 3. Running the ESP32 Python Hardware Simulator (`esp_simulator.py`)
Simulate the ESP32 firmware on a desktop computer for Flutter app development:
```bash
cd tools/scripts
cp config.example.json config.json
# Edit config.json with your broker credentials
python3 esp_simulator.py
```
