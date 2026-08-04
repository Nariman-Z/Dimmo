# Firmware Development & Flashing Guide

This document covers compiling, configuring, flashing, and debugging the ESP32 Dimmo firmware.

---

## 🛠️ Toolchain Prerequisites

- **VS Code**: Installed with the **PlatformIO IDE** extension.
- **PlatformIO CLI** (Optional): `pip install platformio`.
- **USB Driver**: CP210x or CH340 USB-to-UART bridge driver for your ESP32 board.

---

## ⚙️ Configuration Setup

1. Copy `Secrets.h.example` to `Secrets.h`:
   ```bash
   cd firmware/lib/Secrets
   cp Secrets.h.example Secrets.h
   ```
2. Edit `Secrets.h` to specify:
   - Wi-Fi SSID (`WIFI_SSID`) and Password (`WIFI_PASSWORD`).
   - MQTT Broker Hostname (`MQTT_BROKER`), Port (`8883`), Username, and Password.
   - SSL Root CA Certificate (`CERTIFICATE_AUTHORITY`).

---

## 🚀 Building & Flashing

- **Build**:
  ```bash
  cd firmware
  pio run
  ```
- **Upload via USB Serial**:
  ```bash
  pio run --target upload
  ```
- **Monitor Output**:
  ```bash
  pio device monitor --baud 115200
  ```
