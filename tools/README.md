# Dimmo Development & Testing Tools

This directory contains development tools, MQTT simulation scripts, official JSON payload schemas, and automated testing utilities for the Dimmo IoT platform.

---

## 📁 Directory Layout

```text
tools/
├── mqtt/
│   ├── schemas/
│   │   ├── command_schema.json     # JSON Schema for incoming commands (brightness & power)
│   │   ├── telemetry_schema.json   # JSON Schema for periodic electrical telemetry
│   │   └── status_schema.json      # JSON Schema for retained online/offline status
│   ├── samples/
│   │   ├── sample_command_on.json  # Sample {"brightness": 100, "power": true}
│   │   ├── sample_command_dim.json # Sample {"brightness": 45, "power": true}
│   │   ├── sample_telemetry.json   # Sample telemetry dataset
│   │   └── sample_status.json      # Sample state dataset
│   └── test_mqtt_payloads.py       # Executable Python schema validator & live test runner
├── scripts/
│   ├── esp_simulator.py           # Python ESP32 hardware simulator
│   ├── config.json                # Local broker config (gitignored)
│   └── config.example.json        # Public config template
└── requirements.txt               # Python dependency specifications
```

---

## 🧪 MQTT Payload Schema Validator (`test_mqtt_payloads.py`)

`test_mqtt_payloads.py` is an automated validation suite that verifies incoming and outgoing MQTT payloads against official Dimmo JSON schemas.

### Key Capabilities
- **Offline Schema Validation**: Validates all JSON samples against official schema definitions without requiring network or broker access.
- **Live Integration Testing**: Connects over TLS 1.2 to your EMQX MQTT broker, subscribes to telemetry, and verifies live device response times.

### Usage Commands

* **Offline Validation (Default)**:
  ```bash
  python3 tools/mqtt/test_mqtt_payloads.py --offline
  ```
  *Output*:
  ```text
  === Running Dimmo MQTT Offline Payload Schema Tests ===
  Testing Command Payloads:
    ✓ sample_command_on.json: {"brightness": 100, "power": true}
    ✓ sample_command_dim.json: {"brightness": 45, "power": true}
  Testing Telemetry Payload:
    ✓ sample_telemetry.json: {"voltage": 230.1, "current": 350.0, "power": 80.5, ...}
  Testing Status Payload:
    ✓ sample_status.json: {"state": "online", "power": true, "brightness": 75}
  🎉 All JSON payload schemas validated successfully!
  ```

* **Live Broker Integration Test**:
  ```bash
  python3 test_mqtt_payloads.py --live --broker HOST --port 8883 --user USER --pass PASS
  ```

---

## 📋 JSON Schema & Payload Reference

### 1. Command Payload (`devices/{deviceId}/command`)
Used by the Flutter app and MQTT clients to set brightness and power state.
* **Schema**: [`tools/mqtt/schemas/command_schema.json`](schemas/command_schema.json)
* **Sample**: [`tools/mqtt/samples/sample_command_on.json`](samples/sample_command_on.json)
```json
{
  "brightness": 100,
  "power": true
}
```

### 2. Telemetry Payload (`devices/{deviceId}/telemetry`)
Published by the ESP32 every 5 seconds containing True-RMS electrical readings.
* **Schema**: [`tools/mqtt/schemas/telemetry_schema.json`](schemas/telemetry_schema.json)
* **Sample**: [`tools/mqtt/samples/sample_telemetry.json`](samples/sample_telemetry.json)
```json
{
  "voltage": 230.1,
  "current": 350.0,
  "power": 80.5,
  "frequency": 50.0,
  "powerConsumption": 12.45,
  "powerFactor": 0.98,
  "brightness": 75,
  "rssi": -45
}
```

### 3. Device Status Payload (`devices/{deviceId}/status`)
Published with `retain=true` upon connection or disconnect (Last Will & Testament).
* **Schema**: [`tools/mqtt/schemas/status_schema.json`](schemas/status_schema.json)
* **Sample**: [`tools/mqtt/samples/sample_status.json`](samples/sample_status.json)
```json
{
  "state": "online",
  "power": true,
  "brightness": 75
}
```

---

## 💻 Hardware Simulator (`esp_simulator.py`)

The ESP32 Hardware Simulator allows testing the Flutter mobile application without requiring physical hardware connected to AC mains.

### Features
- Connects over TLS 1.2 to your MQTT broker.
- Subscribes to `devices/{deviceId}/command`.
- Simulates zero-crossing TRIAC dimming response and state updates.
- Streams live True-RMS telemetry payloads (`voltage`, `current`, `power`, `frequency`, `powerConsumption`, `powerFactor`).

### Quick Start
1. Install Python dependencies:
   ```bash
   pip install -r tools/requirements.txt
   ```
2. Navigate to `tools/scripts/` and create `config.json`:
   ```bash
   cd tools/scripts
   cp config.example.json config.json
   ```
3. Edit `config.json` with your broker credentials.
4. Launch the simulator:
   ```bash
   python3 esp_simulator.py
   ```
