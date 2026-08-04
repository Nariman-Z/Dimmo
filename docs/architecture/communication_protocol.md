# Dimmo MQTT Communication Protocol Specification

This document defines the complete protocol specification and JSON payload structure synchronized between the ESP32 firmware, Flutter mobile application, and Python simulator tools.

---

## 📡 Topic Hierarchy & Quality of Service (QoS)

| Channel / Intent | Topic Pattern | Direction | QoS | Retained | Purpose |
| ---------------- | ------------- | --------- | --- | -------- | ------- |
| **Command** | `devices/{deviceId}/command` | Client $\rightarrow$ Device | `1` (At Least Once) | `false` | Remote power & brightness control commands |
| **Status** | `devices/{deviceId}/status` | Device $\rightarrow$ Client | `1` (At Least Once) | `true` | Authoritative device availability and state |
| **Telemetry** | `devices/{deviceId}/telemetry` | Device $\rightarrow$ Client | `0` (At Most Once) | `false` | Periodic sensor data stream (every 5 seconds) |
| **Debug Logs** | `devices/{deviceId}/debug` | Device $\rightarrow$ Client | `0` (At Most Once) | `false` | Real-time diagnostic logs |

---

## 📄 Payload Specifications

### 1. Command Payload (`devices/{deviceId}/command`)

Sent by the Flutter Mobile App or Python Simulator to control device state.

```json
{
  "power": true,
  "brightness": 75
}
```

| Key | Type | Description | Valid Range / Values |
| --- | ---- | ----------- | -------------------- |
| `power` | `boolean` | Master power state of the dimmer | `true` (ON), `false` (OFF) |
| `brightness` | `integer` | Firing angle brightness percentage | `0` to `100` |

---

### 2. Status Payload (`devices/{deviceId}/status`)

Published by the ESP32 firmware (with `retain=true`) upon connection, disconnect, or state change.

```json
{
  "state": "online",
  "power": true,
  "brightness": 75
}
```

| Key | Type | Description | Valid Range / Values |
| --- | ---- | ----------- | -------------------- |
| `state` | `string` | Connectivity status | `"online"`, `"offline"` |
| `power` | `boolean` | Current power state | `true`, `false` |
| `brightness` | `integer` | Current brightness percentage | `0` to `100` |

> ℹ️ **Last Will and Testament (LWT)**: The broker automatically dispatches `{"state": "offline", "power": false, "brightness": 0}` if the device unexpectedly loses socket connection.

---

### 3. Telemetry Payload (`devices/{deviceId}/telemetry`)

Published by the device every 5 seconds containing True-RMS electrical readings.

```json
{
  "voltage": 230.1,
  "current": 350.0,
  "power": 80.5,
  "frequency": 50.0,
  "powerConsumption": 12.45,
  "powerFactor": 0.98,
  "brightness": 75,
  "rssi": -60
}
```

| Key | Type | Description | Unit |
| --- | ---- | ----------- | ---- |
| `voltage` | `float` | RMS AC Voltage | Volts (V) |
| `current` | `float` | RMS AC Current | Milliamperes (mA) |
| `power` | `float` | Active AC Power (Wattage) | Watts (W) |
| `frequency` | `float` | Mains AC Frequency | Hertz (Hz) |
| `powerConsumption` | `float` | Accumulated Energy Consumption | Kilowatt-hours (kWh) |
| `powerFactor` | `float` | Power Factor metric | `0.00` to `1.00` |
| `rssi` | `integer` | Wi-Fi Signal Strength | dBm |
| `battery` | `integer` | Battery / Backup Indicator (if applicable) | `%` |

---

## 🔒 Security Configuration

- **Transport**: TLS 1.2 encrypted socket connection over TCP port `8883`.
- **Authentication**: Username and Password digest authentication.
- **Certificate Validation**: Root CA (DigiCert Global Root CA) validation.
