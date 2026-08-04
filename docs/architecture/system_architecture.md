# Dimmo System Architecture

This document describes the high-level system design and architectural components of the Dimmo IoT Smart Dimmer ecosystem.

---

## 🏛 System Overview Diagram

```text
[ Mobile App (Flutter) ] <---> [ MQTT Broker (TLS 1.2) ] <---> [ ESP32 Edge Device ]
                                                                      |
                                                          +-----------+-----------+
                                                          |                       |
                                                   [ Zero-Cross ISR ]     [ RMS Sampling ]
                                                          |                       |
                                                          v                       v
                                                  [ TRIAC Phase Control ] [ ZMPT101B / ACS712 ]
```

---

## 🧩 Core Sub-Systems

### 1. ESP32 Microcontroller Core
- **Interrupt Service Routine (ISR)**: Intercepts zero-crossing rising edges from H11AA1 optocoupler.
- **Hardware Timer Firing Loop**: Delays TRIAC firing gate pulse (GPIO 26) proportionately to requested brightness (0° to 180° phase angle).
- **True RMS Calculation Engine**: Samples AC mains voltage (ZMPT101B) and load current (ACS712) continuously over full 20ms (50Hz) / 16.6ms (60Hz) cycles.

### 2. MQTT Encrypted Backhaul
- Encrypted TLS 1.2 transport over port 8883.
- Subscribes to command topics and streams real-time status and telemetry JSON payloads.

### 3. Flutter Cross-Platform Client
- Cross-platform mobile app managing connection state, user authentication credentials, and real-time visualization.
- Reactive UI architecture with dark/light themes and localized translation support.
