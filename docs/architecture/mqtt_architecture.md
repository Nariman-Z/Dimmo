# Dimmo MQTT Network Architecture

This document describes the MQTT network architecture, connection security, topic routing, and session management used in the Dimmo IoT platform.

---

## 🔒 Security Architecture

- **Transport Encryption**: Mapped to Port `8883` using TLS 1.2.
- **Client Authentication**: Digest authentication via username & password.
- **Session Identification**: Unique client session IDs generated per node (`{deviceId}_dimmo_{timestamp}`).
- **Certificate Verification**: Server root CA verification using DigiCert Global Root CA.

---

## 📡 Message Routing Topology

```text
Publishers                               Broker (EMQX TLS)                             Subscribers
==========                               =================                             ===========
Flutter App      -- (devices/{id}/command)  --> [ Command Queue ]  -- (devices/{id}/command)  --> ESP32 Device
ESP32 Device     -- (devices/{id}/status)   --> [ Retained State]  -- (devices/{id}/status)   --> Flutter App
ESP32 Device     -- (devices/{id}/telemetry)--> [ Sensor Stream ]  -- (devices/{id}/telemetry)--> Flutter App
```

---

## 🔄 Session Lifecycle

1. **Connection Setup**: Client connects over TLS port 8883 and dispatches credentials.
2. **Will Registration**: Last Will and Testament configured on `devices/{deviceId}/status` with payload `{"state": "offline", "power": false, "brightness": 0}`.
3. **Subscribed Channels**: Client subscribes to command topics.
4. **Heartbeat & Keep-Alive**: Ping packets sent every 60 seconds to maintain active socket connections.
