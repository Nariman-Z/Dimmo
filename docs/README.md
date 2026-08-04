# Dimmo Documentation Index

Welcome to the official documentation hub for the **Dimmo IoT Smart Dimmer System**.

---

## 📚 Documentation Map

### ⚡ Hardware Specifications & Signal Analysis (`docs/hardware/`)
* **[Hardware Components & BOM](hardware/components.md)**: Detailed bill of materials matching the official circuit schematic.
* **[Schematic & Circuit Notes](hardware/schematic_notes.md)**: Circuit operation, galvanic isolation topology, and optocoupler driving.
* **[Hardware Setup & Wiring Guide](hardware/hardware_setup.md)**: Step-by-step wiring table and sensor calibration.
* **[Oscilloscope Waveforms & Signal Analysis](hardware/waveforms.md)**: Proteus simulations and physical oscilloscope captures (`PC817` vs `4N25` vs `H11AA1`, phase dimming waveforms).
* **[Datasheets & Academic Thesis](hardware/datasheets.md)**: Direct links to component datasheets (`4N25`, `MOC3021`, `BTA16`, `ZMPT101B`, `ACS712`) and the original B.Sc. undergraduate research thesis.

---

### 📡 System Architecture & Protocols (`docs/architecture/`)
* **[High-Level System Architecture](architecture/system_architecture.md)**: System topology, edge-to-cloud telemetry flow, and embedded subsystem boundaries.
* **[MQTT Network & Security Architecture](architecture/mqtt_architecture.md)**: Port `8883` TLS 1.2 transport encryption, authentication, session routing, and CA verification.
* **[MQTT Protocol & Payload Specification](architecture/communication_protocol.md)**: Topic hierarchy (`command`, `telemetry`, `status`, `debug`), Quality of Service (QoS), Last Will & Testament (LWT), and JSON schemas.

---

### 💻 Development & Troubleshooting (`docs/development/`)
* **[Firmware Setup Guide](development/firmware_setup.md)**: PlatformIO C++ environment, build configurations, flashing routines, and `Secrets.h` provision.
* **[Flutter Mobile App Setup Guide](development/flutter_setup.md)**: Flutter SDK prerequisites, pub dependencies, neumorphic UI dashboard, and MQTT service.
* **[Troubleshooting & FAQs](development/troubleshooting.md)**: Solutions for hardware fault modes, dry-run simulation fallbacks, firmware debugging, and mobile state sync.

---

## 📑 Local PDF & Media Assets

* **Component Datasheets**: [`hardware/datasheets/`](../hardware/datasheets/)
* **Academic B.Sc. Thesis**: [`docs/pdf/BSc_Thesis_Nariman_Ziaie.pdf`](pdf/BSc_Thesis_Nariman_Ziaie.pdf) (By Nariman Ziaie)
* **Oscilloscope Images & Waveform Captures**: [`docs/images/`](images/)

