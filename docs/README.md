# Dimmo Documentation Index

Welcome to the official documentation hub for the **Dimmo IoT Smart Dimmer System**.

---

## 📚 Documentation Map

### ⚡ Hardware Specifications & Signal Analysis (`docs/hardware/`)
* **[Hardware Components & BOM](hardware/components.md)**: Detailed bill of materials matching the official circuit schematic.
* **[Schematic & Circuit Notes](hardware/schematic_notes.md)**: Circuit operation, galvanic isolation topology, and optocoupler driving.
* **[Hardware Setup & Wiring Guide](hardware/hardware_setup.md)**: Step-by-step wiring table and sensor calibration.
* **[Oscilloscope Waveforms & Signal Analysis](hardware/waveforms.md)**: Proteus simulations and physical oscilloscope captures (Zero-cross comparison `PC817` vs `4N25` vs `H11AA1`, phase dimming at 0%, 25%, 75%, 100%).
* **[Datasheets & Academic Thesis](hardware/datasheets.md)**: Direct links to component datasheets (`4N25`, `MOC3021`, `BTA16`, `ZMPT101B`, `ACS712`) and the original B.Sc. undergraduate research thesis.

---

### 📡 System Architecture & Protocols (`docs/architecture/`)
* **[MQTT Protocol Specification](architecture/communication_protocol.md)**: Topic hierarchy (`command`, `telemetry`, `status`, `debug`), Quality of Service (QoS), Last Will & Testament (LWT), and JSON payload schemas.

---

### 💻 Development & Troubleshooting (`docs/development/`)
* **[Firmware Setup Guide](development/firmware_setup.md)**: PlatformIO environment, building, flashing, and `Secrets.h` setup.
* **[Flutter Mobile App Setup Guide](development/flutter_setup.md)**: Prerequisites, dependencies, neumorphic UI dashboard, and MQTT service.
* **[Troubleshooting & FAQs](development/troubleshooting.md)**: Solutions for common hardware, firmware, dry-run testing fallbacks, mobile app, and simulation issues.

---

## 📑 Local PDF & Media Assets

* **Datasheets Directory**: [`docs/pdf/`](pdf/)
* **Academic B.Sc. Thesis**: [`docs/pdf/BSc_Thesis_Nariman_Ziaie.pdf`](pdf/BSc_Thesis_Nariman_Ziaie.pdf) (By Nariman Ziaie)
* **Oscilloscope Images & Waveform Captures**: [`docs/images/`](images/)
