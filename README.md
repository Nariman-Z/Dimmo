<!-- Gradient Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<!-- Dynamic Header -->
<img height=350 alt="Repo Banner - Dimmo" src="https://capsule-render.vercel.app/api?type=waving&color=0:6E9EAB,100:23395B&height=300&section=header&text=🔥Dimmo&fontSize=50&fontColor=ffffff&animation=fadeIn&fontAlignY=38&desc=IoT-Enabled%20AC%20Power%20Regulation%20with%20Secure%20Remote%20Access&descAlignY=60&descAlign=50"></img>

<!--Title-->
<p align="center">
  <b>TRIAC Firing Angle Control – Closed Loop Ecosystem (v3.0)</b>
  <br> <small> <i>- A precision-engineered AC power modulation system featuring real-time telemetry, secure IoT networking, and hardware-level safety mechanisms -</i> </small> <br>
</p> 

---

## 📌 Project Status

> [!NOTE]
> **Dimmo IoT Smart Dimmer Ecosystem (v3.0.0)**  
> This repository contains the complete open-source Dimmo ecosystem, including ESP32 hardware firmware, cross-platform Flutter mobile application, Python hardware simulation suite, PCB schematics, and comprehensive architectural documentation.

---

## 🎓 Academic Project & B.Sc. Thesis

Dimmo was originally developed as an Electrical Engineering Bachelor's Degree Graduation Project by **Nariman Ziaie**.

📄 **[Download / View Official B.Sc. Thesis (PDF)](docs/pdf/BSc_Thesis_Nariman_Ziaie.pdf)**  
*Contains complete undergraduate research findings, phase-angle dimming mathematical models, zero-crossing detection circuit topology, and IoT system integration.*

---

## 🔧 Introduction
Project Dimmo is a high-performance, microcontroller-driven **closed-loop TRIAC control system** engineered for deterministic AC voltage regulation. The platform seamlessly unifies real-time edge telemetry with encrypted remote cloud communication, featuring:
- **True RMS Edge Feedback:** High-fidelity, continuous AC mains voltage and current telemetry.
- **Hardened IoT Architecture:** Secure remote orchestration over network boundaries via MQTT encapsulated in TLS 1.2.
- **Deterministic Power Management:** Dynamic adjustment to load profiles and environmental fluctuations spanning $0^\circ \text{ to } 180^\circ$ phase bounds.
- **Fail-Safe Safety Topologies:** Layered hardware and software safeguards protecting against transient faults and buffer overflow conditions.
- **Cross-Platform Mobile Integration:** Real-time mobile dashboard with neumorphic UI and multilingual localization built with Flutter.

Ideally suited for industrial heating elements, precision lighting dimming systems, and inductive motor speed modulation.

---

## ⚙️ Hardware Components
| Component               | Functional Overview                                                           |
|-------------------------|-------------------------------------------------------------------------------|
| **ESP32-WROOM**         | Dual-core compute engine handling asynchronous control loops, telemetry, and TLS execution. |
| **H11AA1**              | Dual-optoisolated zero-cross detection circuit utilizing hardware RC noise filtration. |
| **BT139 + MOC3021**     | Snubber-protected, optically isolated high-power AC semiconductor switching node.    |
| **ZMPT101B**            | Precision voltage transformer module for analog true RMS mains monitoring (0-250V AC). |
| **ACS712**              | Hall-effect linear current sensor for line-load telemetry tracking.             |
| **16x2 I2C LCD**        | Low-latency visual instrumentation panel displaying systemic edge parameters.      |

---

## 💻 Software Architecture
Implemented via a high-efficiency **C++ (PlatformIO / Arduino-ESP32 Core)** framework alongside a companion **Flutter** mobile client and **Python** hardware simulator:

### Core Execution Modules
- **Hardware-Driven Zero-Cross Interception**  
  Interrupt-driven (ISR) detection engine coupled with dynamic software debouncing algorithms to neutralize harmonic line noise.
- **Adaptive Frequency Calibration**  
  Auto-sensing logic capable of hot-swapping timing parameters between 50Hz and 60Hz mains environments.
- **Microsecond-Resolution Pulse Timing**  
  Hardware timer group allocation providing fine-grained firing angle calibration spanning $0^\circ \text{ to } 180^\circ$ phase bounds.
- **Non-Blocking Telemetry Pipelines**  
  Concurrent analog sampling arrays processing raw inputs from the ZMPT101B and ACS712 sensors into reliable Root-Mean-Square (RMS) metrics.

### Distributed Network Engine
- **Encrypted Transport Layer**  
  TLS 1.2 authenticated session layer mapping command-and-control operations safely over port `8883`.
- **Protected OTA Deployments**  
  Password-authenticated, dual-partition Over-The-Air firmware updater accessible over local management interfaces.
- **Network Time Synchronization**  
  NTP client abstraction layers ensuring sub-millisecond timestamp consistency for edge logging.
- **Runtime Integrity & Heap Sanity**  
  Continuous memory pool inspection with proactive stack boundaries to clear structural leakage before reaching buffer starvation thresholds.

### Mobile & Simulator Ecosystem
- **Neumorphic Flutter App**  
  Cross-platform dashboard supporting real-time control, metrics visualization, status indicators, and multilingual support.
- **Python Hardware Simulator**  
  Complete offline testing suite mirroring the ESP32 MQTT JSON protocol for rapid development without physical hardware.

> 🔐 Edge-to-Cloud telemetry transport routes over an enterprise-hardened [EMQX](https://www.emqx.com/) broker engine.  
> 📚 See the official [EMQX Clustering & Security Documentation](https://docs.emqx.com/) for TLS certificate configuration.

---

## 🚀 Key Features
| Operational Vector | Functional Characteristics |
|------------------|--------------------------------------------------------------------------|
| **Phase Modulation** | Deterministic ZC polling • Microsecond firing precision • Dual-frequency normalization |
| **Instrumentation**| Dynamic Volt/Amp conversion engines • True-RMS estimation • Asynchronous LCD raster updates |
| **Networking**   | Asynchronous MQTT clients • TLS encrypted backhaul • Password-gated OTA partitions |
| **Mobile App**   | Cross-platform Flutter dashboard • Neumorphic design • Real-time telemetry charts |
| **Simulation**   | Python hardware simulator • Full MQTT JSON protocol emulation • Configurable test fixtures |
| **Resilience**   | Hardware watchdog timers • Monitored heap blocks • Galvanic signal decoupling • Panic-stop routines |

---

## 🌐 Remote Access Topography
| Sub-System | Binding Port / Bus | Operational Scope | Security Paradigm |
| :--- | :--- | :--- | :--- |
| **MQTT over TLS** | `8883` | Remote Command, Telemetry, and Device Control | TLS 1.2 Encryption & Digest Authentication |
| **Arduino OTA** | `8266` | Over-The-Air Firmware Image Flashing | Password Protected & Reboot Validation |
| **NTP Client** | `123` (UDP) | Real-Time Clock Synchronization | Public NTP Time Pool |
| **USB Serial** | 115200 Baud | High-Speed Instrumentation & Logging | Direct Hardware UART |

---

## 🛡️ Defect Isolation & Safety Controls
- **Independent Hardware Watchdog (WDT):** Automates compute recovery by forcing hard reboots upon execution stalling.
- **Memory Boundary Defense:** Constantly monitors runtime allocation fragmentation, executing soft-resets prior to fatal heap exhaustion.
- **Asynchronous Emergency Kill-Switch:** Instantaneous absolute power decoupling available via all local and networked vectors.
- **Atomic Critical Path Protections:** Isolate execution counters within memory-safe barriers to avoid variable corruption during rapid ISR fires.
- **Galvanic Boundary Isolation:** Optocoupler arrays decouple low-voltage CMOS logic domains from high-voltage AC execution zones.

---

## 🏗️ Architecture & Repository Structure

```text
Flutter Mobile App (flutter_app/)
        |
        |  (Encrypted MQTT over TLS 1.2 / Port 8883)
        v
    MQTT Broker (EMQX) <---> Python Simulator (tools/)
        ^
        |  (Telemetry & Command JSON Sub/Pub)
        |
  ESP32 Firmware (firmware/)
        |
        |  (Zero-Cross ISR & Phase Firing)
        v
  TRIAC + Sensors (hardware/)
  (BT139 + MOC3021 / ZMPT101B / ACS712 / H11AA1)
```

```text
Dimmo/
├── docs/               # System, MQTT, hardware, and development documentation
├── firmware/           # ESP32 PlatformIO C++ source code & dependencies
├── flutter_app/        # Cross-platform Flutter mobile application
├── hardware/           # Hardware schematics, PCB layouts, and component datasheets
├── tools/              # Python hardware simulator and MQTT development tools
├── LICENSE             # MIT License
├── CHANGELOG.md        # Release version ledger
├── CONTRIBUTING.md     # Development and submission guidelines
├── DISCLAIMER.md       # High-voltage AC safety warning & disclaimer
└── SECURITY.md        # Vulnerability disclosure & secret management guide
```

- [`firmware/`](firmware): PlatformIO ESP32 source code, zero-cross interrupts, and TLS MQTT client.
- [`flutter_app/`](flutter_app): Flutter mobile dashboard application with neumorphic UI and localization.
- [`tools/`](tools): Python-based hardware simulator for local testing without physical hardware.
- [`docs/`](docs): In-depth system architecture, communication protocols, and setup guides.
- [`hardware/`](hardware): Schematics, pin mappings, and board specifications.

---

## 📜 Version Ledger
| Generation | Deployment Context | Notable Milestones & Iterations |
|----------|---------------|----------------------------------------------------------------------------------|
| v1.0     | Nov 2024      | Foundational phase control setup; validated basic current & voltage tracking logic.|
| v2.0     | Apr 2025      | Restructured Zero-Cross ISR handling to optimize phase interception performance. |
| v2.1     | Apr 2025      | Refined phase-angle mathematical models to eliminate phase edge instability.   |
| v2.2     | Apr 2025      | Introduced distributed state tracking via MQTT messaging primitives over TLS 1.2.|
| v2.3     | May 2025      | Patched edge-case configuration state locks.                                   |
| v2.4     | May 2025      | Hardened network vectors via TLS payload wrapping.                             |
| v2.5     | May 2025      | Provisioned secure OTA flash memory layout structures.                           |
| v2.6     | Jun 2025      | Implemented NTP clock sync for deterministic edge telemetry logging.            |
| v2.7     | Jun 2025      | Rolled out companion Cross-Platform Neumorphic Dashboard Mobile App (Flutter).   |
| v2.8     | Jul 2025      | Added proactive memory guardian engines to prevent stack crashes.            |
| v2.9     | Jul 2025      | Optimized Analog-to-Digital filtering constants for refined electrical metrics.|
| **v3.0** | **Aug 2026**  | **Complete repository audit, GitHub open-source release, Python simulator suite, standardized MQTT JSON schema, and MIT license migration.** |

---

## 🛠️ Development Setup

Quick links to setup instructions for each component:

- ⚡ **[Firmware Setup Guide](docs/development/firmware_setup.md)**: PlatformIO environment, dependencies, flashing, and configuration.
- 📱 **[Flutter Setup Guide](docs/development/flutter_setup.md)**: Flutter SDK prerequisites, pub dependencies, build & run instructions.
- 🧪 **[Tools & Simulator Guide](docs/development/troubleshooting.md)**: Python MQTT hardware simulator setup and testing workflows.
- 🎓 **[Academic B.Sc. Thesis (PDF)](docs/pdf/BSc_Thesis_Nariman_Ziaie.pdf)**: Complete undergraduate research paper by Nariman Ziaie.
- 📑 **[Datasheets & Waveforms](docs/hardware/datasheets.md)**: Component datasheets, oscilloscope signal captures, and circuit analysis.
- 🔐 **[Secrets & Configuration Setup](SECURITY.md#secrets-management--configuration-guidelines)**: Generating local configuration files (`Secrets.h` and `config.json`).

---

## 🤝 Contributing

Contributions are welcome and greatly appreciated! Whether you're fixing bugs, improving documentation, optimizing the control algorithms, or proposing new features, your help makes **Dimmo** better.

### Ways to Contribute
- 🐛 Report bugs and unexpected behavior
- 💡 Suggest new features or architectural improvements
- 🔧 Submit bug fixes and performance optimizations
- 📚 Improve documentation and examples
- 🧪 Test the project on different hardware configurations
- 🔒 Review security and safety mechanisms

### Development Workflow
1. Fork the repository.
2. Create a new branch:

```bash
git checkout -b feature/my-awesome-feature
```

3. Commit your changes:

```bash
git commit -m "Add: my awesome feature"
```

4. Push to your fork:

```bash
git push origin feature/my-awesome-feature
```

5. Open a Pull Request describing your changes and their motivation.

### Contribution Guidelines

- Keep code style consistent with the existing codebase.
- Document new features and configuration options.
- Test changes thoroughly on hardware whenever possible.
- Keep commits focused and descriptive.
- Discuss large architectural changes by opening an Issue first.

### Good First Contributions

- PID tuning improvements
- Additional sensor support
- Web configuration dashboard
- Energy consumption logging
- Performance optimizations and bug fixes
- Documentation enhancements

Thank you for helping make **Dimmo** a more robust and reliable open-source AC power control platform. 🤝

---

## ⚠️ Disclaimer

Dimmo is an educational and research project.

Working with mains electricity can be dangerous. The authors and contributors are not responsible for damage, injury, or losses caused by modification, assembly, or usage of this project.

Proper electrical safety practices must be followed. For detailed safety guidance and terms, please see [DISCLAIMER.md](DISCLAIMER.md).

---

## 📚 License
Licensed under the terms of the **MIT License**. For explicit permissions and warranty declarations, review the companion [LICENSE](LICENSE) document asset.

---

<div align="center">

### Made with ❤️ by [Nariman_Z](https://github.com/nariman-z)  
🌐 [Personal Website](https://nariman-z.pages.dev/)

</div>

<!-- Gradient Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

_Last Update: August 4, 2026_