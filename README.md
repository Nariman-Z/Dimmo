<!-- Gradient Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<!-- Dynamic Header -->
<img height=350 alt="Repo Banner - Dimmo" src="https://capsule-render.vercel.app/api?type=waving&color=0:6E9EAB,100:23395B&height=300&section=header&text=🔥Dimmo&fontSize=50&fontColor=ffffff&animation=fadeIn&fontAlignY=38&desc=IoT-Enabled%20AC%20Power%20Regulation%20with%20Secure%20Remote%20Access&descAlignY=60&descAlign=50"></img>

<!--Title-->
<p align="center">
  <b>TRIAC Firing Angle Control – Closed Loop (v3.0)</b>
  <br> <small> <i>- A precision-controlled AC voltage system with real-time feedback, IoT connectivity, and safety mechanisms -</i> </small> <br>
</p> 

---

## 🔧 Introduction
Microcontroller-based **closed-loop TRIAC control system** for precision AC voltage regulation featuring:
- Real-time feedback with voltage/current monitoring
- Secure IoT connectivity (MQTT over TLS)
- Adaptive power management
- Industrial-grade safety protocols

Ideal for dimming, heating control, and industrial power modulation applications.

---

## ⚙️ Hardware Components
| Component               | Functionality                                                                 |
|-------------------------|-------------------------------------------------------------------------------|
| **ESP32-WROOM**         | Main controller with Wi-Fi, OTA, and TLS support                              |
| **H11AA1**              | Hardware-filtered zero-cross detection                                        |
| **BT139 + MOC3021**     | Isolated power switching with snubber circuit                                 |
| **ZMPT101B**            | AC voltage monitoring (0-250V AC)                                             |
| **ACS712**              | Current sensing (5/20/30A options)                                            |
| **16x2 I2C LCD**        | Real-time system parameter display                                            |

---

## 💻 Software Architecture
**C++ (Arduino-ESP32 Core)** implementation featuring:

### Core Functions
- **Zero-Cross Detection**  
  Interrupt-driven with hardware/software debouncing
- **Adaptive Frequency Handling**  
  Auto-detection of 50Hz/60Hz mains
- **Precision Timing**  
  Microsecond-resolution firing angle control (0°-180°)
- **Real-Time Monitoring**  
  Voltage/current feedback via ZMPT101B and ACS712

### IoT Features
- **Secure MQTT Control**  
  TLS 1.2 encrypted communication (Port 8883)
- **OTA Updates**  
  Encrypted firmware updates via `http://<IP>:8266`
- **Time Synchronization**  
  NTP client for Tehran (UTC+3:30) timestamping
- **Heap Monitoring**  
  Auto-recovery from memory exhaustion

> 🔐 MQTT implemented via [EMQX](https://www.emqx.com/) broker  
> 📚 See [EMQX Documentation](https://docs.emqx.com/) for TLS setup

---

## 🚀 Key Features
| Category         | Features                                                                 |
|------------------|--------------------------------------------------------------------------|
| **Core Control** | Zero-cross detection • 0-180° firing angle • 50/60Hz auto-adapt          |
| **Monitoring**   | Voltage sensing • Current measurement • LCD status display               |
| **Connectivity** | MQTT over TLS • OTA updates                                              |
| **Safety**       | Hardware watchdog • Heap monitor • Opto-isolation • Emergency stop       |
| **Upcoming**     | Enhanced logging • PID regulation • Telnet (Port 23) integration • WebSerial integration  |

---

## 🌐 Remote Access
| Protocol       | Port       | Functionality                              | Security          |
|----------------|------------|--------------------------------------------|-------------------|
| **MQTT**       | 8883       | Secure command/control                    | TLS 1.2 Encryption|
| **OTA**        | 8266       | Firmware updates                          | Password Protected|
| **Telnet**     | 23         | Command-line interface                    | Plaintext         |
| **WebSerial**  | 80         | Browser-based serial monitor              | HTTP              |

---

## 🛡️ Safety Systems
- **Hardware Watchdog** - Automatic reset on system freeze
- **Memory Guardian** - Heap monitoring with auto-recovery
- **Emergency Stop** - Instant shutdown via all interfaces
- **Critical Section Protection** - Interrupt-safe code segments
- **Galvanic Isolation** - Optocoupler-separated AC/DC circuits

---

## 📜 Version History
| Version | Release Date | Key Improvements                                      |
|---------|--------------|-------------------------------------------------------|
| v1.0    | Jan 2025     | Basic TRIAC control                                  |
| v2.0    | Mar 2025     | Improved ZCD accuracy                                |
| v2.2    | Apr 2025     | MQTT remote control                                  |
| v2.3    | May 2025     | TLS encryption                                       |
| **v3.0**| Jul 2025     | OTA updates • NTP sync • Heap recovery • Safety upgrades |

---

## 🎯 Project Roadmap
- [x] Closed-loop voltage regulation
- [x] Secure remote control (MQTT/TLS)
- [x] Over-the-air updates
- [ ] **PID-based power regulation**
- [ ] **Web-based configuration portal**
- [ ] **Energy consumption logging**
- [ ] **Multi-device synchronization**

---

## 📚 License
**MIT License** - See [LICENSE.md](https://github.com/Nariman-Z/Dimmo/blob/main/LICENSE) for full terms.

---

<div align="center">

### Crafted with ❤️ by [Nariman_Z](https://github.com/Nariman-Z)  
🌐 [Personal Website](https://nariman-z.pages.io/)

</div>

<!-- Gradient Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

_Last Updated: July 8, 2025_
