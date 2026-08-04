# Dimmo Hardware Setup & Assembly Guide

Follow this guide to assemble, wire, and calibrate your Dimmo hardware module safely based on the official circuit schematic.

---

## 🔌 Circuit Pin Wiring Table

| Schematic Signal | Component Pin | ESP32 Pin Assignment | Power / Pull-Up |
| ---------------- | ------------- | -------------------- | --------------- |
| **`Zero-Cross`** | 4N25 Collector (Pin 5) | GPIO 17 (or GPIO 34 on PCB) | +5V (10k$\Omega$ `R4` Pull-Up) |
| **`PWM`** | MOC3021 Anode (Pin 1 via `R5` 220$\Omega$) | GPIO 4 (or GPIO 26 on PCB) | 3.3V Logic Output |
| **`ZMPT`** | ZMPT101B OUT | GPIO 36 (VP) / GPIO 35 | 5V Power Supply |
| **`ACS`** | ACS712 OUT | GPIO 39 (VN) / GPIO 32 | 5V Power Supply |
| **`SDA`** | 16x2 LCD SDA | GPIO 21 | 5V Power Supply |
| **`SCL`** | 16x2 LCD SCL | GPIO 22 | 5V Power Supply |

---

## 🎯 Sensor Calibration

1. **ZMPT101B Voltage Calibration**:
   - Measure mains AC voltage using a calibrated multimeter.
   - Adjust the multi-turn trimpot on the ZMPT101B module until the RMS reading on the LCD matches your multimeter reading.
2. **ACS712 Current Calibration**:
   - Ensure zero load current is flowing.
   - Verify the baseline analog reading corresponds to $V_{CC} / 2$ (approx $2.5\text{V}$).
