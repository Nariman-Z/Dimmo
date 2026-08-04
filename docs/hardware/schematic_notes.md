# Dimmo Hardware Schematic & Circuit Notes

This document details the official circuit topology, safety isolation boundaries, and zero-crossing detection mechanics based on the hardware schematic.

---

## ⚡ Safety & Isolation Boundaries

> [!CAUTION]
> **HIGH VOLTAGE WARNING**: The Dimmo hardware operates directly on mains AC voltage (110V - 240V AC). Always ensure mains power is disconnected before touching circuit components.

### Galvanic Isolation Topology
- **Zero-Cross Stage**: Dual **120k$\Omega$** resistors (`R1`, `R2`) + bridge rectifier (`BR`) isolated via **4N25** phototransistor optocoupler (`U1`).
- **TRIAC Gate Stage**: Isolated via **MOC3021** non-zero-crossing optoisolator (`U2`).
- **Power Switching**: **BTA16** TRIAC (`Triac`) protected by dual snubber/filter capacitors (`C1` = 0.05$\mu$F, `C2` = 0.01$\mu$F).
- **Voltage Sensing**: Isolated via ZMPT101B voltage transformer module.
- **Current Sensing**: Isolated via ACS712 Hall-effect sensor module.

---

## 🔄 Zero-Crossing Circuit Notes

- **Input Current Limiting & Rectification**: AC Line (`L`) and Neutral (`N`) pass through `R1` (120k$\Omega$) and `R2` (120k$\Omega$) into full-wave bridge rectifier `BR`. Resistor `R3` (47k$\Omega$) limits current into 4N25 Pin 1 (Anode).
- **Pulse Output (`Zero-Cross`)**: The 4N25 collector (Pin 5) is pulled up to **+5V** via `R4` (10k$\Omega$). When AC voltage passes zero, the 4N25 transistor turns OFF, causing the `Zero-Cross` signal line to pulse HIGH/LOW to trigger the ESP32 hardware interrupt.
- **TRIAC Firing Control (`PWM`)**: The ESP32 MCU outputs a firing pulse on the `PWM` line through `R5` (220$\Omega$) to MOC3021 Pin 1, with visual feedback provided by indicator LED `D1`.
- **TRIAC Gate Drive**: MOC3021 output (Pin 6/Pin 4) fires the BTA16 TRIAC gate via resistor network `R6` (360$\Omega$) and `R7` (470$\Omega$), providing smooth phase-cut AC dimming to the output load (`Out`).
