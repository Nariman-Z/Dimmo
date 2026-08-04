# Dimmo Hardware Components Specification (BOM)

Detailed bill of materials and component specifications matching the official Dimmo hardware schematic.

---

## 🛠️ Hardware Bill of Materials (BOM)

| Component | Part Symbol / Label | Purpose & Specification | Quantity |
| --------- | ------------------- | ----------------------- | -------- |
| **Microcontroller** | ESP32 DevKit v1 | Dual-core 240MHz, 4MB Flash, Wi-Fi & Bluetooth (`Zero-Cross` / `PWM`) | 1 |
| **Bridge Rectifier** | `BR` | Full-wave diode bridge rectifier for AC zero-cross sensing | 1 |
| **Zero-Cross Optocoupler** | `U1` (**4N25**) | Phototransistor optocoupler for zero-crossing detection | 1 |
| **TRIAC Driver Opto** | `U2` (**MOC3021**) | Non-zero-crossing optoisolator driver for phase firing | 1 |
| **Power TRIAC** | `Triac` (**BTA16**) | 16A 600V TRIAC for AC power dimming | 1 |
| **Line Input Resistors** | `R1`, `R2` (**120k$\Omega$** each) | High-voltage 1/2W resistors limiting AC Line (L) and Neutral (N) current | 2 |
| **Optocoupler LED Resistor** | `R3` (**47k$\Omega$**) | Series current-limiting resistor for 4N25 LED input (Pin 1) | 1 |
| **Zero-Cross Pull-Up Resistor**| `R4` (**10k$\Omega$**) | Collector pull-up resistor to **+5V** for `Zero-Cross` output | 1 |
| **PWM Input Resistor** | `R5` (**220$\Omega$**) | Series current-limiting resistor for `PWM` trigger input signal | 1 |
| **Gate Drive Resistor 1** | `R6` (**360$\Omega$**) | Series resistor from MOC3021 Pin 6 to gate trigger node | 1 |
| **Gate Drive Resistor 2** | `R7` (**470$\Omega$**) | Gate bias resistor connected to BTA16 MT2 | 1 |
| **Filter Resistor** | `R8` (**360$\Omega$**) | Resistor in snubber filter network across Line output | 1 |
| **Firing Indicator LED** | `D1` (LED) | Red LED for visual TRIAC trigger pulse indication | 1 |
| **Snubber Capacitor 1** | `C1` (**0.05$\mu$F** / 50nF) | Film capacitor for primary RC snubber filter | 1 |
| **Snubber Capacitor 2** | `C2` (**0.01$\mu$F** / 10nF) | Film capacitor for secondary output filter network | 1 |
| **Voltage Sensor Module** | ZMPT101B | Active AC voltage transformer sensor for True-RMS voltage tracking | 1 |
| **Current Sensor Module** | ACS712-05B | Hall-effect linear current sensor for True-RMS current tracking | 1 |
| **Display** | 16x2 I2C LCD | HD44780 LCD with PCF8574 I2C adapter (`0x27`) | 1 |
