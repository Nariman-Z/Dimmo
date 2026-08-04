# Dimmo Flutter Mobile Application

The official cross-platform mobile application for the Dimmo IoT Smart Dimmer ecosystem (`com.narimanz.dimmo`).

---

## 📋 Requirements

- **Flutter SDK**: `>=3.12.2` (Dart `^3.12.2`)
- **Android**: Target SDK 36 / Compile SDK 37 (Android 8.0+ recommended)
- **iOS**: iOS 12.0 or higher
- **Desktop** (Optional): macOS / Linux GTK / Windows

---

## 🚀 Setup & Execution

1. Change directory to the Flutter application root:
   ```bash
   cd flutter_app
   ```
2. Fetch dependencies:
   ```bash
   flutter pub get
   ```
3. Run code analysis:
   ```bash
   flutter analyze
   ```
4. Execute tests:
   ```bash
   flutter test
   ```
5. Launch the application:
   ```bash
   flutter run
   ```

---

## ⚙️ Features

- **Device Connection**: Secure MQTT authentication screen supporting broker host, port, credentials, and custom Device IDs.
- **MQTT Communication**: Asynchronous publish/subscribe over TLS 1.2 (`8883`).
- **Brightness Control**: Smooth neumorphic slider control with instant response and 300ms command debouncing.
- **Telemetry**: Real-time electrical monitoring dashboard (RMS Voltage, Current, Active Power, Frequency, kWh, Power Factor).
- **Device Status**: Live connectivity monitoring with automatic reconnection logic.
- **Bilingual & Dual Theme**: Persian & English localized UI with Dark and Light mode support.

---

## 🏗️ Architecture & Code Structure

```text
lib/
├── core/
│   ├── config/         # ThemeNotifier, AppTheme, and language state
│   ├── constants/      # Color palettes and design tokens (AppColors)
│   ├── localization/   # Localized string dictionaries (AppTranslations)
│   └── services/       # MQTTService implementation (mqtt_client)
│
├── features/
│   ├── auth/           # Login / Connection configuration screen & widgets
│   └── dashboard/      # Main monitoring and control screen & modals
│
└── main.dart           # App entry point
```

### Architecture Breakdown
- **State Management**: `ChangeNotifier` and `ListenableBuilder` pattern for reactive theme, language, and connection states.
- **Services**: Encapsulated `MQTTService` wrapping `MqttServerClient` with SSL context configuration.
- **Screens**:
  - `LoginPage`: Validates and saves MQTT connection parameters via `SharedPreferences`.
  - `DashboardPage`: Displays real-time telemetry gauges, brightness controls, and settings.
