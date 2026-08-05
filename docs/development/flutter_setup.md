# Flutter App Development Guide

This guide covers setting up, building, running, and testing the Dimmo Flutter mobile app (`com.narimanz.dimmo`).

---

## 📋 Prerequisites

- **Flutter SDK**: `>=3.12.2` (Dart `^3.12.2`). Verify with `flutter --version`.
- **Android Studio / Xcode**: Configured for Android SDK 36/37 or iOS 12.0+.

---

## 🚀 Setup Steps

1. Navigate to the app directory:
   ```bash
   cd flutter_app
   ```
2. Copy the example SSL certificate configuration:
   ```bash
   cp assets/certificates/client.crt.example assets/certificates/client.crt
   ```
3. Install dependencies:
   ```bash
   flutter pub get
   ```
3. Run static code analysis:
   ```bash
   flutter analyze
   ```
4. Execute unit & widget tests:
   ```bash
   flutter test
   ```
5. Launch the application:
   ```bash
   flutter run
   ```
