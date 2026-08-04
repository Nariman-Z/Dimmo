# Changelog

All notable changes to the Dimmo project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [3.0.0] - 2026-08-04

### Added
- Complete repository audit and GitHub open-source release structure.
- Comprehensive documentation under `docs/architecture/`, `docs/hardware/`, and `docs/development/`.
- Clean separate sub-system documentation (`firmware/README.md`, `flutter_app/README.md`, `tools/README.md`, `hardware/README.md`).
- Python hardware simulator with JSON configuration support (`tools/scripts/esp_simulator.py`).
- Security policy (`SECURITY.md`), contribution guidelines (`CONTRIBUTING.md`), `.editorconfig`.

### Changed
- Migrated Flutter mobile app to `flutter_app/` with package signature `com.narimanz.dimmo`.
- Re-generated high-resolution launcher icons for Android and iOS.
- Cleaned up obsolete asset files and unused dependencies (`permission_handler`).
- Standardized MQTT JSON telemetry schema across ESP32 firmware, Python simulator, and Flutter dashboard.
- Sanitized all hardcoded Wi-Fi and MQTT broker credentials with `.example` templates.

---

## [2.9.0] - 2025-07-15
- Optimized Analog-to-Digital filtering constants for refined electrical metrics (RMS voltage and current).

## [2.7.0] - 2025-06-20
- Introduced companion cross-platform Neumorphic Dashboard Mobile App in Flutter.

## [2.2.0] - 2025-04-12
- Added distributed state tracking and remote command handling via MQTT messaging over TLS 1.2.

## [1.0.0] - 2024-11-05
- Initial release featuring zero-crossing TRIAC firing angle control and true-RMS sensing loops on ESP32.
