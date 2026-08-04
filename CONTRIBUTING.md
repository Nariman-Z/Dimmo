# Contributing to Dimmo

Thank you for your interest in contributing to **Dimmo**! We welcome contributions, bug reports, feature requests, documentation improvements, and hardware optimizations.

## Code of Conduct

Please treat every member of the community with respect and professional courtesy.

---

## How to Contribute

### 1. Reporting Bugs
- Search existing issues before opening a new one.
- Provide a clear title and description.
- Include details: OS version, micro-controller hardware revision, Flutter SDK version, steps to reproduce, and error logs.

### 2. Suggesting Enhancements
- Open an Issue describing the proposed feature or improvement.
- Provide context on the use case and expected benefit.

### 3. Submitting Code Changes
1. Fork the repository.
2. Create a feature branch off `main`:
   ```bash
   git checkout -b feature/my-feature
   ```
3. Make your changes adhering to existing code conventions.
4. Run validation checks:
   - **Firmware**: Ensure PlatformIO compiles cleanly (`pio run`).
   - **Flutter**: Ensure `flutter analyze` and `flutter test` pass cleanly.
   - **Python Tools**: Run `python3 -m py_compile tools/scripts/esp_simulator.py`.
5. Commit your changes with descriptive commit messages:
   ```bash
   git commit -m "feat(firmware): add zero-cross software debouncing filter"
   ```
6. Push to your fork and submit a Pull Request targeting `main`.

---

## Code Formatting Guidelines

- **C++ / ESP32**: Follow standard C++ Google/LLVM style. Use explicit types, keep functions modular, and comment hardware register interactions.
- **Dart / Flutter**: Adhere to `flutter_lints`. Format with `dart format .`.
- **Python**: Follow PEP 8 guidelines.

---

## Security Policy

Never include private API keys, Wi-Fi credentials, or broker passwords in your pull requests. Review [`SECURITY.md`](SECURITY.md) for details on handling secrets.
