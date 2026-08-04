# Security Policy & Guidance

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 3.0.x   | :white_check_mark: |
| < 3.0   | :x:                |

## Reporting a Vulnerability

If you discover a potential security vulnerability within the Dimmo project, please report it responsibly rather than opening a public issue.

### Disclosure Process
1. Email your findings to the repository maintainer or contact privately via GitHub.
2. Include a detailed description of the vulnerability, steps to reproduce, and any proof-of-concept code.
3. Allow up to 48 hours for an initial response acknowledging your report.

---

## Secrets Management & Configuration Guidelines

This project relies on sensitive configurations such as Wi-Fi credentials, MQTT SSL certificates, and broker access tokens. **Never commit private keys or passwords to version control.**

### 1. ESP32 Firmware Credentials
- Copy [`firmware/lib/Secrets/Secrets.h.example`](/firmware/lib/Secrets/Secrets.h.example) to `firmware/lib/Secrets/Secrets.h`.
- Fill in your local Wi-Fi SSID, Wi-Fi password, MQTT broker hostname, port, username, password, and Root CA certificate.
- `Secrets.h` is excluded from version control via [`firmware/.gitignore`](/firmware/.gitignore).

### 2. Python Simulator Credentials
- Copy [`tools/scripts/config.example.json`](/tools/scripts/config.example.json) to `tools/scripts/config.json`.
- Fill in your broker hostname, port, username, password, and device ID.
- `tools/scripts/config.json` is excluded from version control via [`tools/scripts/.gitignore`](/tools/scripts/.gitignore).

### 3. Flutter Application SSL Certificate
- Place your TLS/SSL root certificate or client certificate in `flutter_app/assets/certificates/client.crt`.
- Configure your connection parameters securely in the app login screen.
