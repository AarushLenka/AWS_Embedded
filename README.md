# AWS_Embedded

[![project language](https://img.shields.io/badge/language-C%2B%2B-blue)]()
[![status](https://img.shields.io/badge/status-alpha-orange)]()

> ESP32 multisensor telemetry firmware — streams temperature (DS18B20), SpO2 + HR (MAX30105) and ECG (AD8232) and publishes measured data as JSON to AWS IoT Core over MQTT/TLS.

Table of contents
- [What the project does](#what-the-project-does)
- [Why this project is useful](#why-this-project-is-useful)
- [Features](#features)
- [Hardware & wiring](#hardware--wiring)
- [Getting started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Prepare credentials (secrets.h)](#prepare-credentials-secretsh)
  - [Build & flash (Arduino IDE or PlatformIO)](#build--flash-arduino-ide-or-platformio)
  - [Verify operation](#verify-operation)
- [Payload format (example)](#payload-format-example)
- [Where to get help](#where-to-get-help)
- [Maintainers & contributing](#maintainers--contributing)
- [Notes & security](#notes--security)
- [References](#references)

## What the project does
AWS_Embedded is an Arduino/ESP32 firmware reference implementation that samples multiple biomedical/environmental sensors and publishes JSON telemetry to AWS IoT Core using MQTT over TLS. It demonstrates local sensor readout, simple signal buffering (ECG), SpO2/HR calculation using the MAX30105 sensor and an award-winning SpO2 algorithm, and secure device-to-cloud communication.

## Why this project is useful
- Quick reference for integrating sensors and cloud connectivity on ESP32.
- Demonstrates secure MQTT/TLS setup with AWS IoT Core (certificates).
- Provides a compact example of real-time sampling (ECG @ 250 Hz) alongside lower-rate telemetry.
- Useful starting point for prototyping remote monitoring, IoT health projects, or classroom demos.

## Features
- Reads temperature from DS18B20 (OneWire).
- Reads SpO2 and heart rate via MAX30105 and the included spo2 algorithm.
- Samples ECG (AD8232) at 250 Hz and includes a rolling 1-second buffer.
- Publishes telemetry to AWS IoT Core using MQTT over TLS (WiFiClientSecure + PubSubClient).
- Simple JSON payload suitable for ingestion into AWS services (IoT Rules → Lambda / DynamoDB / S3).

## Hardware & wiring
Minimum hardware
- ESP32 development board (e.g., esp32dev)
- MAX30105 (pulse oximeter / heart-rate module)
- DS18B20 (temperature sensor, 1-Wire) + 4.7k pull-up
- AD8232 ECG front-end (or similar analog ECG output)

Relevant pins (as used in `src/src.ino`)
- I2C (MAX30105) — SDA = 21, SCL = 22
- DS18B20 (OneWire) — GPIO 4 (ONE_WIRE_BUS)
- ECG analog input — GPIO 34 (ECG_PIN)
- Power notes — ensure sensors get the correct voltage; many breakout boards accept 3.3V and/or 5V. Verify module datasheets.

Wiring tips
- DS18B20: 3.3V — DATA — GPIO4, with 4.7k pull-up between DATA and VCC.
- MAX30105: SDA → 21, SCL → 22, VIN → 3.3V (or 5V per board spec), GND common.
- AD8232: OUTPUT → ADC pin (GPIO34), follow reference ECG front-end wiring and safe isolation practices.

## Getting started

### Prerequisites
- Arduino IDE (or PlatformIO for VS Code) with ESP32 support installed, or Arduino CLI.
- Libraries (install via Arduino Library Manager or PlatformIO):
  - MAX30105
  - spo2_algorithm (or the algorithm files included in the repository)
  - OneWire
  - DallasTemperature
  - PubSubClient
  - WiFi
  - WiFiClientSecure
- AWS account with IoT Core permissions to create a Thing and certificates.

### Prepare credentials (secrets.h)
The project expects a `src/secrets.h` file (excluded via .gitignore). Create `src/secrets.h` from the template below and fill in your values.

```c++
/* src/secrets.h - TEMPLATE (do NOT commit real credentials) */

#ifndef SECRETS_H
#define SECRETS_H

// Wi-Fi
#define WIFI_SSID "your-ssid"
#define WIFI_PASS "your-wifi-password"

// AWS IoT
#define AWS_IOT_ENDPOINT "your-ats-endpoint.iot.<region>.amazonaws.com"
#define AWS_IOT_PORT 8883
#define AWS_IOT_TOPIC "your/topic"

#define AWS_ROOT_CA "-----BEGIN CERTIFICATE-----\n...root CA...\n-----END CERTIFICATE-----\n"
#define AWS_DEVICE_CERT "-----BEGIN CERTIFICATE-----\n...device cert...\n-----END CERTIFICATE-----\n"
#define AWS_PRIVATE_KEY "-----BEGIN PRIVATE KEY-----\n...private key...\n-----END PRIVATE KEY-----\n"

#endif // SECRETS_H
```

How to get AWS certs (summary)
1. In AWS IoT Core, create a Thing and generate a device certificate (one-time).
2. Attach an IoT policy that allows `iot:Connect`, `iot:Publish`, `iot:Subscribe`, `iot:Receive` to the Thing (or use least-privilege policy).
3. Get the AWS IoT endpoint from the console and paste into `AWS_IOT_ENDPOINT`.
4. Copy the Root CA, device certificate and private key into the respective macros in `secrets.h`.

Important: never commit `secrets.h` into source control. It's present in `.gitignore`.

### Build & flash (Arduino IDE)
1. Open Arduino IDE.
2. Add ESP32 board support (if not already) via the board manager — follow Espressif's instructions.
3. Install required libraries via Library Manager.
4. Copy the repository into your Arduino sketchbook or open `src/src.ino`.
5. Create `src/secrets.h` from the template and fill in credentials.
6. Select the target ESP32 board and the correct COM port.
7. Upload the sketch. Open Serial Monitor at 115200 baud to observe logs.

### Build & flash (PlatformIO)
Create a simple `platformio.ini` (example):

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
  sparkfun/MAX30105
  OneWire
  milesburton/DallasTemperature
  knolleary/PubSubClient
```

- Put `src` folder with `src.ino` in project root (PlatformIO expects `src/`).
- Create `src/secrets.h`.
- Run `pio run -t upload` to build and flash.

### Verify operation
- After boot, the device will connect to Wi‑Fi, sync time (NTP), set up TLS and connect to MQTT.
- Every ~2 seconds it will publish a JSON payload to the configured `AWS_IOT_TOPIC`. Watch the Serial Monitor for messages and the printed JSON.

## Payload format (example)
The firmware publishes a compact JSON object. Example (single publish):

{
  "temperature_c": 25.12,
  "heart_rate": 72,
  "spo2": 98,
  "ecg": [3.294, 3.295, 3.293, ...] // first 10 samples printed
}

Note: heart_rate/spo2 are set to 0 if the respective algorithm indicates invalid data.

## Where to get help
- Open an issue on this repository for bugs, feature requests, or questions.
- For AWS IoT setup questions, see AWS IoT Core documentation and official SDK guides.
- For sensor library issues, refer to the libraries' GitHub pages or Arduino Library Manager pages.

## Maintainers & contributing
Maintainer
- AarushLenka — https://github.com/AarushLenka

Contributing
- Contributions are welcome. Please open issues or PRs.
- For contribution guidelines, see `docs/CONTRIBUTING.md` (if present) or start a discussion via issues.
- Keep contributions focused and testable. Do not include private credentials.

If you want this repo to become more robust, consider adding:
- A proper `LICENSE` file,
- CI (PlatformIO or Arduino CI) for build checks,
- A CONTRIBUTING.md to outline PR and testing expectations.

## Notes & security
- secrets.h contains sensitive data and is intentionally ignored by git. Keep certs and private keys secure.
- TLS certificate rotation and secure storage are recommended for production devices.
- The project is a demonstration/prototype — validate any medical use with qualified professionals and applicable regulations; the sensors and code are not medical-grade.

## References
- Sensor libraries are available via the Arduino Library Manager:
  - MAX30105
  - spo2 algorithm implementations (search "spo2_algorithm")
  - OneWire / DallasTemperature
- AWS IoT Core docs: https://docs.aws.amazon.com/iot/latest/developerguide/what-is-aws-iot.html
