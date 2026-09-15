# esp32-sovereign-watch

Firmware for a custom-built ESP32 smartwatch with IMU, ADC, RTC, and a ST7789 display.

## What it is

ESP-IDF/C firmware for a from-scratch smartwatch:

- Drives a ST7789 display with custom bitmap-font rendering (`fontx` / `fontbdf`)
- Reads an ICM42670 IMU for motion sensing
- Reads an MCP3427 ADC
- Keeps time via a DS3231 real-time clock, synced over WiFi with SNTP
- Handles 5 physical buttons
- Connects to WiFi with a configured fallback network

Uses [esp-idf-lib](https://github.com/UncleRus/esp-idf-lib) as a git submodule for peripheral
drivers.

## Stack

- C, ESP-IDF, FreeRTOS, ESP32

## Status

Actively developed — this is the current firmware revision (a duplicate, earlier copy at `esp/watch`
is superseded and not maintained).

## Building

Standard ESP-IDF build:

```bash
idf.py build
idf.py -p <PORT> flash monitor
```
