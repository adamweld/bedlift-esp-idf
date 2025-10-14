# CLAUDE.md

## Hardware Architecture

The dev board is a Adafruit ESP32-S3 Reverse TFT

The builtin display is an ST7789

## Libraries
Use only these libraries and components by default, ask before using anything additional

ESP-IDF standard libraries
LovyanGFX
LVGL

## Development Workflow

- User handles building and flashing (idf.py build, idf.py flash monitor)
- Claude focuses on code implementation and debugging