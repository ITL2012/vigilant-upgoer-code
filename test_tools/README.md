ESP32 Test Tools

A minimal PlatformIO project that provides a serial CLI to test:
- SD card mounting and writing
- PSRAM allocation
- I2C bus scanning
- Memory diagnostics

Usage:

1. Build and upload to your ESP32-S3 board:

```bash
pio run --environment esp32-s3-devkitc-1 --target upload
pio device monitor --environment esp32-s3-devkitc-1 --baud 115200
```

2. Use the serial CLI commands (type `help`).

Notes:
- Edit `include/test_tools.h` to adjust pin defines for `SD_CS`, `I2C_SDA`, `I2C_SCL` if your board differs.
- This project is intentionally small and portable for use across ESP32 S3 projects.
