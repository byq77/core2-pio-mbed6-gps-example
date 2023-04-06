# CORE2 mbed6 pio template

## How to build PlatformIO based project
1. [Install PlatformIO Core](https://docs.platformio.org/page/core.html)
2. Run these commands:
```shell
# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Build specific environment
$ pio run -e core2

# Upload firmware for the specific environment
$ pio run -e core2 --target upload

# Clean build files
$ pio run --target clean
```
