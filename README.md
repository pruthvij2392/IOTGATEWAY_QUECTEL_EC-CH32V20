# SWAIOT Gateway V1 (IOTGATEWAY_V1)

**SWAIOT Gateway** is an industrial-grade IoT gateway firmware developed for the **WCH CH32V203** 32-bit RISC-V microcontroller. It interfaces with field sensors via RS-485 Modbus RTU, logs data to external SPI Flash, tracks high-precision timestamps with an RTC, and transmits telemetry to cloud/MQTT brokers over 4G LTE cellular connectivity.

---

## 🚀 Key Features

- **Microcontroller**: WCH CH32V203 (RISC-V Core)
- **Cellular Connectivity**: Quectel 4G LTE Module (USART2) with AT command parser and MQTT/TCP/HTTP support.
- **Industrial Fieldbus**: RS-485 Modbus RTU Master (USART1 with Hardware Direction Control Pin).
- **USB CDC Virtual COM Port**: Interactive Command Line Interface (CLI) and JSON-based configuration tool for Docklight / Serial Terminals.
- **External Flash Storage**: Winbond W25Q32 (SPI1) for non-volatile parameter storage, system logs, and offline buffering.
- **Real-Time Clock (RTC)**: Maxim DS3231 high-precision I2C RTC for time-stamping sensor measurements.
- **Status Indicators**: 4-Channel Multi-color LEDs (Red, Blue, White, Yellow) for connection, sensor, and system state feedback.
- **System Reliability**: Independent Watchdog (IWDG) and system recovery routines.

---

## 🛠️ Hardware & Pin Configuration

| Peripheral | Interface / Pin | Description |
| :--- | :--- | :--- |
| **MCU** | CH32V203 | 32-bit RISC-V MCU |
| **USB CDC** | USB FS (`PA11`/`PA12`) | Serial Monitor & CLI Configuration |
| **RS-485** | USART1 (`PA9`: TX, `PA10`: RX, `PA8`: DE/RE) | Modbus RTU Sensor Bus |
| **4G Cellular** | USART2 (`PA2`: TX, `PA3`: RX, `PB4`: EN, `PB3`: PWR) | Quectel LTE Module |
| **RTC (DS3231)**| I2C1 (`PB6`: SCL, `PB7`: SDA) | Real-Time Clock |
| **SPI Flash (W25Q32)** | SPI1 (`PA4`: CS, `PA5`: SCK, `PA6`: MISO, `PA7`: MOSI) | 32Mbit Flash Memory |
| **Status LEDs** | `PB1` (Red), `PA1` (Blue), `PA15` (White), `PB0` (Yellow) | System & Network Status |

---

## 📁 Project Structure

```text
IOTGATEWAY_V1/
├── Core/               # RISC-V core system definitions
├── Ld/                 # Linker scripts for CH32V203
├── Peripheral/         # WCH CH32V20x Standard Peripheral Drivers
├── Startup/            # Assembly startup files (startup_ch32v20x_D6.S)
├── User/               # Application Code
│   ├── Main.c          # Main state machine and entry point
│   ├── UART/           # UART driver layers
│   ├── USBLIB/         # USB CDC peripheral stack
│   ├── ds3231.c/.h     # DS3231 RTC driver
│   ├── gsm_quectel.c/.h# Quectel 4G LTE communication & MQTT engine
│   ├── modbus_rtu.c/.h # Modbus RTU master implementation
│   ├── w25qxx.c/.h     # W25Q32 SPI Flash driver
│   └── comman.c/.h     # Common utilities & JSON parameter parsers
└── IOTGATEWAY_V1.wvproj# MounRiver Studio Project Configuration
```

---

## 💻 Development Environment & Build

- **IDE**: [MounRiver Studio (MRS)](http://www.mounriver.com/)
- **Toolchain**: RISC-V Embedded GCC (`riscv-none-embed-gcc`)
- **Flashing / Debugging**: WCH-LinkE debugger / OpenOCD

---

## 👤 Author
- **Developer**: Pruthvi Jyoti
- **Organization**: SWASEMI PVT LTD
