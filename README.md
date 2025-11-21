# FlightComputer

High-performance embedded flight computer for aerospace applications, built on the STM32G4xx platform with comprehensive sensor fusion and real-time telemetry.

## Overview

FlightComputer is a professional-grade flight control system designed for model rockets, UAVs, and high-performance aerospace applications. It features a 9-DOF attitude estimation system, multi-GNSS positioning, barometric altitude sensing, and high-G shock detection - all running on a 170 MHz ARM Cortex-M4 with hardware-accelerated mathematics.

**Target Applications:**
- Model rocket flight computers
- UAV autopilot systems
- High-performance aerospace telemetry
- Sensor fusion research platforms

## Key Features

### Sensor Suite
- **9-DOF Attitude Estimation** - Mahony AHRS filter with hardware CORDIC acceleration
- **ICM-42688-P IMU** - 6-axis gyro/accel at 1000 Hz with ±2000 dps / ±16g range
- **MMC5983MA Magnetometer** - 3-axis at 1000 Hz for accurate heading
- **BMP581 Barometer** - High-precision altitude measurement at 100 Hz
- **H3LIS331DL High-G Accelerometer** - ±400g shock detection
- **u-blox SAM-M10Q GPS** - Multi-GNSS with UBX protocol, covariance data, DOP values

### Architecture
- **DMA-First Design** - All sensors use DMA for zero-copy, non-blocking transfers
- **Hardware Acceleration** - CORDIC for trigonometry, FPU for floating-point math
- **Priority Scheduling** - I2C arbiter with intelligent sensor prioritization
- **4MB External Flash** - QSPI for flight data logging
- **Real-Time Performance** - 500 Hz AHRS updates, 1000 Hz sensor acquisition

### Communication
- **BLE Module (RN4871)** - Wireless telemetry and command interface with DMA
- **GPS (USART3)** - UBX binary protocol with DMA circular buffer
- **USB CDC** - Debug output and configuration
- **USART1/2** - Additional serial interfaces with DMA support

## Hardware Platform

### Microcontroller: STM32G4xx
- **Core:** ARM Cortex-M4F @ 170 MHz
- **FPU:** Hardware floating-point unit with DSP instructions
- **CORDIC:** Hardware trigonometric accelerator
- **DMA:** 7 channels for concurrent zero-copy transfers
- **Memory:** Flash + RAM (size depends on specific variant)
- **Voltage:** 3.3V operation with boost mode for maximum performance

### Sensors
| Sensor | Interface | Update Rate | Range | DMA |
|--------|-----------|-------------|-------|-----|
| ICM-42688-P | SPI3 | 1000 Hz | ±2000 dps / ±16g | ✓ |
| MMC5983MA | I2C1 | 1000 Hz | 3-axis magnetometer | ✓ |
| BMP581 | I2C1 | 100 Hz | Pressure/altitude | ✓ |
| H3LIS331DL | I2C1 | 50-100 Hz | ±400g | ✓ |
| SAM-M10Q | USART3 | 5-10 Hz | GPS/GLONASS/Galileo/BeiDou | ✓ |

### Peripherals
- **4MB QSPI Flash** - Winbond W25Q32 for data logging
- **BLE Module** - RN4871 for wireless telemetry
- **3x Status LEDs** - PA4, PA5, PC4
- **1 PPS Signal** - GPS time pulse on PC15

## Software Architecture

### Layered Design
```
┌─────────────────────────────────────────────┐
│ Application Layer                           │
│ (Mahony Filter, GPS Integration, Telemetry) │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ Management Layer                            │
│ (Sensor Manager, I2C Arbiter, Data Logger)  │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ Driver Layer                                │
│ (Sensor Drivers, BLE, GPS)                  │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ HAL Layer                                   │
│ (STM32 HAL: I2C, SPI, DMA, CORDIC, USART)   │
└─────────────────────────────────────────────┘
```

### Key Modules

**Sensor Manager** (`sensor_manager.h/c`)
- Unified sensor coordination with DMA-based acquisition
- Automatic scaling and calibration
- Priority-based decimation for efficient CPU usage
- Timestamp synchronization at microsecond resolution

**I2C DMA Arbiter** (`i2c_dma_arbiter.h/c`)
- Priority-based arbitration for shared I2C bus
- Non-preemptive scheduling with conflict detection
- Supports magnetometer (highest), barometer (medium), high-g accel (lowest)

**Mahony AHRS Filter** (`mahony_filter.h/c`)
- 9-DOF sensor fusion in NED frame
- Hardware CORDIC acceleration for trigonometry
- Outputs: quaternions, Euler angles, rotation matrices
- Configurable gains (Kp, Ki) and sample rate

**GPS Module** (`gps_module.h/c`)
- UBX protocol with comprehensive data extraction
- DMA circular buffer with idle line detection
- Covariance matrices for EKF integration
- Complete DOP values (PDOP, HDOP, VDOP, etc.)
- Spoofing/jamming detection

**BLE Module** (`ble_module.h/c`)
- RN4871 wireless interface with DMA
- Built-in commands: start, stop, status, help, erase
- Custom command extensibility
- Data transmission control
- Connection status monitoring

## Project Structure

```
FlightComputer/
├── Inc/                    # Header files
│   ├── Sensor drivers (icm42688.h, mmc5983ma.h, bmp581.h, h3lis331dl.h)
│   ├── System (sensor_manager.h, i2c_dma_arbiter.h)
│   ├── Algorithms (mahony_filter.h, cordic_math.h)
│   ├── Communication (gps_module.h, ble_module.h, data_logger.h)
│   └── HAL config (stm32g4xx_hal_conf.h, main.h)
├── Src/                    # Source files
│   ├── Sensor implementations
│   ├── System implementations
│   ├── Algorithm implementations
│   ├── Communication implementations
│   └── main.c (application entry point)
├── CLAUDE.md              # Comprehensive developer documentation
└── README.md              # This file
```

## Getting Started

### Prerequisites
- **STM32CubeIDE** (recommended) or compatible ARM toolchain
- **ST-Link** debugger for programming
- **STM32CubeMX** for peripheral configuration (optional)

### Build
1. Open project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to target via ST-Link

### Configuration
Key configuration parameters in `main.c`:

```c
// Mahony filter parameters
float kp = 1.0f;           // Proportional gain
float ki = 0.0f;           // Integral gain
float sample_freq = 500.0f; // Hz

// Sensor update rates
// IMU: 1000 Hz (hardware configured)
// Magnetometer: 1000 Hz (hardware configured)
// Barometer: 100 Hz (decimated)
// GPS: 5-10 Hz (hardware dependent)
```

### Hardware Setup
1. **Power:** 3.3V to VDD
2. **IMU:** SPI3 (CS on PC13, INT on PC14)
3. **I2C Sensors:** I2C1 (SCL/SDA with appropriate pull-ups)
4. **GPS:** USART3 @ 115200 baud
5. **BLE:** USART1 @ 115200 baud (RST_BT on PC6)
6. **QSPI Flash:** QSPI1 interface
7. **Debug:** USB CDC or ST-Link SWD

### Testing

**BLE Test (included in main.c):**
```c
// Built-in commands via BLE:
// - "start" - Start data logging
// - "stop"  - Stop data logging
// - "status" - Get system status
// - "help" - List commands
// - "erase" - Erase flash memory

// Custom commands:
// - "test" - BLE connectivity test
// - "info" - System information
```

**Expected Output:**
- BLE sends telemetry every 1 second
- Debug output via USB CDC
- Statistics monitoring (RX/TX bytes, connections)

## Performance Specifications

### Update Rates
- **IMU:** 1000 Hz (gyro + accel)
- **Magnetometer:** 1000 Hz (critical for heading)
- **Mahony Filter:** 500 Hz (minimum for stability)
- **Barometer:** 100 Hz (decimated)
- **High-G Accel:** 50-100 Hz (shock detection)
- **GPS:** 5-10 Hz (typical)

### Coordinate Frames
- **NED (North-East-Down):** Standard aerospace reference frame
- All attitude outputs in NED frame
- Transformations handled in Mahony filter

### DMA Architecture
All high-bandwidth interfaces use DMA:
- **Interrupt reduction:** ~99% fewer interrupts vs. polling
- **CPU availability:** Frees CPU for sensor fusion calculations
- **Deterministic timing:** Predictable latency for real-time operations
- **Zero-copy:** Direct memory transfers without CPU intervention

## Development

### Code Conventions
- **Files:** Lowercase with underscores (`sensor_manager.c`)
- **Functions:** `ModuleName_FunctionName()` (e.g., `GPS_Init()`)
- **Types:** `ModuleName_TypeName_t` (e.g., `GPS_Data_t`)
- **Macros:** `UPPER_CASE_WITH_UNDERSCORES`

### STM32CubeMX Integration
Code is generated with STM32CubeMX. Custom code must be placed within protected sections:

```c
/* USER CODE BEGIN 0 */
// Custom code here - preserved during regeneration
/* USER CODE END 0 */
```

### Adding Features
See `CLAUDE.md` for detailed developer documentation including:
- Adding new sensors
- Modifying filter parameters
- Implementing telemetry
- Flash logging
- Debugging strategies

## Current Status

### Implemented ✓
- Complete sensor driver infrastructure
- GPS with comprehensive UBX protocol support
- Sensor Manager with DMA acquisition
- I2C DMA Arbiter
- Mahony AHRS filter with CORDIC acceleration
- QSPI flash interface
- BLE wireless communication with DMA
- Data logger stub
- Comprehensive BLE test suite

### In Development
- Full AHRS integration in main loop
- Sensor calibration routines
- Flight state machine
- Complete data logging implementation
- EKF integration with GPS
- Ground testing utilities

## Documentation

- **README.md** (this file) - Project overview and quick start
- **CLAUDE.md** - Comprehensive developer documentation with architecture details

## License

Copyright (c) 2025 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file in the root directory of this software component.

## Contributing

This is a research/development project. For questions or contributions, please refer to the project documentation.

## Acknowledgments

- STMicroelectronics for HAL library and development tools
- TDK InvenSense for ICM-42688-P IMU
- MEMSIC for MMC5983MA magnetometer
- Bosch Sensortec for BMP581 barometer
- u-blox for SAM-M10Q GNSS receiver
- Microchip for RN4871 BLE module

---

**Built with STM32CubeIDE** | **Powered by ARM Cortex-M4** | **DMA-First Architecture**
