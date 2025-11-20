# CLAUDE.md - FlightComputer AI Assistant Guide

This document provides AI assistants with comprehensive information about the FlightComputer codebase structure, conventions, and development workflows.

## Project Overview

**FlightComputer** is an embedded flight computer system built on the **STM32G4xx** microcontroller platform (ARM Cortex-M4). It implements a high-performance sensor fusion system for aerospace applications, featuring:

- 9-DOF attitude estimation (Mahony AHRS filter)
- Multi-sensor integration with DMA-based data acquisition
- GPS positioning with UBX protocol support
- Hardware-accelerated mathematics (CORDIC)
- External flash storage (4MB QSPI)
- Real-time telemetry via USB/USART

**Target Application:** Rocket flight computer, UAV autopilot, or similar high-performance aerospace systems.

---

## Directory Structure

```
/home/user/FlightComputer/
├── Inc/                    # Header files (.h)
│   ├── Sensor drivers (icm42688.h, mmc5983ma.h, bmp581.h, etc.)
│   ├── System management (sensor_manager.h, i2c_dma_arbiter.h)
│   ├── HAL configuration (stm32g4xx_hal_conf.h, main.h)
│   ├── Communication (i2c.h, spi.h, usart.h, quadspi.h)
│   └── Math/filtering (mahony_filter.h, cordic_math.h)
├── Src/                    # Source files (.c)
│   ├── Sensor driver implementations
│   ├── System management implementations
│   ├── HAL/MSP initialization
│   └── Application entry point (main.c)
└── README.md              # Minimal project description
```

---

## Hardware Platform: STM32G4xx

### Microcontroller Specifications
- **Core:** ARM Cortex-M4 with FPU and DSP
- **Clock:** 170 MHz maximum (PLL from 8 MHz HSE)
- **Voltage Scaling:** SCALE1_BOOST mode for maximum performance
- **VDD:** 3.3V (VDD_VALUE = 3300)

### Hardware Accelerators
- **CORDIC:** Hardware trigonometric functions (sine, cosine, arctangent, square root)
- **FMAC:** Filtering MAC unit for digital filtering
- **DMA:** 7 channels for zero-copy sensor data transfers

### Clock Configuration
- **HSE:** 8 MHz external crystal oscillator
- **PLL:** Configured for 170 MHz system clock
- **HSI48:** 48 MHz internal oscillator for USB operation

---

## Sensor Suite

### 1. ICM-42688-P (6-Axis IMU)
- **Interface:** SPI3 with DMA
- **Chip Select:** PC13
- **Capabilities:**
  - 3-axis gyroscope: ±2000 dps (16.4 LSB/dps)
  - 3-axis accelerometer: ±16g (2048 LSB/g)
  - Output Data Rate: 1000 Hz
  - Temperature sensor
- **Interrupt:** INT_IMU (PC14) - data ready signal
- **Update Rate:** 1000 Hz
- **File:** `Inc/icm42688.h`, `Src/icm42688.c`

### 2. MMC5983MA (3-Axis Magnetometer)
- **Interface:** I2C1 (address: 0x30) with DMA
- **Capabilities:**
  - Resolution: 16384 LSB/Gauss
  - Continuous mode at 1000 Hz
  - SET/RESET calibration
  - Self-test capability
- **Interrupt:** INT_MAG (PB7)
- **DMA Priority:** **Highest** (critical for Mahony heading)
- **Update Rate:** 1000 Hz
- **File:** `Inc/mmc5983ma.h`, `Src/mmc5983ma.c`

### 3. H3LIS331DL (High-G Accelerometer)
- **Interface:** I2C1 (address: 0x18) with DMA
- **Capabilities:**
  - Selectable ranges: ±100g, ±200g, ±400g
  - ODR: 50, 100, 400, 1000 Hz
  - Shock/impact detection
- **Interrupt:** INT_ACC (PA13)
- **DMA Priority:** **Lowest** (decimated for shock events)
- **Update Rate:** 50-100 Hz (decimated)
- **File:** `Inc/h3lis331dl.h`, `Src/h3lis331dl.c`

### 4. BMP581 (Barometric Pressure Sensor)
- **Interface:** I2C1 (address: 0x47) with DMA
- **Capabilities:**
  - Pressure resolution: 64 LSB/Pa
  - Temperature compensation
  - Configurable ODR: 50-240 Hz
  - IIR filtering support
  - Altitude calculation
- **Interrupt:** INT_BMP (PB6)
- **DMA Priority:** **Medium**
- **Update Rate:** 100 Hz (decimated)
- **File:** `Inc/bmp581.h`, `Src/bmp581.c`

### 5. GPS Module (u-blox SAM-M10Q)
- **Hardware:** u-blox SAM-M10Q concurrent GNSS receiver
- **Interface:** USART3 @ 115200 baud
- **Protocol:** UBX (binary protocol)
- **GNSS Constellations:** GPS, GLONASS, Galileo, BeiDou, QZSS (configurable)
- **Dynamic Model:** Airborne <4g (configured for rocket flight)
- **Fix Mode:** 3D only
- **Data Provided:**
  - Position: Latitude, Longitude, Altitude (MSL)
  - Velocity: North, East, Down components (NED frame)
  - Accuracy: Horizontal, Vertical, Speed
  - Covariance: Position and velocity matrices (NAV-COV)
  - DOP: PDOP, HDOP, VDOP, GDOP, TDOP, NDOP, EDOP (NAV-DOP)
  - Course accuracy and climb rate
  - Spoofing/jamming detection (NAV-STATUS)
  - Differential corrections status
  - Time to first fix
- **UBX Messages Enabled:**
  - NAV-PVT: Position, velocity, time
  - NAV-COV: Covariance matrices for EKF integration
  - NAV-DOP: Dilution of precision values
  - NAV-VELNED: Velocity in NED frame
  - NAV-STATUS: Fix quality and spoofing detection
- **Configuration:** NMEA disabled, UBX-only for efficiency
- **Time Pulse:** PC15 (1 PPS signal for time synchronization)
- **Control Pins:**
  - RST_SAM (PA1): Hardware reset
  - INT_SAM (PA0): Interrupt/event notification
- **Update Rate:** 5-10 Hz (typical), configurable
- **File:** `Inc/gps_module.h`, `Src/gps_module.c`

---

## Major Subsystems

### 1. Sensor Manager (`sensor_manager.h/c`)

**Purpose:** Unified coordination of all sensors with optimized data fusion.

**Key Features:**
- DMA-based sensor reading for maximum efficiency
- Priority-based sensor scheduling
- Decimation for lower-priority sensors
- Timestamp synchronization (microsecond resolution)
- Raw and scaled data outputs
- Automatic scaling factor calculation
- Mahony filter data conversion

**Data Structures:**
```c
SensorManager_RawData_t      // Raw sensor values with validity flags
SensorManager_ScaledData_t   // Calibrated physical units
SensorManager_Config_t       // Runtime configuration
SensorManager_Status_t       // Performance statistics
```

**Key Functions:**
```c
SensorManager_Init()                          // Initialize all sensors
SensorManager_ReadRaw(&raw_data)              // Get raw sensor data
SensorManager_ConvertToScaled(&raw, &scaled)  // Convert to physical units
SensorManager_GetMahonyData(...)              // Extract data for AHRS filter
```

**Design Pattern:** Centralized sensor hub with fail-safe fallbacks.

**File Location:** `Inc/sensor_manager.h:1`, `Src/sensor_manager.c:1`

---

### 2. I2C DMA Arbiter (`i2c_dma_arbiter.h/c`)

**Purpose:** Priority-based arbitration for multiple I2C devices sharing a single bus.

**Priority Hierarchy:**
1. **Magnetometer (MMC5983MA):** Highest priority - 1000 Hz for Mahony filter
2. **Barometer (BMP581):** Medium priority - decimated to 100 Hz
3. **High-G Accelerometer (H3LIS331DL):** Lowest priority - decimated to 50 Hz

**Key Features:**
- Non-preemptive scheduling (simpler, more predictable)
- Conflict detection and statistics
- DMA callback chaining
- Busy status checking

**Design Pattern:** Resource arbiter with priority queuing.

**File Location:** `Inc/i2c_dma_arbiter.h:1`, `Src/i2c_dma_arbiter.c:1`

---

### 3. Mahony AHRS Filter (`mahony_filter.h/c`)

**Purpose:** 9-DOF sensor fusion for attitude and heading estimation.

**Key Features:**
- Fuses gyroscope, accelerometer, and magnetometer data
- NED (North-East-Down) reference frame
- Hardware CORDIC acceleration for trigonometric functions
- Configurable gains (Kp, Ki)
- Launch pad initialization (X-axis pointing up)

**Output Formats:**
- Quaternions (q0, q1, q2, q3)
- Euler angles (roll, pitch, yaw)
- Rotation matrices (DCM - Direction Cosine Matrix)

**Update Rate:** 500 Hz (default, configurable)

**Key Functions:**
```c
Mahony_Init(&filter, sample_freq, kp, ki)              // Initialize filter
Mahony_Update(&filter, gx, gy, gz, ax, ay, az, mx, my, mz)  // Update with sensor data
Mahony_GetEulerAngles(&filter, &euler)                 // Get roll/pitch/yaw
Mahony_GetQuaternion(&filter, &q0, &q1, &q2, &q3)     // Get quaternion
```

**Design Pattern:** Complementary filter with hardware acceleration.

**File Location:** `Inc/mahony_filter.h:1`, `Src/mahony_filter.c:1`

---

### 4. Hardware Math Acceleration (`cordic_math.h/c`)

**Purpose:** Deterministic, hardware-accelerated trigonometric functions using the STM32G4's CORDIC coprocessor.

**Functions Provided:**
```c
CORDIC_Atan2(y, x)      // Two-argument arctangent
CORDIC_Sqrt(x)          // Square root
CORDIC_SinCos(angle)    // Simultaneous sine/cosine
CORDIC_Asin(x)          // Arcsine
CORDIC_Acos(x)          // Arccosine
```

**Advantages:**
- Deterministic execution time (no iterative loops)
- Lower CPU overhead vs. standard math library (`libm`)
- Precision: ~20-24 bits (15 CORDIC cycles)

**Implementation:** Q1.31 fixed-point with float conversion wrappers.

**File Location:** `Inc/cordic_math.h:1`, `Src/cordic_math.c:1`

---

### 5. QSPI Flash Interface (`quadspi.h/c`)

**Purpose:** External flash storage (Winbond W25Q32, 4MB).

**Features:**
- Quad SPI mode for fast transfers
- DMA support for non-blocking operations
- Sector erase (4KB)
- Chip erase
- Page programming (256 bytes)
- JEDEC ID reading

**Use Cases:** Flight data logging, configuration storage, black box recorder.

**File Location:** `Inc/quadspi.h:1`, `Src/quadspi.c:1`

---

## Communication Interfaces

### I2C1
- **Purpose:** Sensor bus (magnetometer, barometer, high-g accelerometer)
- **DMA:** Yes, with priority arbitration via I2C DMA Arbiter
- **Speed:** Standard/Fast mode
- **File:** `Inc/i2c.h`, `Src/i2c.c`

### SPI3
- **Purpose:** IMU communication (ICM-42688-P)
- **DMA:** Yes, for high-speed sensor reads
- **CS Pin:** PC13
- **File:** `Inc/spi.h`, `Src/spi.c`

### USART Channels
- **USART1:** General purpose / Bluetooth module
- **USART2:** General purpose
- **USART3:** GPS module (UBX protocol)
- **File:** `Inc/usart.h`, `Src/usart.c`

### QUADSPI1
- **Purpose:** External flash memory
- **Memory:** Winbond W25Q32 (4MB)
- **Mode:** Quad SPI with DMA
- **File:** `Inc/quadspi.h`, `Src/quadspi.c`

### USB Device
- **Class:** CDC (Virtual COM Port)
- **Purpose:** Debug output, telemetry, configuration
- **Clock:** HSI48 (48 MHz internal oscillator)

---

## GPIO Pin Assignments

### LEDs
- **LED1:** PA4
- **LED2:** PA5
- **LED3:** PC4

### Sensor Interrupts
- **INT_IMU:** PC14 (IMU data ready)
- **INT_MAG:** PB7 (Magnetometer data ready)
- **INT_ACC:** PA13 (High-G accelerometer data ready)
- **INT_BMP:** PB6 (Barometer data ready)

### GPS Signals
- **TIMEPULSE:** PC15 (GPS PPS - Pulse Per Second for time sync)

### SPI Chip Selects
- **CS:** PC13 (IMU chip select)

### Module Control
- **INT_SAM:** PA0
- **RST_SAM:** PA1
- **RST_BT:** PC6 (Bluetooth module reset)
- **LPM:** PA8 (Low power mode control)

### Radio/LoRa Control
- **M1:** PB12
- **M0:** PB13
- **AUX:** PB14
- **CONFIG:** PB15

**Reference:** See `Inc/main.h:1` for complete GPIO definitions.

---

## Architectural Patterns

### 1. DMA-First Design
- All high-frequency sensors use DMA for zero-copy transfers
- Non-blocking I/O for maximum CPU availability
- Callback-based completion handling
- Minimizes interrupt overhead and CPU load

### 2. Priority-Based Scheduling
- Critical sensors (IMU, magnetometer) get highest priority
- Non-critical sensors (barometer, high-g accel) are decimated
- Fail-safe fallbacks maintain system stability
- Implemented via I2C DMA Arbiter

### 3. Hardware Acceleration
- CORDIC for trigonometry (Mahony filter)
- FMAC available for filtering (not currently used)
- DMA for all data movement

### 4. Layered Architecture
```
┌─────────────────────────────────────────────┐
│ Application Layer                           │
│ (Mahony Filter, GPS Integration, Telemetry) │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ Management Layer                            │
│ (Sensor Manager, I2C Arbiter)               │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ Driver Layer                                │
│ (ICM42688, MMC5983MA, BMP581, H3LIS331DL)   │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ HAL Layer                                   │
│ (STM32 HAL: I2C, SPI, DMA, CORDIC, USART)   │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│ Hardware Layer                              │
│ (STM32G4xx MCU)                             │
└─────────────────────────────────────────────┘
```

### 5. Data Flow Pattern
```
Sensors → DMA → Raw Data Buffer → Sensor Manager →
Scaling → Mahony Filter → Attitude Output → Telemetry
```

---

## Performance Characteristics

### Update Rates
- **IMU (ICM-42688-P):** 1000 Hz
- **Magnetometer (MMC5983MA):** 1000 Hz
- **Mahony AHRS Filter:** 500 Hz (default)
- **Barometer (BMP581):** 100 Hz (decimated from sensor rate)
- **High-G Accelerometer:** 50-100 Hz (decimated, used for shock detection)
- **GPS:** 5-10 Hz (typical, hardware dependent)

### DMA Channels Used
- I2C1 TX/RX (sensors)
- SPI3 TX/RX (IMU)
- QUADSPI (flash memory)
- USART1/2/3 (GPS, telemetry, Bluetooth)

### CPU Load Optimization
- DMA offloads all sensor data transfers
- CORDIC accelerates Mahony filter math
- Priority-based decimation reduces unnecessary reads
- Non-blocking I/O keeps main loop responsive

---

## Coordinate Frames

### NED Frame (North-East-Down)
- **Standard aerospace reference frame**
- X-axis: North
- Y-axis: East
- Z-axis: Down (gravity positive)
- Used by Mahony filter output
- Transformations provided for sensor→NED conversion

### Sensor Frame
- **IMU:** As mounted on PCB (see sensor datasheets)
- **Magnetometer:** Includes hard/soft iron compensation
- **Coordinate transformations:** Handled in Mahony filter initialization

**Reference:** See `Src/mahony_filter.c` for coordinate frame conversions.

---

## Build System

**Note:** This repository contains source code only. No build files (Makefile, CMakeLists.txt, `.ioc`) are present in the repository.

### Evidence of STM32CubeMX Usage
- HAL library structure
- `MX_*_Init()` function naming convention
- `USER CODE` comment sections in generated files
- Standard peripheral initialization pattern

### Expected Build Environment
- **IDE:** STM32CubeIDE (recommended) or Keil MDK / IAR EWARM
- **Toolchain:** ARM GCC (arm-none-eabi-gcc)
- **Configuration Tool:** STM32CubeMX (for peripheral configuration)
- **Debugger:** ST-Link (SWD interface)

### Typical Build Workflow
1. Open project in STM32CubeIDE
2. Configure peripherals with STM32CubeMX (if `.ioc` file exists)
3. Build with ARM GCC toolchain
4. Flash via ST-Link debugger
5. Debug with OpenOCD or proprietary ST tools

---

## Development Workflow

### Initialization Sequence
```c
// Standard STM32 initialization pattern
HAL_Init();                  // Initialize HAL library
SystemClock_Config();        // Configure clocks (170 MHz)
MX_GPIO_Init();              // Initialize GPIOs
MX_DMA_Init();               // Initialize DMA channels
MX_I2C1_Init();              // Initialize I2C bus
MX_SPI3_Init();              // Initialize SPI bus
MX_USART1_Init();            // Initialize USART channels
MX_USART2_Init();
MX_USART3_Init();
MX_CORDIC_Init();            // Initialize CORDIC accelerator
MX_QUADSPI_Init();           // Initialize QSPI flash
SensorManager_Init();        // Initialize sensor subsystem
Mahony_Init(&filter, ...);   // Initialize AHRS filter
```

### Main Loop Pattern
```c
while(1) {
    // Main application loop
    // Typically includes:
    // - Sensor data acquisition (via SensorManager)
    // - Mahony filter updates
    // - Telemetry transmission
    // - Command processing
    // - State machine updates
}
```

### Sensor Reading Pattern
```c
SensorManager_RawData_t raw_data;
SensorManager_ScaledData_t scaled_data;

// Read all sensors
SensorManager_ReadRaw(&raw_data);

// Convert to physical units
SensorManager_ConvertToScaled(&raw_data, &scaled_data);

// Use scaled data
float accel_x = scaled_data.imu_accel_x;  // in m/s²
float pressure = scaled_data.baro_pressure;  // in Pa
```

### Mahony Filter Update Pattern
```c
float gx, gy, gz, ax, ay, az, mx, my, mz;
Mahony_EulerAngles_t euler;

// Extract sensor data for Mahony filter
SensorManager_GetMahonyData(&raw_data, &gx, &gy, &gz,
                            &ax, &ay, &az, &mx, &my, &mz);

// Update AHRS filter
Mahony_Update(&filter, gx, gy, gz, ax, ay, az, mx, my, mz);

// Get attitude output
Mahony_GetEulerAngles(&filter, &euler);

// Use attitude data
float roll = euler.roll;    // in radians
float pitch = euler.pitch;  // in radians
float yaw = euler.yaw;      // in radians (heading)
```

---

## Code Conventions

### File Organization
- **Headers (`.h`):** In `Inc/` directory
- **Sources (`.c`):** In `Src/` directory
- **Naming:** Lowercase with underscores (e.g., `sensor_manager.c`)
- **Module pattern:** Each peripheral/subsystem has paired `.h` and `.c` files

### Code Generation Sections
Files contain STM32CubeMX-generated code with protected sections:
```c
/* USER CODE BEGIN 0 */
// Custom code here - preserved during regeneration
/* USER CODE END 0 */
```

**Important:** Always place custom code within `USER CODE` sections to prevent loss during regeneration.

### Naming Conventions
- **Functions:** `ModuleName_FunctionName()` (e.g., `SensorManager_Init()`)
- **Types:** `ModuleName_TypeName_t` (e.g., `SensorManager_RawData_t`)
- **Macros:** `UPPER_CASE_WITH_UNDERSCORES`
- **Constants:** `UPPER_CASE_WITH_UNDERSCORES`
- **Variables:** `lower_case_with_underscores`

### Header Guards
```c
#ifndef MODULE_NAME_H
#define MODULE_NAME_H
// ...
#endif /* MODULE_NAME_H */
```

---

## Critical Timing Requirements

### Mahony Filter Stability
- **Minimum update rate:** 500 Hz
- **Recommended:** 500-1000 Hz
- **Consequence of violation:** Divergence, orientation drift

### Magnetometer Reading
- **Required rate:** 1000 Hz for accurate heading
- **Priority:** Highest (via I2C DMA Arbiter)
- **Consequence of violation:** Heading errors, yaw drift

### GPS Updates
- **Rate:** Asynchronous (5-10 Hz typical)
- **Handling:** Interrupt-driven via USART3
- **Consequence of violation:** Position staleness

---

## Common Development Tasks

### Adding a New Sensor
1. Create driver files: `Inc/new_sensor.h`, `Src/new_sensor.c`
2. Implement initialization, read, and configuration functions
3. Add sensor to `SensorManager` data structures
4. Configure I2C/SPI interface in STM32CubeMX (if needed)
5. Update `SensorManager_Init()` to initialize new sensor
6. Add DMA channel if high-frequency sensor
7. Update I2C DMA Arbiter priority if using I2C

### Modifying Mahony Filter Parameters
- **File:** `Inc/mahony_filter.h`, `Src/mahony_filter.c`
- **Key parameters:**
  - `kp`: Proportional gain (default: 1.0-2.0)
  - `ki`: Integral gain (default: 0.0-0.1)
  - `sample_freq`: Update rate in Hz (default: 500)
- **Re-initialization required after changes**

### Adding Telemetry Output
1. Choose interface: USB CDC, USART1, or USART2
2. Implement packet serialization (binary or text)
3. Call from main loop at desired rate
4. Consider DMA for high-rate telemetry

### Flash Logging
1. Use QSPI interface (`quadspi.h/c`)
2. Erase sectors before writing
3. Page program in 256-byte chunks
4. Consider circular buffer for continuous logging

---

## Debugging and Diagnostics

### Debug Utilities (`debug_utils.h/c`)
- Provides debug output functions
- Typically routed to USART or USB CDC
- Use for printf-style debugging

### I2C Scanner (`i2c_scanner.h/c`)
- Scans I2C bus for connected devices
- Useful for hardware verification
- Detects address conflicts

### LED Indicators
- **LED1 (PA4):** System status / heartbeat
- **LED2 (PA5):** Error indication
- **LED3 (PC4):** Custom application use

### Common Issues
1. **I2C conflicts:** Check I2C DMA Arbiter statistics
2. **Sensor initialization failures:** Verify power, pull-ups, addresses
3. **Mahony divergence:** Check update rate, sensor calibration
4. **GPS not updating:** Verify USART3 baud rate, UBX protocol
5. **Flash memory errors:** Check QSPI clock, erase before write

---

## File Reference Quick Index

### Core System Files
- **Application Entry:** `Src/main.c:1`
- **System Initialization:** `Src/system_stm32g4xx.c:1`
- **HAL Configuration:** `Inc/stm32g4xx_hal_conf.h:1`
- **GPIO Definitions:** `Inc/main.h:1`

### Sensor Drivers
- **IMU:** `Inc/icm42688.h:1`, `Src/icm42688.c:1`
- **Magnetometer:** `Inc/mmc5983ma.h:1`, `Src/mmc5983ma.c:1`
- **Barometer:** `Inc/bmp581.h:1`, `Src/bmp581.c:1`
- **High-G Accel:** `Inc/h3lis331dl.h:1`, `Src/h3lis331dl.c:1`
- **GPS:** `Inc/gps_module.h:1`, `Src/gps_module.c:1`

### Management Layer
- **Sensor Manager:** `Inc/sensor_manager.h:1`, `Src/sensor_manager.c:1`
- **I2C Arbiter:** `Inc/i2c_dma_arbiter.h:1`, `Src/i2c_dma_arbiter.c:1`

### Algorithms
- **Mahony Filter:** `Inc/mahony_filter.h:1`, `Src/mahony_filter.c:1`
- **CORDIC Math:** `Inc/cordic_math.h:1`, `Src/cordic_math.c:1`

### Communication
- **I2C:** `Inc/i2c.h:1`, `Src/i2c.c:1`
- **SPI:** `Inc/spi.h:1`, `Src/spi.c:1`
- **USART:** `Inc/usart.h:1`, `Src/usart.c:1`
- **QSPI:** `Inc/quadspi.h:1`, `Src/quadspi.c:1`

### Utilities
- **Debug:** `Inc/debug_utils.h:1`, `Src/debug_utils.c:1`
- **I2C Scanner:** `Inc/i2c_scanner.h:1`, `Src/i2c_scanner.c:1`

---

## Key Takeaways for AI Assistants

1. **This is an embedded real-time system** - Timing is critical. Respect update rates and DMA priorities.

2. **Hardware acceleration is key** - Use CORDIC for math, DMA for data transfers. Don't introduce blocking calls.

3. **Layered architecture** - Respect abstraction layers. Don't bypass Sensor Manager to access drivers directly.

4. **STM32CubeMX generated code** - Always place custom code in `USER CODE` sections to prevent loss during regeneration.

5. **Priority-based scheduling** - Magnetometer and IMU are highest priority. Don't starve critical sensors.

6. **NED coordinate frame** - All outputs use North-East-Down reference. Maintain consistency.

7. **DMA arbitration** - Multiple I2C devices share one bus. Conflicts are managed by I2C DMA Arbiter.

8. **Mahony filter requires 500 Hz minimum** - Don't reduce update rate below this threshold.

9. **GPS is asynchronous** - UBX protocol updates arrive via interrupt, not polled.

10. **Flash memory requires erase before write** - QSPI flash must be erased in 4KB sectors before programming.

---

## Additional Resources

### STM32G4 Documentation
- **Reference Manual:** RM0440 (STM32G4 series)
- **Datasheet:** Check specific part number (e.g., STM32G473xx)
- **HAL Documentation:** STM32G4 HAL User Manual

### Sensor Datasheets
- **ICM-42688-P:** TDK InvenSense datasheet
- **MMC5983MA:** MEMSIC datasheet
- **H3LIS331DL:** STMicroelectronics datasheet
- **BMP581:** Bosch Sensortec datasheet

### Algorithms
- **Mahony Filter:** "Nonlinear Complementary Filters on the Special Orthogonal Group" (Mahony, Hamel, Pflimlin)
- **CORDIC:** "The CORDIC Trigonometric Computing Technique" (Jack E. Volder)

---

## Current Implementation Status

### What's Implemented ✓
- Complete sensor driver infrastructure (IMU, magnetometer, barometer, high-g accel)
- GPS module with comprehensive UBX protocol support (SAM-M10Q)
- Sensor Manager with DMA-based acquisition and priority scheduling
- I2C DMA Arbiter for conflict-free multi-device I2C bus access
- Mahony AHRS filter with hardware CORDIC acceleration
- QSPI flash interface for data storage
- Hardware math acceleration (CORDIC)
- Multiple communication interfaces (USB CDC, 3x USART, I2C, SPI, QSPI)

### What's Missing ✗
- **Main application loop** (currently empty!)
- Sensor initialization in main.c
- Mahony filter initialization and updates
- Interrupt handler connections
- Health monitoring and watchdog
- Sensor calibration routines
- Data logging implementation
- Telemetry protocol
- Configuration management system
- Error recovery and graceful degradation
- Flight state machine
- Ground testing utilities

### Critical Notes for AI Assistants

1. **The main loop is empty** - This is the highest priority issue. See `Src/main.c:119-125`.

2. **No sensors are initialized** - While drivers exist, nothing in main.c calls `SensorManager_Init()` or any sensor initialization.

3. **GPS hardware is SAM-M10Q** - All documentation should reference the u-blox SAM-M10Q specifically.

4. **Refer to REFACTOR_RECOMMENDATIONS.md** - Comprehensive refactoring analysis with 20+ specific recommendations prioritized by severity.

5. **This is a flight-critical system** - All changes must consider real-time constraints, deterministic timing, and safety implications.

6. **Timing is critical** - Mahony filter requires minimum 500 Hz updates. Don't introduce blocking calls in the main loop.

---

## Version History

- **Version 1.0** (2025-11-20): Initial comprehensive codebase documentation
- **Version 1.1** (2025-11-20): Added SAM-M10Q GPS specification and implementation status notes

---

## Related Documentation

- **REFACTOR_RECOMMENDATIONS.md** - Detailed refactoring analysis with prioritized recommendations
- **README.md** - Project overview

---

**End of CLAUDE.md**
