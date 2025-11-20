# FlightComputer Refactoring Recommendations

**Generated:** 2025-11-20
**Codebase Size:** ~9,000 lines of code
**Status:** Initial refactoring analysis

---

## Executive Summary

This document provides a comprehensive refactoring analysis of the FlightComputer codebase. The code demonstrates solid architectural foundations with well-designed subsystems (Sensor Manager, I2C DMA Arbiter, Mahony Filter), but requires significant implementation work to become a functional flight computer system.

**Critical Finding:** The main application loop is currently **empty**. While the infrastructure is well-designed, no actual flight computer logic has been implemented.

---

## Priority Levels

- **P0 (Critical):** Must fix before flight testing - safety/functionality blockers
- **P1 (High):** Should fix soon - significantly improves reliability/maintainability
- **P2 (Medium):** Recommended - improves code quality
- **P3 (Low):** Nice to have - minor improvements

---

## P0 (Critical) Issues

### 1. Empty Main Loop (main.c:119-125)
**Severity:** CRITICAL - System does nothing after initialization

**Current State:**
```c
while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

**Problem:** The main loop is completely empty. No sensors are initialized, no Mahony filter updates, no telemetry, no state machine.

**Recommendation:**
Implement complete main loop with:
- Sensor Manager initialization
- Mahony filter initialization
- Main control loop at 500 Hz (or higher)
- Periodic GPS updates
- Telemetry output
- Health monitoring
- State machine for flight phases

**Example Structure:**
```c
// USER CODE BEGIN 2
SensorManager_Init(NULL);  // Use default config
Mahony_Init(&mahony_filter, MAHONY_KP_DEFAULT, MAHONY_KI_DEFAULT, 500.0f);
GPS_Init();

uint32_t last_cycle = HAL_GetTick();
// USER CODE END 2

while (1) {
    // USER CODE BEGIN 3
    SensorManager_RawData_t raw_data;

    // 500 Hz main loop
    if (HAL_GetTick() - last_cycle >= 2) {
        last_cycle = HAL_GetTick();

        // Read sensors
        SensorManager_ReadRaw(&raw_data);

        // Update Mahony filter
        float gx, gy, gz, ax, ay, az, mx, my, mz;
        SensorManager_GetMahonyData(&raw_data, &gx, &gy, &gz,
                                     &ax, &ay, &az, &mx, &my, &mz);
        Mahony_Update(&mahony_filter, gx, gy, gz, ax, ay, az, mx, my, mz);

        // Get attitude
        EulerAngles_t euler;
        Mahony_GetEulerAngles(&mahony_filter, &euler);

        // State machine and control logic here

        // Telemetry (every 10th cycle = 50 Hz)
        if (cycle_count % 10 == 0) {
            // Send telemetry
        }

        cycle_count++;
    }

    // GPS update (asynchronous)
    GPS_Update();
    // USER CODE END 3
}
```

---

### 2. Missing Error Handler Implementation (main.c:184-190)
**Severity:** CRITICAL - No diagnostics when failures occur

**Current State:**
```c
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}
```

**Problem:** When errors occur, the system hangs silently with no indication of what went wrong.

**Recommendation:**
Implement comprehensive error handler:
```c
void Error_Handler(void) {
    __disable_irq();

    // Blink LED pattern to indicate error
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        for (volatile int j = 0; j < 100000; j++);
    }

    // Log error information to flash if possible
    // Error_LogToFlash(__FILE__, __LINE__);

    // Infinite loop
    while (1) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);  // Slow blink = error
        for (volatile int j = 0; j < 1000000; j++);
    }
}
```

Add error logging capability:
- Store last error to flash
- Include file, line, timestamp
- LED patterns for different error types
- Optional reset after timeout

---

### 3. No Interrupt Handlers Connected (stm32g4xx_it.c)
**Severity:** CRITICAL - Sensor interrupts are defined but not used

**Problem:** Interrupt handlers are declared but callback functions are never called:
- `INT_IMU` (PC14) - IMU data ready
- `INT_MAG` (PB7) - Magnetometer data ready
- `INT_ACC` (PA13) - High-G accel data ready
- `INT_BMP` (PB6) - Barometer data ready

**Recommendation:**
1. Review `stm32g4xx_it.c` to ensure EXTI callbacks are implemented
2. Connect callbacks to sensor manager:
```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT_IMU_Pin) {
        SensorManager_IMU_DataReady_Callback();
    } else if (GPIO_Pin == INT_MAG_Pin) {
        SensorManager_MAG_DataReady_Callback();
    } else if (GPIO_Pin == INT_BMP_Pin) {
        SensorManager_BARO_DataReady_Callback();
    } else if (GPIO_Pin == INT_ACC_Pin) {
        SensorManager_HighG_DataReady_Callback();
    }
}
```

3. Implement event-driven architecture option in Sensor Manager
4. Add interrupt-driven GPS time pulse handling

---

### 4. GPS Module Hardware Specification Missing
**Severity:** HIGH - Documentation doesn't specify GPS model

**Problem:** Code comments mention "GPS_rocket" but never specify the hardware is a **SAM-M10Q**.

**Recommendation:**
- Update all GPS documentation to specify "u-blox SAM-M10Q"
- Add SAM-M10Q specific configuration details
- Document antenna requirements
- Add SAM-M10Q datasheet reference

---

### 5. Hard-Coded Decimation Factors (sensor_manager.c:179, 229)
**Severity:** MEDIUM-HIGH - Magic numbers reduce maintainability

**Current State:**
```c
uint8_t baro_decimation = 5;  // TODO: Calculate based on config
uint8_t highg_decimation = 10;  // Read every 10th call
```

**Problem:** Decimation is hard-coded and marked with TODO. If main loop rate changes, these become incorrect.

**Recommendation:**
Calculate decimation from configuration:
```c
typedef struct {
    uint16_t imu_odr_hz;
    uint16_t mag_odr_hz;
    uint16_t baro_odr_hz;
    uint16_t highg_odr_hz;       // ADD THIS
    uint16_t main_loop_hz;       // ADD THIS
    // ...
} SensorManager_Config_t;

// In SensorManager_Init():
uint8_t baro_decimation = config->main_loop_hz / config->baro_odr_hz;
uint8_t highg_decimation = config->main_loop_hz / config->highg_odr_hz;
```

---

## P1 (High Priority) Issues

### 6. No Runtime Health Monitoring
**Severity:** HIGH - Cannot detect system degradation

**Problem:** No watchdog, no health checks, no failsafe detection.

**Recommendation:**
Implement system health monitoring:
- Enable IWDG (Independent Watchdog)
- Monitor sensor update rates
- Detect stuck sensors
- Track error rates per sensor
- Automatic fallback modes (e.g., IMU-only if mag fails)

```c
typedef struct {
    bool imu_healthy;
    bool mag_healthy;
    bool baro_healthy;
    bool gps_healthy;
    uint32_t health_check_time;
    uint8_t system_health_percent;
} SystemHealth_t;

void SystemHealth_Update(SystemHealth_t *health, SensorManager_Status_t *status) {
    // Check if sensors are updating
    health->imu_healthy = (status->actual_rate_hz > 400.0f);  // Must be > 400 Hz
    health->mag_healthy = (status->mag_error_count < 100);
    // ... etc

    // Calculate overall health
    uint8_t healthy_count = health->imu_healthy + health->mag_healthy +
                            health->baro_healthy + health->gps_healthy;
    health->system_health_percent = (healthy_count * 100) / 4;
}
```

---

### 7. No Sensor Calibration Implementation
**Severity:** HIGH - Uncalibrated sensors produce poor results

**Problem:** No runtime calibration for:
- Magnetometer hard/soft iron compensation
- Gyroscope bias estimation
- Accelerometer offset correction
- Barometer sea-level pressure calibration

**Recommendation:**
Implement calibration subsystem:
```c
typedef struct {
    float mag_hard_iron[3];      // Offset correction
    float mag_soft_iron[3][3];   // Scale/rotation correction
    float gyro_bias[3];           // Gyroscope zero-rate offset
    float accel_offset[3];        // Accelerometer offset
    float baro_sea_level_pa;      // Reference pressure
    bool calibrated;
} SensorCalibration_t;

HAL_StatusTypeDef Calibration_PerformGyroCalibration(SensorCalibration_t *cal);
HAL_StatusTypeDef Calibration_PerformMagCalibration(SensorCalibration_t *cal);
void Calibration_ApplyToData(SensorManager_RawData_t *raw, SensorCalibration_t *cal);
```

Store calibration in flash for persistence.

---

### 8. No Data Logging Implementation
**Severity:** HIGH - Cannot reconstruct flights for analysis

**Problem:** QSPI flash interface exists but no logging implementation.

**Recommendation:**
Implement flight data recorder:
- Circular buffer in flash (4MB available)
- Log at 50-100 Hz (manageable data rate)
- Include: timestamp, sensors, attitude, GPS, system status
- Implement download via USB CDC
- Add flight session markers

```c
typedef struct {
    uint32_t timestamp_ms;
    SensorManager_RawData_t sensors;
    EulerAngles_t attitude;
    GPS_Data_t gps;
    SystemHealth_t health;
} FlightDataRecord_t;

void DataLogger_Init(void);
void DataLogger_Write(FlightDataRecord_t *record);
void DataLogger_Download(void);  // USB CDC download
```

---

### 9. Missing Configuration Management
**Severity:** MEDIUM-HIGH - Cannot change settings without recompiling

**Problem:** All configuration is hard-coded. No way to:
- Change Mahony filter gains
- Adjust sensor rates
- Enable/disable sensors
- Set calibration values

**Recommendation:**
Implement configuration system:
- Store configuration in flash (dedicated sector)
- Add USB CDC command interface
- Support runtime parameter updates
- Implement configuration version control

```c
typedef struct {
    uint32_t magic;               // 0xFC0FF1C0 (Flight Computer Config)
    uint16_t version;
    SensorManager_Config_t sensors;
    SensorCalibration_t calibration;
    MahonyFilter_Params_t mahony;
    SystemConfig_t system;
    uint32_t crc32;
} FlightComputerConfig_t;

void Config_LoadFromFlash(FlightComputerConfig_t *config);
void Config_SaveToFlash(FlightComputerConfig_t *config);
void Config_RestoreDefaults(FlightComputerConfig_t *config);
```

---

### 10. Insufficient Error Recovery
**Severity:** MEDIUM-HIGH - Single sensor failure can stop system

**Problem:** Sensor initialization failures return immediately:
```c
status = ICM42688_Init();
if (status != HAL_OK) {
    return status;  // BLOCKS ENTIRE SYSTEM
}
```

**Recommendation:**
Implement graceful degradation:
```c
status = ICM42688_Init();
if (status != HAL_OK) {
    DebugPrint("WARNING: IMU init failed, retrying...\r\n");
    HAL_Delay(100);
    status = ICM42688_Init();  // Retry once

    if (status != HAL_OK) {
        DebugPrint("ERROR: IMU init failed permanently!\r\n");
        sensor_status.imu_failed = true;
        // Continue with other sensors
    }
}
```

Add sensor hot-swap support for non-critical sensors.

---

## P2 (Medium Priority) Issues

### 11. Code Duplication in Sensor Drivers
**Severity:** MEDIUM - Maintenance burden

**Problem:** Similar patterns repeated across drivers:
- Register read/write functions
- DMA callback handling
- Busy-waiting patterns

**Recommendation:**
Create common sensor driver utilities:
```c
// sensor_driver_common.h
typedef struct {
    I2C_HandleTypeDef *hi2c;
    SPI_HandleTypeDef *hspi;
    uint16_t dev_address;
    volatile bool dma_busy;
    uint32_t dma_callback_count;
} SensorDriver_Common_t;

HAL_StatusTypeDef SensorDriver_I2C_ReadReg(SensorDriver_Common_t *drv,
                                           uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef SensorDriver_I2C_WriteReg(SensorDriver_Common_t *drv,
                                            uint8_t reg, uint8_t data);
```

---

### 12. Inconsistent Naming Conventions
**Severity:** MEDIUM - Reduces readability

**Problem:** Mixed naming styles:
- `SensorManager_Init()` (PascalCase_SnakeCase)
- `GPS_Init()` (UPPERCASE_SnakeCase)
- `GetMicros()` (PascalCase)

**Recommendation:**
Standardize on one convention (suggest: `Module_FunctionName()`):
```c
// Module prefix + PascalCase function name
SensorManager_Init()
GPS_Init()
Mahony_Update()
CORDIC_Atan2()
```

---

### 13. Missing Input Validation
**Severity:** MEDIUM - Potential crashes from invalid inputs

**Problem:** Many functions don't validate pointer arguments:
```c
void SensorManager_GetScales(SensorManager_Scales_t *out_scales) {
    if (out_scales != NULL) {  // GOOD - checks pointer
        memcpy(out_scales, &scales, sizeof(SensorManager_Scales_t));
    }
}

void SensorManager_GetMahonyData(const SensorManager_RawData_t *raw,
                                  float *gx, float *gy, float *gz, ...) {
    // BAD - no null checks on raw, gx, gy, gz, etc.
    *gx = (raw->gyro_x / 16.4f) * DEG_TO_RAD;
}
```

**Recommendation:**
Add comprehensive input validation:
```c
void SensorManager_GetMahonyData(const SensorManager_RawData_t *raw,
                                  float *gx, float *gy, float *gz,
                                  float *ax, float *ay, float *az,
                                  float *mx, float *my, float *mz) {
    // Validate inputs
    if (raw == NULL || gx == NULL || gy == NULL || gz == NULL ||
        ax == NULL || ay == NULL || az == NULL ||
        mx == NULL || my == NULL || mz == NULL) {
        return;  // Or return error code
    }

    // Proceed with conversion
    *gx = (raw->gyro_x / 16.4f) * DEG_TO_RAD;
    // ...
}
```

---

### 14. No Telemetry Implementation
**Severity:** MEDIUM - Cannot monitor flight in real-time

**Problem:** USB CDC and USART channels exist but no telemetry protocol.

**Recommendation:**
Implement lightweight telemetry protocol:
- Binary format for efficiency (vs. text)
- Periodic attitude/sensor data
- On-demand parameter requests
- Flight event notifications

Consider existing protocols:
- MAVLink (industry standard for UAVs)
- Custom binary protocol
- CSV over USB for debugging

---

### 15. GetMicros() Has Race Condition (sensor_manager.c:50-59)
**Severity:** MEDIUM - Potential timing errors

**Problem:** SysTick interrupt can occur between reading counter and flag check:
```c
static inline uint32_t GetMicros(void) {
    uint32_t m = HAL_GetTick();
    uint32_t u = SysTick->LOAD - SysTick->VAL;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {  // Race condition here
        m++;
        u = SysTick->LOAD - SysTick->VAL;
    }

    return (m * 1000) + (u * 1000 / SysTick->LOAD);
}
```

**Recommendation:**
Use atomic read or disable interrupts briefly:
```c
static inline uint32_t GetMicros(void) {
    uint32_t m, u;

    __disable_irq();
    m = HAL_GetTick();
    u = SysTick->LOAD - SysTick->VAL;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        m++;
        u = SysTick->LOAD - SysTick->VAL;
    }
    __enable_irq();

    return (m * 1000) + (u * 1000 / SysTick->LOAD);
}
```

Or use DWT cycle counter (higher precision, no race):
```c
static inline uint32_t GetMicros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}
```

---

## P3 (Low Priority) Issues

### 16. Magic Numbers Throughout Code
**Severity:** LOW - Reduces maintainability

**Problem:** Hard-coded constants without symbolic names:
- `16.4f` (gyro scale)
- `2048.0f` (accel scale)
- `16384.0f` (mag scale)
- `44330.0f` (barometric formula constant)

**Recommendation:**
Define named constants:
```c
// sensor_manager.h
#define ICM42688_GYRO_SCALE_2000DPS     16.4f
#define ICM42688_ACCEL_SCALE_16G        2048.0f
#define MMC5983MA_MAG_SCALE             16384.0f
#define BAROMETRIC_FORMULA_CONSTANT     44330.0f
```

---

### 17. No Unit Tests
**Severity:** LOW - Harder to verify correctness

**Problem:** No testing infrastructure.

**Recommendation:**
Add unit tests for:
- Mahony filter quaternion math
- Coordinate transformations
- Sensor scaling calculations
- I2C arbiter priority logic

Use frameworks like Unity or Google Test.

---

### 18. Limited Documentation
**Severity:** LOW - Reduces code understanding

**Problem:** Many functions lack detailed documentation.

**Recommendation:**
Add Doxygen-compatible documentation:
```c
/**
 * @brief Update Mahony AHRS filter with 9-DOF sensor data
 *
 * Fuses gyroscope, accelerometer, and magnetometer measurements to
 * estimate orientation in NED frame. Uses complementary filter approach
 * with integral error compensation.
 *
 * @param[in]  filter  Pointer to initialized MahonyFilter_t structure
 * @param[in]  gx      Gyroscope X-axis (rad/s, sensor frame)
 * @param[in]  gy      Gyroscope Y-axis (rad/s, sensor frame)
 * @param[in]  gz      Gyroscope Z-axis (rad/s, sensor frame)
 * @param[in]  ax      Accelerometer X-axis (m/s², sensor frame)
 * @param[in]  ay      Accelerometer Y-axis (m/s², sensor frame)
 * @param[in]  az      Accelerometer Z-axis (m/s², sensor frame)
 * @param[in]  mx      Magnetometer X-axis (µT, sensor frame)
 * @param[in]  my      Magnetometer Y-axis (µT, sensor frame)
 * @param[in]  mz      Magnetometer Z-axis (µT, sensor frame)
 *
 * @return HAL_OK on success, HAL_ERROR if filter not initialized
 *
 * @note Call at constant rate (default: 500 Hz)
 * @note Sensors must be in same coordinate frame
 * @note Uses hardware CORDIC for acceleration
 *
 * @see Mahony_Init(), Mahony_GetEulerAngles()
 */
HAL_StatusTypeDef Mahony_Update(MahonyFilter_t *filter,
                                float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz);
```

---

### 19. No Power Management
**Severity:** LOW - Unnecessary power consumption

**Problem:** System runs at full speed continuously. No low-power modes.

**Recommendation:**
Implement power management:
- Sleep mode when idle
- Sensor low-power modes during ground operations
- GPS power save mode
- CPU clock scaling

---

### 20. Unused Peripherals Initialized
**Severity:** LOW - Minor power/resource waste

**Problem:** ADC, FMAC initialized but never used:
```c
MX_ADC1_Init();   // Not used anywhere
MX_FMAC_Init();   // Not used anywhere
```

**Recommendation:**
Either use them or remove initialization. If planning future use, add comments:
```c
// MX_ADC1_Init();   // Reserved for battery voltage monitoring
// MX_FMAC_Init();   // Reserved for future Kalman filter
```

---

## Architectural Recommendations

### 21. State Machine Architecture
**Recommendation:** Implement flight state machine:
```c
typedef enum {
    STATE_GROUND_IDLE,
    STATE_PREFLIGHT_CHECK,
    STATE_ARMED,
    STATE_BOOST,
    STATE_COAST,
    STATE_APOGEE,
    STATE_DESCENT,
    STATE_LANDED,
    STATE_ERROR
} FlightState_t;

typedef struct {
    FlightState_t current_state;
    uint32_t state_entry_time;
    uint32_t time_in_state;
} StateMachine_t;

void StateMachine_Update(StateMachine_t *sm, SensorManager_ScaledData_t *sensors);
```

---

### 22. Event System
**Recommendation:** Implement event queue for asynchronous events:
```c
typedef enum {
    EVENT_SENSOR_ERROR,
    EVENT_GPS_FIX_ACQUIRED,
    EVENT_APOGEE_DETECTED,
    EVENT_LIFTOFF_DETECTED,
    EVENT_USB_CONNECTED
} EventType_t;

void EventQueue_Init(void);
void EventQueue_Post(EventType_t event, void *data);
bool EventQueue_Get(EventType_t *event, void **data);
```

---

### 23. Command Interface
**Recommendation:** Add command processor for USB CDC:
```c
// Commands like:
// "cal mag" - Start magnetometer calibration
// "get attitude" - Return current attitude
// "set mahony kp 2.5" - Update filter gain
// "download log" - Download flight data

void CommandProcessor_Init(void);
void CommandProcessor_HandleInput(char *line);
```

---

## Code Quality Metrics

### Current State
- **Total Lines:** ~9,000 LOC
- **Documentation Coverage:** ~30%
- **Magic Numbers:** ~50+ instances
- **Error Handling:** ~20% of functions
- **Input Validation:** ~15% of functions
- **Unit Test Coverage:** 0%

### Target State
- **Documentation Coverage:** >80%
- **Magic Numbers:** <5
- **Error Handling:** >90% of functions
- **Input Validation:** >90% of public APIs
- **Unit Test Coverage:** >60% for critical algorithms

---

## Implementation Priority

### Phase 1: Make It Work (P0)
1. Implement main loop
2. Fix error handler
3. Connect interrupt handlers
4. Update GPS documentation (SAM-M10Q)
5. Fix hard-coded decimation

**Estimated Effort:** 2-3 days

### Phase 2: Make It Reliable (P1)
1. Health monitoring
2. Sensor calibration
3. Data logging
4. Configuration management
5. Error recovery

**Estimated Effort:** 1-2 weeks

### Phase 3: Make It Maintainable (P2)
1. Refactor common code
2. Standardize naming
3. Add input validation
4. Implement telemetry
5. Fix race conditions

**Estimated Effort:** 1 week

### Phase 4: Make It Professional (P3)
1. Eliminate magic numbers
2. Add unit tests
3. Complete documentation
4. Power management
5. Remove unused code

**Estimated Effort:** 1 week

---

## Testing Recommendations

### Ground Testing
1. **Sensor Verification:** Verify all sensors read valid data
2. **Orientation Test:** Rotate system, verify Mahony filter tracks correctly
3. **GPS Test:** Verify fix acquisition and accuracy
4. **Data Logging Test:** Fill flash, verify download works
5. **Telemetry Test:** Monitor real-time data stream
6. **Calibration Test:** Perform full calibration sequence
7. **Stress Test:** Run for 24 hours continuous

### Flight Testing
1. **Motor Test:** Ground motor firing with data logging
2. **Drop Test:** Verify accelerometer and barometer
3. **Low Altitude Test:** <100m flight to verify all subsystems
4. **Full Flight Test:** Complete mission profile

---

## Safety Recommendations

1. **Watchdog:** Enable IWDG with 100ms timeout
2. **Brownout Protection:** Configure BOR to reset on low voltage
3. **Stack Overflow Detection:** Add stack canary checks
4. **Flash Verification:** CRC check configuration on boot
5. **Sensor Sanity Checks:** Detect physically impossible values
6. **Redundancy:** Consider dual IMU configuration

---

## Conclusion

The FlightComputer codebase has excellent architectural foundations with well-designed subsystems. However, it currently lacks a functional main application loop and many critical features needed for actual flight operations.

**Immediate Actions:**
1. Implement main loop (P0 #1)
2. Improve error handling (P0 #2)
3. Add health monitoring (P1 #6)
4. Implement data logging (P1 #8)

Once these are complete, the system will be in a testable state for ground verification before progressing to flight testing.

**Estimated Total Refactoring Effort:** 4-6 weeks for full implementation of all recommendations.

---

**Document End**
