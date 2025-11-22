/**
  ******************************************************************************
  * @file    data_logger.c
  * @brief   Data logging module - 192-BYTE STRUCTURE
  * @note    Adapted for current FlightComputer codebase (no EKF)
  * @note    Uses SensorManager, Mahony filter, and GPS module
  ******************************************************************************
  */

#include "data_logger.h"
#include "quadspi.h"
#include "ble_module.h"
#include "gps_module.h"
#include "sensor_manager.h"
#include "mahony_filter.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Configuration Constants */
#define EXTERNAL_FLASH_SIZE     (4 * 1024 * 1024)  // 4MB total
#define TARGET_RECORDING_TIME_MS (30 * 60 * 1000)  // 30 minutes
#define RECORDING_INTERVAL_MS   (TARGET_RECORDING_TIME_MS / (DATA_AREA_SIZE / RECORD_SIZE))
#define SECTOR_SIZE             4096

// Global metadata instance
static FlashMetadata_t current_metadata = {0};
static bool metadata_loaded = false;

/* Private Variables */
static LoggerStatus_t logger_status = LOGGER_IDLE;
static bool flash_initialized = false;
static uint32_t current_write_address = 0;
static uint32_t records_written = 0;
static uint32_t recording_start_time = 0;
static uint32_t last_record_time = 0;
static uint32_t last_recording_attempt = 0;

/* Temporary record buffer */
static DataRecord_t temp_record;

/* External Mahony filter instance - defined in main.c or where Mahony is initialized */
extern Mahony_Filter_t mahony_filter;

/**
 * @brief Erase a range of flash memory by erasing all sectors in range
 * @param start_addr: Start address (will be aligned to 4KB)
 * @param end_addr: End address
 * @return HAL_OK on success, HAL_ERROR on failure
 */
static HAL_StatusTypeDef EraseFlashRange(uint32_t start_addr, uint32_t end_addr) {
    // Align start to sector boundary
    uint32_t sector_start = (start_addr / SECTOR_SIZE) * SECTOR_SIZE;
    uint32_t sector_end = ((end_addr + SECTOR_SIZE - 1) / SECTOR_SIZE) * SECTOR_SIZE;

    for (uint32_t addr = sector_start; addr < sector_end; addr += SECTOR_SIZE) {
        if (QSPI_Simple_Erase(addr) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief Pack data into 192-byte record structure (adapted for current system)
 * @note Uses SensorManager, Mahony filter, and GPS module
 * @note EKF fields are set to zero/NaN (no EKF in current system)
 */
static void PackDataRecord(DataRecord_t* record) {
    if (!record) {
        return;
    }

    // Clear record completely
    memset(record, 0, sizeof(DataRecord_t));

    // === HEADER ===
    record->timestamp_ms = HAL_GetTick();

    // === SENSOR DATA === Get from SensorManager
    SensorManager_ScaledData_t sensor_data;
    if (SensorManager_ReadScaled(&sensor_data)) {
        // Accelerometer (m/s²)
        if (sensor_data.imu_accel_valid) {
            record->accel[0] = sensor_data.imu_accel_x;
            record->accel[1] = sensor_data.imu_accel_y;
            record->accel[2] = sensor_data.imu_accel_z;
        }

        // Gyroscope (rad/s)
        if (sensor_data.imu_gyro_valid) {
            record->gyro[0] = sensor_data.imu_gyro_x;
            record->gyro[1] = sensor_data.imu_gyro_y;
            record->gyro[2] = sensor_data.imu_gyro_z;
        }
    }

    // === GPS DATA ===
    GPS_Data_t gps_data = {0};
    bool gps_valid = GPS_GetCurrentData(&gps_data);

    if (gps_valid) {
        record->gps_lat = gps_data.latitude;
        record->gps_lon = gps_data.longitude;
        record->gps_alt = gps_data.altitude;
        record->gps_speed = gps_data.speed;
        record->gps_satellites = gps_data.satellites;
        record->gps_fix_status = gps_data.fix_status;
        record->gps_hdop = gps_data.hdop;

        // GPS velocity components (NED frame)
        record->gps_vel_n = gps_data.velN;
        record->gps_vel_e = gps_data.velE;
        record->gps_vel_d = gps_data.velD;
    } else {
        record->gps_fix_status = 'V';  // Invalid
        record->gps_hdop = 99.9f;
    }

    // === MAHONY ATTITUDE === Get quaternion from Mahony filter
    Mahony_GetQuaternion(&mahony_filter,
                         &record->quat[0], &record->quat[1],
                         &record->quat[2], &record->quat[3]);

    // === NO EKF - Set position/velocity to zero ===
    // pos_ned[3] = {0, 0, 0}
    // vel_ned[3] = {0, 0, 0}
    // Already zeroed by memset

    // === SYSTEM HEALTH ===
    // Use GPS validity as navigation health indicator
    record->nav_valid = gps_valid ? 1 : 0;

    // === EKF FIELDS - Not available, set to NaN or zero ===
    // These maintain 192-byte structure for downloader compatibility
    for (int i = 0; i < 3; i++) {
        record->pos_uncertainty[i] = NAN;
        record->vel_uncertainty[i] = NAN;
        record->innovation_pos[i] = NAN;
        record->innovation_vel[i] = NAN;
        record->kalman_gain_pos[i] = NAN;
        record->kalman_gain_vel[i] = NAN;
        record->accel_ned[i] = NAN;
    }

    record->gps_pos_rejected = 0;
    record->gps_vel_rejected = 0;
    record->zupt_applied = 0;
    record->motion_state = 3;  // UNKNOWN
    record->gps_velocity_suspect = 0;
}

/**
 * @brief Initialize data logger
 */
bool DataLogger_Init(void) {
    // Test flash communication
    uint8_t flash_id[3] = {0};

    if (QSPI_Read_ID(flash_id) != HAL_OK) {
        // Retry after delay (no reset function available)
        HAL_Delay(100);
        if (QSPI_Read_ID(flash_id) != HAL_OK) {
            logger_status = LOGGER_ERROR;
            return false;
        }
    }

    flash_initialized = true;

    // Load or create metadata
    if (Metadata_Load()) {
        // Previous session restored
    } else {
        Metadata_Clear();
        current_metadata.recording_session_id = 1;
        Metadata_Save();
    }

    logger_status = LOGGER_IDLE;
    return true;
}

/**
 * @brief Start recording
 */
bool DataLogger_StartRecording(void) {
    if (!flash_initialized) {
        return false;
    }

    if (logger_status == LOGGER_RECORDING) {
        return true;
    }

    // Reset recording state
    current_write_address = 0;
    records_written = 0;
    recording_start_time = HAL_GetTick();
    last_record_time = 0;
    last_recording_attempt = 0;
    logger_status = LOGGER_RECORDING;

    // Update metadata
    current_metadata.recording_session_id++;
    Metadata_Save();

    return true;
}

/**
 * @brief Stop recording
 */
bool DataLogger_StopRecording(void) {
    if (logger_status != LOGGER_RECORDING) {
        return true;
    }

    logger_status = LOGGER_IDLE;
    Metadata_Save();
    return true;
}

/**
 * @brief Record current sensor/GPS/attitude data to flash
 * @note Reads data from SensorManager, Mahony filter, and GPS module
 */
bool DataLogger_RecordData(void) {
    if (logger_status != LOGGER_RECORDING || !flash_initialized) {
        return false;
    }

    uint32_t current_time = HAL_GetTick();

    // Rate limiting
    if (current_time - last_recording_attempt < RECORDING_INTERVAL_MS) {
        return true;
    }
    last_recording_attempt = current_time;

    // Check flash space
    if (current_write_address + sizeof(DataRecord_t) > DATA_AREA_SIZE) {
        DataLogger_StopRecording();
        return false;
    }

    // Pack data into record (reads from current system state)
    PackDataRecord(&temp_record);

    // Critical validation - timestamp must be valid
    if (temp_record.timestamp_ms == 0) {
        return false;
    }

    // Write to flash using DMA for efficiency
    HAL_StatusTypeDef write_result = QSPI_Quad_Write_DMA((uint8_t*)&temp_record, current_write_address, sizeof(DataRecord_t));

    if (write_result != HAL_OK) {
        logger_status = LOGGER_ERROR;
        return false;
    }

    // Update state
    current_write_address += sizeof(DataRecord_t);
    records_written++;
    last_record_time = current_time;

    // Periodic metadata save
    if (records_written % 50 == 0) {
        Metadata_Save();
    }

    return true;
}

/**
 * @brief Update function (for compatibility)
 */
void DataLogger_Update(void) {
    // Empty function for compatibility with existing main loop calls
}

/**
 * @brief Get logger statistics
 */
bool DataLogger_GetStats(LoggerStats_t* stats) {
    if (!stats) {
        return false;
    }

    stats->status = logger_status;
    stats->records_written = records_written;
    stats->flash_bytes_used = current_write_address;
    stats->recording_start_time = recording_start_time;
    stats->last_record_time = last_record_time;
    stats->recording_rate_hz = 1000 / RECORDING_INTERVAL_MS;
    stats->flash_ready = flash_initialized;
    stats->record_size = sizeof(DataRecord_t);

    // Calculate time remaining
    if (logger_status == LOGGER_RECORDING) {
        uint32_t bytes_remaining = DATA_AREA_SIZE - current_write_address;
        uint32_t records_remaining = bytes_remaining / sizeof(DataRecord_t);
        stats->estimated_time_remaining_ms = records_remaining * RECORDING_INTERVAL_MS;
    } else {
        stats->estimated_time_remaining_ms = 0;
    }

    return true;
}

/**
 * @brief Get status string
 */
bool DataLogger_GetStatusString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size < 100) {
        return false;
    }

    LoggerStats_t stats;
    if (!DataLogger_GetStats(&stats)) {
        return false;
    }

    const char* status_str;
    switch (stats.status) {
        case LOGGER_IDLE: status_str = "IDLE"; break;
        case LOGGER_RECORDING: status_str = "RECORDING"; break;
        case LOGGER_DOWNLOADING: status_str = "DOWNLOADING"; break;
        case LOGGER_ERROR: status_str = "ERROR"; break;
        default: status_str = "UNKNOWN"; break;
    }

    float flash_usage_percent = (float)(stats.flash_bytes_used * 100) / EXTERNAL_FLASH_SIZE;

    if (stats.status == LOGGER_RECORDING) {
        snprintf(buffer, buffer_size,
                "Status: %s, Records: %lu (%.1f%%), Rate: %luHz, Time left: %.1fmin",
                status_str, stats.records_written, flash_usage_percent,
                stats.recording_rate_hz, stats.estimated_time_remaining_ms / 60000.0f);
    } else {
        snprintf(buffer, buffer_size,
                "Status: %s, Records: %lu (%.1f%% used), Flash: %s, RecordSize: %lu bytes",
                status_str, stats.records_written, flash_usage_percent,
                stats.flash_ready ? "Ready" : "Error", stats.record_size);
    }

    if (memcmp(&current_metadata, &verify_metadata, sizeof(FlashMetadata_t)) != 0) {
        return false;
    }

    metadata_loaded = true;
    return true;
}

bool Metadata_Load(void) {
    if (!flash_initialized) {
        return false;
    }

    FlashMetadata_t loaded_metadata;
    if (QSPI_Quad_Read((uint8_t*)&loaded_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    if (!Metadata_Validate(&loaded_metadata)) {
        return false;
    }

    // Restore state
    current_metadata = loaded_metadata;
    records_written = current_metadata.records_written;
    current_write_address = current_metadata.current_write_address;
    recording_start_time = current_metadata.recording_start_time;
    last_record_time = current_metadata.last_record_time;
    logger_status = (LoggerStatus_t)current_metadata.logger_status;

    // Don't resume recording after power cycle
    if (logger_status == LOGGER_RECORDING) {
        logger_status = LOGGER_IDLE;
    }

    metadata_loaded = true;
    return true;
}

/**
 * @brief Check if recording
 */
bool DataLogger_IsRecording(void) {
    return logger_status == LOGGER_RECORDING;
}

/**
 * @brief Check if downloading
 */
bool DataLogger_IsDownloading(void) {
    return logger_status == LOGGER_DOWNLOADING;
}

/**
 * @brief Erase all data
 */
bool DataLogger_EraseAll(void) {
    if (!flash_initialized) {
        return false;
    }

    if (logger_status == LOGGER_RECORDING || logger_status == LOGGER_DOWNLOADING) {
        return false;
    }

    // Erase data area only (this will take time - ~10-30 seconds for 4MB)
    if (EraseFlashRange(0, DATA_AREA_SIZE - 1) != HAL_OK) {
        return false;
    }

    // Reset state
    current_write_address = 0;
    records_written = 0;
    recording_start_time = 0;
    last_record_time = 0;

    Metadata_Save();
    return true;
}

/* === METADATA FUNCTIONS === */

uint32_t Metadata_CalculateChecksum(const FlashMetadata_t* metadata) {
    if (!metadata) return 0;

    uint32_t checksum = 0;
    const uint8_t* data = (const uint8_t*)metadata;

    for (int i = 0; i < (sizeof(FlashMetadata_t) - sizeof(uint32_t)); i++) {
        checksum += data[i];
    }
    return checksum;
}

bool Metadata_Validate(const FlashMetadata_t* metadata) {
    if (!metadata) return false;

    if (metadata->magic_number != METADATA_MAGIC) return false;
    if (metadata->version != METADATA_VERSION) return false;
    if (metadata->records_written > (DATA_AREA_SIZE / sizeof(DataRecord_t))) return false;
    if (metadata->current_write_address > DATA_AREA_SIZE) return false;

    uint32_t calculated_checksum = Metadata_CalculateChecksum(metadata);
    return (metadata->checksum == calculated_checksum);
}

bool Metadata_Save(void) {
    if (!flash_initialized) {
        return false;
    }

    // Update metadata
    current_metadata.magic_number = METADATA_MAGIC;
    current_metadata.version = METADATA_VERSION;
    current_metadata.records_written = records_written;
    current_metadata.current_write_address = current_write_address;
    current_metadata.recording_start_time = recording_start_time;
    current_metadata.last_record_time = last_record_time;
    current_metadata.logger_status = (uint8_t)logger_status;
    current_metadata.record_size = sizeof(DataRecord_t);
    current_metadata.checksum = Metadata_CalculateChecksum(&current_metadata);

    // Erase and write metadata
    if (QSPI_Simple_Erase(METADATA_SECTOR_ADDR) != HAL_OK) {
        return false;
    }

    // Write metadata (small, can use blocking write or DMA - using DMA for consistency)
    if (QSPI_Quad_Write_DMA((uint8_t*)&current_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    // Verify
    FlashMetadata_t verify_metadata;
    if (QSPI_Quad_Read((uint8_t*)&verify_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    if (memcmp(&current_metadata, &verify_metadata, sizeof(FlashMetadata_t)) != 0) {
        return false;
    }

    metadata_loaded = true;
    return true;
}

bool Metadata_Load(void) {
    if (!flash_initialized) {
        return false;
    }

    FlashMetadata_t loaded_metadata;
    if (QSPI_Quad_Read((uint8_t*)&loaded_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    if (!Metadata_Validate(&loaded_metadata)) {
        return false;
    }

    // Restore state
    current_metadata = loaded_metadata;
    records_written = current_metadata.records_written;
    current_write_address = current_metadata.current_write_address;
    recording_start_time = current_metadata.recording_start_time;
    last_record_time = current_metadata.last_record_time;
    logger_status = (LoggerStatus_t)current_metadata.logger_status;

    // Don't resume recording after power cycle
    if (logger_status == LOGGER_RECORDING) {
        logger_status = LOGGER_IDLE;
    }

    metadata_loaded = true;
    return true;
}

void Metadata_Clear(void) {
    memset(&current_metadata, 0, sizeof(FlashMetadata_t));
    metadata_loaded = false;
    records_written = 0;
    current_write_address = 0;
    recording_start_time = 0;
    last_record_time = 0;
}
