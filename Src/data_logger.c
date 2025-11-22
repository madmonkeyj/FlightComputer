/**
  ******************************************************************************
  * @file    data_logger.c
  * @brief   Data logging module - OPTIMIZED 192-BYTE STRUCTURE
  ******************************************************************************
  */

#include "data_logger.h"
#include "quadspi.h"
#include "ble_module.h"
#include "gps_module.h"
#include "battery_monitor.h"
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

/**
 * @brief Pack data into optimized record structure with EKF debugging data
 */
static void PackDataRecord(DataRecord_t* record, const NavigationSolution_t* nav_solution, const SensorData_t* sensor_data) {
    if (!record || !nav_solution || !sensor_data) {
        return;
    }

    // Clear record completely
    memset(record, 0, sizeof(DataRecord_t));

    // === HEADER ===
    record->timestamp_ms = HAL_GetTick();

    // === CRITICAL SENSOR DATA === (reduced from raw sensor data)
    if (sensor_data->accel_valid) {
        record->accel[0] = sensor_data->accel[0];
        record->accel[1] = sensor_data->accel[1];
        record->accel[2] = sensor_data->accel[2];
    }

    if (sensor_data->gyro_valid) {
        record->gyro[0] = sensor_data->gyro[0];
        record->gyro[1] = sensor_data->gyro[1];
        record->gyro[2] = sensor_data->gyro[2];
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

        // Enhanced: Include all velocity components for complete debugging
        record->gps_vel_n = gps_data.velN;
        record->gps_vel_e = gps_data.velE;
        record->gps_vel_d = gps_data.velD;
    } else {
        record->gps_fix_status = 'V';
        record->gps_hdop = 99.9f;
    }

    // === NAVIGATION DATA ===
    record->quat[0] = nav_solution->attitude.q0;
    record->quat[1] = nav_solution->attitude.q1;
    record->quat[2] = nav_solution->attitude.q2;
    record->quat[3] = nav_solution->attitude.q3;

    memcpy(record->pos_ned, nav_solution->position_ned, sizeof(record->pos_ned));
    memcpy(record->vel_ned, nav_solution->velocity_ned, sizeof(record->vel_ned));

    // === SYSTEM HEALTH ===
    record->nav_valid = nav_solution->navigation_valid ? 1 : 0;

    // === EKF UNCERTAINTY ESTIMATES ===
    NavigationEKF_t* ekf = NavigationManager_GetEKF();
    if (ekf && nav_solution->navigation_valid) {
        record->pos_uncertainty[0] = sqrtf(ekf->P[0][0]);
        record->pos_uncertainty[1] = sqrtf(ekf->P[1][1]);
        record->pos_uncertainty[2] = sqrtf(ekf->P[2][2]);
        record->vel_uncertainty[0] = sqrtf(ekf->P[3][3]);
        record->vel_uncertainty[1] = sqrtf(ekf->P[4][4]);
        record->vel_uncertainty[2] = sqrtf(ekf->P[5][5]);

        // === **FIXED: INNOVATION VALUES** ===
        if (ekf->debug.innovation_pos_valid) {
            memcpy(record->innovation_pos, ekf->debug.innovation_pos, sizeof(record->innovation_pos));
        } else {
            for (int i = 0; i < 3; i++) record->innovation_pos[i] = NAN;
        }

        if (ekf->debug.innovation_vel_valid) {
            memcpy(record->innovation_vel, ekf->debug.innovation_vel, sizeof(record->innovation_vel));
        } else {
            for (int i = 0; i < 3; i++) record->innovation_vel[i] = NAN;
        }

        // === **FIXED: KALMAN GAINS** ===
        if (ekf->debug.kalman_gain_pos_valid) {
            memcpy(record->kalman_gain_pos, ekf->debug.kalman_gain_pos, sizeof(record->kalman_gain_pos));
        } else {
            for (int i = 0; i < 3; i++) record->kalman_gain_pos[i] = NAN;
        }

        if (ekf->debug.kalman_gain_vel_valid) {
            memcpy(record->kalman_gain_vel, ekf->debug.kalman_gain_vel, sizeof(record->kalman_gain_vel));
        } else {
            for (int i = 0; i < 3; i++) record->kalman_gain_vel[i] = NAN;
        }

        // === **FIXED: COORDINATE TRANSFORM RESULTS** ===
        if (ekf->debug.accel_ned_valid) {
            memcpy(record->accel_ned, ekf->debug.accel_ned, sizeof(record->accel_ned));
        } else {
            for (int i = 0; i < 3; i++) record->accel_ned[i] = NAN;
        }

        // === **NEW: MEASUREMENT UPDATE FLAGS** ===
        record->gps_pos_rejected = ekf->debug.gps_pos_rejected;
        record->gps_vel_rejected = ekf->debug.gps_vel_rejected;
        record->zupt_applied = ekf->debug.zupt_applied;

        // === REDUCED MOTION DETECTION ===
        record->motion_state = (uint8_t)ekf->motion.current_state;
        record->gps_velocity_suspect = (gps_valid && record->motion_state == 0 && gps_data.speed > 2.0f) ? 1 : 0;

        // Clear debug flags for next cycle
        ekf->debug.innovation_pos_valid = false;
        ekf->debug.innovation_vel_valid = false;
        ekf->debug.kalman_gain_pos_valid = false;
        ekf->debug.kalman_gain_vel_valid = false;
        ekf->debug.gps_pos_rejected = 0;
        ekf->debug.gps_vel_rejected = 0;
        ekf->debug.zupt_applied = 0;

    } else {
        // No EKF data - set defaults
        for (int i = 0; i < 3; i++) {
            record->pos_uncertainty[i] = 999.9f;
            record->vel_uncertainty[i] = 999.9f;
            record->innovation_pos[i] = NAN;
            record->innovation_vel[i] = NAN;
            record->kalman_gain_pos[i] = NAN;
            record->kalman_gain_vel[i] = NAN;
            record->accel_ned[i] = NAN;
        }
        record->motion_state = 3; // UNKNOWN
        record->gps_velocity_suspect = 0;
        record->gps_pos_rejected = 0;
        record->gps_vel_rejected = 0;
        record->zupt_applied = 0;
    }
}

/**
 * @brief Initialize data logger
 */
bool DataLogger_Init(void) {
    // Test flash communication
    uint8_t flash_id[3] = {0};

    if (QSPI_Read_ID(flash_id) != HAL_OK) {
        if (QSPI_Reset_After_Error() != HAL_OK) {
            logger_status = LOGGER_ERROR;
            return false;
        }
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
 * @brief Record data
 */
bool DataLogger_RecordData(const NavigationSolution_t* nav_solution, const SensorData_t* sensor_data) {
    if (logger_status != LOGGER_RECORDING || !flash_initialized) {
        return false;
    }

    if (!nav_solution || !sensor_data) {
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

    // Pack data into record
    PackDataRecord(&temp_record, nav_solution, sensor_data);

    // Critical validation - timestamp must be valid
    if (temp_record.timestamp_ms == 0) {
        return false;
    }

    // Write to flash
    HAL_StatusTypeDef write_result = CSP_QSPI_WriteMemory((uint8_t*)&temp_record, current_write_address, sizeof(DataRecord_t));

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

    // Erase data area only
    if (CSP_QSPI_EraseSector(0, DATA_AREA_SIZE - 1) != HAL_OK) {
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
    if (CSP_QSPI_EraseSector(METADATA_SECTOR_ADDR, METADATA_SECTOR_ADDR + METADATA_SECTOR_SIZE - 1) != HAL_OK) {
        return false;
    }

    if (CSP_QSPI_WriteMemory((uint8_t*)&current_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    // Verify
    FlashMetadata_t verify_metadata;
    if (CSP_QSPI_Read((uint8_t*)&verify_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
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
    if (CSP_QSPI_Read((uint8_t*)&loaded_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
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
