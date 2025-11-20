/**
  ******************************************************************************
  * @file    gps_module.h
  * @brief   Enhanced GPS module for model rocket EKF integration
  * @note    GPS_rocket engine with enhanced UBX data extraction
  * @version 3.0 - GPS_rocket integration with enhanced data fields
  ******************************************************************************
  */

#ifndef GPS_MODULE_H_
#define GPS_MODULE_H_

#include "main.h"
#include <stdbool.h>
#include <math.h>

/**
 * @brief Enhanced GPS data structure with comprehensive UBX fields
 * @note Includes all original fields plus new covariance, DOP, and status data
 */
typedef struct {
    // === BASIC POSITION DATA ===
    float latitude;              // Latitude in degrees
    float longitude;             // Longitude in degrees
    float altitude;              // Height above MSL (meters)

    // === BASIC VELOCITY DATA ===
    float speed;                 // Ground speed (m/s)
    float heading;               // Heading of motion (radians)

    // === UBX VELOCITY COMPONENTS (for EKF) ===
    float velN;                  // North velocity component (m/s)
    float velE;                  // East velocity component (m/s)
    float velD;                  // Down velocity component (m/s)

    // === UBX ACCURACY ESTIMATES (for EKF) ===
    float hAcc;                  // Horizontal position accuracy (meters)
    float vAcc;                  // Vertical position accuracy (meters)
    float sAcc;                  // Speed accuracy (m/s)

    // === ENHANCED: COVARIANCE DATA (from NAV-COV) ===
    float pos_cov_nn;            // North position variance (mÂ²)
    float pos_cov_ee;            // East position variance (mÂ²)
    float pos_cov_dd;            // Down position variance (mÂ²)
    float vel_cov_nn;            // North velocity variance (mÂ²/sÂ²)
    float vel_cov_ee;            // East velocity variance (mÂ²/sÂ²)
    float vel_cov_dd;            // Down velocity variance (mÂ²/sÂ²)
    bool pos_cov_valid;          // Position covariance data valid
    bool vel_cov_valid;          // Velocity covariance data valid

    // === ENHANCED: COMPLETE DOP VALUES (from NAV-DOP) ===
    float pdop;                  // Position DOP
    float hdop;                  // Horizontal DOP
    float vdop;                  // Vertical DOP
    float gdop;                  // Geometric DOP
    float tdop;                  // Time DOP
    float ndop;                  // Northing DOP
    float edop;                  // Easting DOP

    // === ENHANCED: COURSE ACCURACY (from NAV-VELNED) ===
    float course_acc;            // Course accuracy (degrees)
    float climb_rate;            // Climb rate (m/s, positive = climbing)

    // === ENHANCED: SPOOFING/JAMMING DETECTION (from NAV-STATUS) ===
    uint8_t spoofing_state;      // 0=unknown, 1=no spoofing, 2=spoofing, 3=multiple spoofing
    uint8_t jamming_state;       // Jamming indicator

    // === ENHANCED: POWER MANAGEMENT (from NAV-STATUS) ===
    uint8_t power_state;         // 0=acquisition, 1=tracking, 2=power_opt, 3=inactive

    // === ENHANCED: FIX QUALITY (from NAV-STATUS) ===
    bool differential_solution; // DGPS corrections applied
    uint32_t time_to_first_fix; // Time to first fix (ms)

    // === SATELLITE INFORMATION ===
    int satellites;              // Number of satellites in use
    char fix_status;             // 'A' = valid, 'V' = invalid

    // === TIME/DATE INFORMATION ===
    int gps_hour;
    int gps_minute;
    int gps_second;
    int gps_day;
    int gps_month;
    int gps_year;

    // === DATA VALIDITY FLAGS ===
    bool time_valid;             // Time is valid
    bool date_valid;             // Date is valid
    bool data_valid;             // Overall data validity (from UBX fixType and flags)

    // === TIMING ===
    uint32_t last_update;        // HAL_GetTick() of last update

} GPS_Data_t;

// === COMPATIBILITY API (unchanged interface) ===

/**
 * @brief Initialize GPS module for rocket flight
 * @return true if initialization successful
 * @note Now uses GPS_rocket engine with comprehensive UBX configuration
 */
bool GPS_Init(void);

/**
 * @brief Update GPS data by reading UBX messages
 * @return true if new UBX data was received and parsed
 * @note Now uses GPS_rocket's interrupt-based streaming
 */
bool GPS_Update(void);

/**
 * @brief Get current GPS data with enhanced UBX fields
 * @param gps_data Pointer to GPS_Data_t structure to fill
 * @return true if data is available
 * @note Now includes covariance, complete DOP, and status data
 */
bool GPS_GetCurrentData(GPS_Data_t* gps_data);

/**
 * @brief Check if GPS data is currently valid for navigation
 * @return true if GPS has valid 3D fix and recent data
 */
bool GPS_IsDataValid(void);

/**
 * @brief Get GPS fix status
 * @return 'A' for valid 3D fix, 'V' for invalid
 */
char GPS_GetFixStatus(void);

/**
 * @brief Get number of satellites in solution
 * @return Number of satellites used in navigation solution
 */
int GPS_GetSatelliteCount(void);

/**
 * @brief Check if GPS position data is available
 * @return true if latitude/longitude are valid
 */
bool GPS_HasPosition(void);

/**
 * @brief Check if GPS time/date data is available
 * @return true if time and date are valid
 */
bool GPS_HasDateTime(void);

/**
 * @brief Get timestamp of last GPS update
 * @return HAL_GetTick() timestamp of last successful UBX message
 */
uint32_t GPS_GetLastUpdateTime(void);

// === ENHANCED ACCESSORS (new functions for enhanced data) ===

/**
 * @brief Get position standard deviations (from covariance)
 * @param std_north Pointer to North position std dev (meters)
 * @param std_east Pointer to East position std dev (meters)
 * @param std_down Pointer to Down position std dev (meters)
 * @return true if covariance data valid
 */
bool GPS_GetPositionStandardDeviations(float* std_north, float* std_east, float* std_down);

/**
 * @brief Get velocity standard deviations (from covariance)
 * @param std_north Pointer to North velocity std dev (m/s)
 * @param std_east Pointer to East velocity std dev (m/s)
 * @param std_down Pointer to Down velocity std dev (m/s)
 * @return true if covariance data valid
 */
bool GPS_GetVelocityStandardDeviations(float* std_north, float* std_east, float* std_down);

/**
 * @brief Check if spoofing/jamming detected
 * @return true if spoofing or jamming detected
 */
bool GPS_IsSpoofingDetected(void);

/**
 * @brief Get fix quality string
 * @return String describing fix quality (GPS, DGPS, RTK-FLOAT, RTK-FIXED)
 */
const char* GPS_GetFixQualityString(void);

// === INLINE UTILITY FUNCTIONS (unchanged) ===

/**
 * @brief Get North velocity component (for EKF)
 */
static inline float GPS_GetVelocityNorth(const GPS_Data_t* data) {
    return data->velN;
}

/**
 * @brief Get East velocity component (for EKF)
 */
static inline float GPS_GetVelocityEast(const GPS_Data_t* data) {
    return data->velE;
}

/**
 * @brief Get Down velocity component (for EKF)
 */
static inline float GPS_GetVelocityDown(const GPS_Data_t* data) {
    return data->velD;
}

/**
 * @brief Get horizontal position accuracy (for EKF covariance)
 */
static inline float GPS_GetHorizontalAccuracy(const GPS_Data_t* data) {
    return data->hAcc;
}

/**
 * @brief Get speed accuracy (for EKF covariance)
 */
static inline float GPS_GetSpeedAccuracy(const GPS_Data_t* data) {
    return data->sAcc;
}

/**
 * @brief Get 3D speed magnitude
 */
static inline float GPS_Get3DSpeed(const GPS_Data_t* data) {
    return sqrtf(data->velN * data->velN + data->velE * data->velE + data->velD * data->velD);
}

// === LEGACY COMPATIBILITY ===
bool InitializeGPS(void);   // Calls GPS_Init()
bool ReadGPSData(void);     // Calls GPS_Update()

// === UART CALLBACK FUNCTION ===
/**
 * @brief GPS UART callback - called from main UART callback
 * @note Call this from HAL_UART_RxCpltCallback when huart == &huart3
 */
void GPS_UART_RxCallback(void);

#endif /* GPS_MODULE_H_ */
