// File: Core/Src/gps_module.c (WITH DIAGNOSTICS)

/**
  ******************************************************************************
  * @file    gps_module.c
  * @brief   Enhanced GPS module implementation using GPS_rocket engine
  * @note    GPS_rocket core with data extraction bridge for compatibility
  * @version 3.0 - GPS_rocket integration with enhanced data extraction
  ******************************************************************************
  */

#include "gps_module.h"
#include "debug_utils.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// This makes the global DMA handle for USART3 RX, defined in usart.c, visible to this file.
extern DMA_HandleTypeDef hdma_usart3_rx;


/* === GPS_ROCKET CORE (unchanged from gps_rocket.c) === */

/* Configuration constants */
#define TARGET_BAUD_RATE            115200
#define CFG_RATE_MEAS               0x30210001
#define CFG_RATE_NAV                0x30210002
#define CFG_SIGNAL_GPS_ENA          0x1031001f
#define CFG_SIGNAL_GLO_ENA          0x10310025
#define CFG_SIGNAL_GAL_ENA          0x10310021
#define CFG_SIGNAL_BDS_ENA          0x10310022
#define CFG_SIGNAL_QZSS_ENA         0x10310024
#define CFG_NAVSPG_DYNMODEL         0x20110021
#define CFG_NAVSPG_SIGATTCOMP       0x20110023
#define CFG_NAVSPG_FIXMODE          0x20110011
#define CFG_NAVSPG_INFIL_MINELEV    0x201100a4
#define CFG_NAVSPG_INFIL_MINSVS     0x201100a1

/* NMEA disable keys - full list from GPS_rocket */
#define CFG_MSGOUT_NMEA_ID_GGA_UART1    0x209100bb
#define CFG_MSGOUT_NMEA_ID_GLL_UART1    0x209100ca
#define CFG_MSGOUT_NMEA_ID_GSA_UART1    0x209100c0
#define CFG_MSGOUT_NMEA_ID_GSV_UART1    0x209100c5
#define CFG_MSGOUT_NMEA_ID_RMC_UART1    0x209100ac
#define CFG_MSGOUT_NMEA_ID_VTG_UART1    0x209100b1
#define CFG_MSGOUT_NMEA_ID_ZDA_UART1    0x209100d9
#define CFG_MSGOUT_NMEA_ID_GBS_UART1    0x209100de
#define CFG_MSGOUT_NMEA_ID_GST_UART1    0x209100d4
#define CFG_MSGOUT_NMEA_ID_GNS_UART1    0x209100b6
#define CFG_MSGOUT_NMEA_ID_GRS_UART1    0x209100cf
#define CFG_MSGOUT_NMEA_ID_VLW_UART1    0x209100e8
#define CFG_MSGOUT_NMEA_ID_RLM_UART1    0x20910401
#define CFG_MSGOUT_NMEA_ID_DTM_UART1    0x209100a7
#define CFG_MSGOUT_NMEA_ID_TXT_UART1    0x209100ec
#define CFG_MSGOUT_PUBX_ID_POLYP_UART1  0x209100ed
#define CFG_MSGOUT_PUBX_ID_POLYS_UART1  0x209100f2
#define CFG_MSGOUT_PUBX_ID_POLYT_UART1  0x209100f7

#define CFG_LAYER_RAM               0x01
#define CFG_LAYER_BBR               0x02
#define DYNMODEL_AIRBORNE_4G        0x08
#define FIXMODE_3D_ONLY             0x02

#define GPS_RX_BUFFER_SIZE          1024

/* === DATA EXTRACTION BRIDGE === */

static GPS_Data_t gps_data = {0};
static bool gps_initialized = false;

/* === GPS_ROCKET STREAMING VARIABLES === */

static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static uint16_t buffer_read_pos = 0;
static bool streaming_active = false;

// --- NEW VARIABLE ---
// This will store a snapshot of the DMA write position.
// 'volatile' is crucial because it's updated by an ISR and read by the main loop.
static volatile uint16_t last_dma_write_pos = 0;
// --------------------

/* UBX message parser (from GPS_rocket) */
typedef struct {
    uint8_t state;
    uint16_t bytes_received;
    uint16_t payload_length;
    uint8_t msg_class, msg_id;
    uint8_t checksum_a, checksum_b;
    uint8_t message_buffer[512];
} UBX_Parser_t;
static UBX_Parser_t ubx_parser = {0};

/* === GPS_ROCKET CORE FUNCTIONS (unchanged) === */

static void GPS_FlushUARTBuffer(void);
static bool GPS_SendUBXCommand(uint8_t* cmd, uint16_t cmd_len, uint32_t timeout_ms);
static bool GPS_TestCommunication(void);
static void GPS_ProcessUBXByte(uint8_t byte);
static void GPS_HandleCompleteMessage(void);
static bool GPS_EnableMessageLegacy(uint8_t msg_class, uint8_t msg_id);

/* Enhanced message parsing - extract data AND optionally print */
static void GPS_ParseAndExtractNAVPVT(uint8_t* payload, uint16_t len);
static void GPS_ParseAndExtractNAVCOV(uint8_t* payload, uint16_t len);
static void GPS_ParseAndExtractNAVDOP(uint8_t* payload, uint16_t len);
static void GPS_ParseAndExtractNAVVELNED(uint8_t* payload, uint16_t len);
static void GPS_ParseAndExtractNAVSTATUS(uint8_t* payload, uint16_t len);

bool GPS_SetBaudRate115200(void);
bool GPS_ConfigureRocketMode(void);
bool GPS_DisableAllNMEA(void);
bool GPS_EnableEKFMessages(void);
bool GPS_StartStreaming(void);
bool GPS_SendCfgValSet(uint32_t key, uint32_t value, uint8_t layer);

/* === COMPATIBILITY API IMPLEMENTATION === */

/**
 * @brief Initialize GPS module - uses GPS_rocket engine
 */
bool GPS_Init(void) {
    DebugPrint("GPS: Initializing GPS_rocket engine...\r\n");

    // Clear data structure
    memset(&gps_data, 0, sizeof(gps_data));
    gps_data.fix_status = 'V';

    // Hardware reset
    HAL_GPIO_WritePin(RST_SAM_GPIO_Port, RST_SAM_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(RST_SAM_GPIO_Port, RST_SAM_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);

    GPS_FlushUARTBuffer();
    HAL_Delay(500);

    // GPS_rocket initialization sequence
    bool baud_ok = GPS_SetBaudRate115200();
    bool config_ok = GPS_ConfigureRocketMode();
    bool nmea_ok = GPS_DisableAllNMEA();
    bool messages_ok = GPS_EnableEKFMessages();
    bool stream_ok = GPS_StartStreaming();

    // Use all variables to avoid warnings
    (void)baud_ok;
    (void)config_ok;
    (void)nmea_ok;
    (void)messages_ok;

    if (stream_ok) {
        gps_initialized = true;
        DebugPrint("GPS: ✅ GPS_rocket initialization successful\r\n");
        return true;
    } else {
        gps_initialized = false;
        DebugPrint("GPS: ❌ GPS_rocket initialization failed\r\n");
        return false;
    }
}

/**
 * @brief Update GPS data - uses GPS_rocket engine (MODIFIED LOGIC)
 */
bool GPS_Update(void) {
    if (!gps_initialized || !streaming_active) {
        return false;
    }

    // --- MODIFIED ---
    // Make a local, non-volatile copy of the write position. This prevents the
    // value from changing in the middle of our processing loop.
    uint16_t current_write_pos = last_dma_write_pos;
    // ----------------

    uint8_t messages_processed = 0;

    // We no longer need the diagnostic print here, as the logic is now safer.

    // Process all bytes from the last read position up to our STABLE snapshot position.
    while (buffer_read_pos != current_write_pos) {
        uint8_t byte = gps_rx_buffer[buffer_read_pos];
        uint8_t old_state = ubx_parser.state;
        GPS_ProcessUBXByte(byte);
        buffer_read_pos = (buffer_read_pos + 1) % GPS_RX_BUFFER_SIZE;
        if (old_state == 8 && ubx_parser.state == 0) {
            messages_processed++;
        }
    }
    return messages_processed > 0;
}

/**
 * @brief Get current GPS data
 */
bool GPS_GetCurrentData(GPS_Data_t* gps_data_out) {
    if (!gps_data_out || !gps_initialized) {
        return false;
    }

    *gps_data_out = gps_data;
    return true;
}

/**
 * @brief Check if GPS data is valid
 */
bool GPS_IsDataValid(void) {
    if (!gps_initialized) {
        return false;
    }

    bool recent_update = (HAL_GetTick() - gps_data.last_update) < 2000;
    return gps_data.data_valid && (gps_data.fix_status == 'A') && recent_update;
}

/**
 * @brief Get GPS fix status
 */
char GPS_GetFixStatus(void) {
    return gps_data.fix_status;
}

/**
 * @brief Get satellite count
 */
int GPS_GetSatelliteCount(void) {
    return gps_data.satellites;
}

/**
 * @brief Check if position data available
 */
bool GPS_HasPosition(void) {
    return gps_initialized && gps_data.data_valid;
}

/**
 * @brief Check if time/date available
 */
bool GPS_HasDateTime(void) {
    return gps_initialized && gps_data.time_valid;
}

/**
 * @brief Get last update timestamp
 */
uint32_t GPS_GetLastUpdateTime(void) {
    return gps_data.last_update;
}

/* === ENHANCED ACCESSORS === */

/**
 * @brief Get position standard deviations
 */
bool GPS_GetPositionStandardDeviations(float* std_north, float* std_east, float* std_down) {
    if (!std_north || !std_east || !std_down || !gps_data.pos_cov_valid) {
        return false;
    }

    *std_north = sqrtf(gps_data.pos_cov_nn);
    *std_east = sqrtf(gps_data.pos_cov_ee);
    *std_down = sqrtf(gps_data.pos_cov_dd);
    return true;
}

/**
 * @brief Get velocity standard deviations
 */
bool GPS_GetVelocityStandardDeviations(float* std_north, float* std_east, float* std_down) {
    if (!std_north || !std_east || !std_down || !gps_data.vel_cov_valid) {
        return false;
    }

    *std_north = sqrtf(gps_data.vel_cov_nn);
    *std_east = sqrtf(gps_data.vel_cov_ee);
    *std_down = sqrtf(gps_data.vel_cov_dd);
    return true;
}

/**
 * @brief Check if spoofing detected
 */
bool GPS_IsSpoofingDetected(void) {
    return gps_data.spoofing_state >= 2;
}

/**
 * @brief Get fix quality string
 */
const char* GPS_GetFixQualityString(void) {
	if (gps_data.fix_status == 'A') {
        return "GPS";
    } else {
        return "NO_FIX";
    }
}

/* === GPS_ROCKET CORE IMPLEMENTATION (unchanged) === */

static void GPS_FlushUARTBuffer(void) {
    uint8_t dummy;
    HAL_UART_AbortReceive(&huart3);
    HAL_Delay(50);

    int drained = 0;
    while (HAL_UART_Receive(&huart3, &dummy, 1, 1) == HAL_OK && drained < 500) {
        drained++;
    }
}

static bool GPS_TestCommunication(void) {
    uint8_t ubx_mon_ver[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

    GPS_FlushUARTBuffer();
    HAL_Delay(100);

    HAL_StatusTypeDef result = HAL_UART_Transmit(&huart3, ubx_mon_ver, sizeof(ubx_mon_ver), 1000);
    if (result != HAL_OK) return false;

    uint8_t response[50];
    int bytes_received = 0;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < 1000 && bytes_received < sizeof(response)) {
        if (HAL_UART_Receive(&huart3, &response[bytes_received], 1, 10) == HAL_OK) {
            bytes_received++;
            if (bytes_received >= 8 && response[0] == 0xB5 && response[1] == 0x62) {
                return true;
            }
        }
    }

    return false;
}

static bool GPS_SendUBXCommand(uint8_t* cmd, uint16_t cmd_len, uint32_t timeout_ms) {
    GPS_FlushUARTBuffer();

    HAL_StatusTypeDef result = HAL_UART_Transmit(&huart3, cmd, cmd_len, 1000);
    if (result != HAL_OK) return false;

    uint32_t start_time = HAL_GetTick();
    uint8_t state = 0;

    while ((HAL_GetTick() - start_time) < timeout_ms) {
        uint8_t byte;
        if (HAL_UART_Receive(&huart3, &byte, 1, 10) == HAL_OK) {
            switch (state) {
                case 0: if (byte == 0xB5) state = 1; break;
                case 1: if (byte == 0x62) state = 2; else state = 0; break;
                case 2: if (byte == 0x05) state = 3; else state = 0; break;
                case 3: return (byte == 0x01);
            }
        }
    }
    return false;
}

bool GPS_SetBaudRate115200(void) {
    uint32_t original_baud = huart3.Init.BaudRate;

    if (!GPS_TestCommunication()) {
        return false;
    }

    char pubx_cmd[80];
    snprintf(pubx_cmd, sizeof(pubx_cmd), "$PUBX,41,1,0007,0001,%d,0*", TARGET_BAUD_RATE);

    uint8_t checksum = 0;
    for (int i = 1; i < strlen(pubx_cmd) - 1; i++) {
        checksum ^= pubx_cmd[i];
    }
    snprintf(pubx_cmd + strlen(pubx_cmd) - 1, 6, "%02X\r\n", checksum);

    HAL_UART_Transmit(&huart3, (uint8_t*)pubx_cmd, strlen(pubx_cmd), 1000);

    HAL_Delay(2000);
    huart3.Init.BaudRate = TARGET_BAUD_RATE;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        huart3.Init.BaudRate = original_baud;
        HAL_UART_Init(&huart3);
        return false;
    }

    HAL_Delay(1000);
    GPS_FlushUARTBuffer();

    if (GPS_TestCommunication()) {
        return true;
    }

    huart3.Init.BaudRate = original_baud;
    HAL_UART_Init(&huart3);
    return false;
}

bool GPS_ConfigureRocketMode(void) {
    int success_count = 0;

    if (GPS_SendCfgValSet(CFG_RATE_MEAS, 100, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_RATE_NAV, 1, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_SIGNAL_GPS_ENA, 1, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_SIGNAL_BDS_ENA, 1, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_SIGNAL_QZSS_ENA, 1, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_SIGNAL_GLO_ENA, 0, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_SIGNAL_GAL_ENA, 0, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_NAVSPG_DYNMODEL, DYNMODEL_AIRBORNE_4G, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_NAVSPG_SIGATTCOMP, 1, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_NAVSPG_FIXMODE, FIXMODE_3D_ONLY, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_NAVSPG_INFIL_MINELEV, 10, CFG_LAYER_RAM)) success_count++;
    if (GPS_SendCfgValSet(CFG_NAVSPG_INFIL_MINSVS, 4, CFG_LAYER_RAM)) success_count++;

    return success_count >= 10;
}

bool GPS_DisableAllNMEA(void) {
    const uint32_t nmea_keys[18] = {
        CFG_MSGOUT_NMEA_ID_GGA_UART1, CFG_MSGOUT_NMEA_ID_GLL_UART1,
        CFG_MSGOUT_NMEA_ID_GSA_UART1, CFG_MSGOUT_NMEA_ID_GSV_UART1,
        CFG_MSGOUT_NMEA_ID_RMC_UART1, CFG_MSGOUT_NMEA_ID_VTG_UART1,
        CFG_MSGOUT_NMEA_ID_ZDA_UART1, CFG_MSGOUT_NMEA_ID_GBS_UART1,
        CFG_MSGOUT_NMEA_ID_GST_UART1, CFG_MSGOUT_NMEA_ID_GNS_UART1,
        CFG_MSGOUT_NMEA_ID_GRS_UART1, CFG_MSGOUT_NMEA_ID_VLW_UART1,
        CFG_MSGOUT_NMEA_ID_RLM_UART1, CFG_MSGOUT_NMEA_ID_DTM_UART1,
        CFG_MSGOUT_NMEA_ID_TXT_UART1, CFG_MSGOUT_PUBX_ID_POLYP_UART1,
        CFG_MSGOUT_PUBX_ID_POLYS_UART1, CFG_MSGOUT_PUBX_ID_POLYT_UART1
    };

    int success_count = 0;
    for (int i = 0; i < 18; i++) {
        if (GPS_SendCfgValSet(nmea_keys[i], 0, CFG_LAYER_RAM)) {
            success_count++;
        }
        HAL_Delay(200);
    }

    HAL_Delay(3000);
    GPS_FlushUARTBuffer();
    return success_count >= 16;
}

bool GPS_EnableEKFMessages(void) {
    struct {
        uint8_t msg_class;
        uint8_t msg_id;
    } ekf_messages[] = {
        {0x01, 0x07}, // NAV-PVT
        {0x01, 0x36}, // NAV-COV
        {0x01, 0x04}, // NAV-DOP
        {0x01, 0x12}, // NAV-VELNED
        {0x01, 0x03}  // NAV-STATUS
    };

    int success_count = 0;
    for (int i = 0; i < 5; i++) {
        if (GPS_EnableMessageLegacy(ekf_messages[i].msg_class, ekf_messages[i].msg_id)) {
            success_count++;
        } else {
        }
        HAL_Delay(500);
    }

    return success_count >= 4;
}

static bool GPS_EnableMessageLegacy(uint8_t msg_class, uint8_t msg_id) {
    uint8_t cfg_msg[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        msg_class, msg_id,
        0x00, 0x01, 0x01, 0x01, 0x01, 0x01,
        0x00, 0x00
    };

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 14; i++) {
        ck_a += cfg_msg[i];
        ck_b += ck_a;
    }
    cfg_msg[14] = ck_a;
    cfg_msg[15] = ck_b;

    return GPS_SendUBXCommand(cfg_msg, sizeof(cfg_msg), 2000);
}

bool GPS_StartStreaming(void) {
    memset(&ubx_parser, 0, sizeof(ubx_parser));
    buffer_read_pos = 0;

    HAL_UART_AbortReceive(&huart3);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_rx_buffer, GPS_RX_BUFFER_SIZE) != HAL_OK) {
        streaming_active = false;
        DebugPrint("GPS: ❌ HAL_UARTEx_ReceiveToIdle_DMA failed\r\n");
        return false;
    }

    // Disable the half-transfer interrupt which we don't need
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    streaming_active = true;
    return true;
}

bool GPS_SendCfgValSet(uint32_t key, uint32_t value, uint8_t layer) {
    uint8_t ubx_cfg_valset[22] = {
        0xB5, 0x62, 0x06, 0x8A, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    int payload_size = 8;
    uint8_t key_size = (key >> 28) & 0x0F;
    switch(key_size) {
        case 1: case 2: value &= 1; payload_size += 1; break;
        case 3: payload_size += 2; break;
        case 4: payload_size += 4; break;
        default: return false;
    }

    ubx_cfg_valset[4] = payload_size & 0xFF;
    ubx_cfg_valset[5] = (payload_size >> 8) & 0xFF;
    ubx_cfg_valset[6] = 0x00;
    ubx_cfg_valset[7] = layer;
    ubx_cfg_valset[8] = 0x00;
    ubx_cfg_valset[9] = 0x00;

    ubx_cfg_valset[10] = (key >> 0) & 0xFF;
    ubx_cfg_valset[11] = (key >> 8) & 0xFF;
    ubx_cfg_valset[12] = (key >> 16) & 0xFF;
    ubx_cfg_valset[13] = (key >> 24) & 0xFF;

    ubx_cfg_valset[14] = (value >> 0) & 0xFF;
    if (payload_size > 9) ubx_cfg_valset[15] = (value >> 8) & 0xFF;
    if (payload_size > 10) ubx_cfg_valset[16] = (value >> 16) & 0xFF;
    if (payload_size > 11) ubx_cfg_valset[17] = (value >> 24) & 0xFF;

    uint8_t ck_a = 0, ck_b = 0;
    int total_len = 6 + payload_size;
    for (int i = 2; i < total_len; i++) {
        ck_a += ubx_cfg_valset[i];
        ck_b += ck_a;
    }
    ubx_cfg_valset[total_len] = ck_a;
    ubx_cfg_valset[total_len + 1] = ck_b;

    return GPS_SendUBXCommand(ubx_cfg_valset, total_len + 2, 2000);
}

/* === UBX MESSAGE PROCESSING === */

static void GPS_ProcessUBXByte(uint8_t byte) {
    switch (ubx_parser.state) {
        case 0:
            if (byte == 0xB5) {
                ubx_parser.message_buffer[0] = byte;
                ubx_parser.bytes_received = 1;
                ubx_parser.state = 1;
            }
            break;
        case 1:
            if (byte == 0x62) {
                ubx_parser.message_buffer[1] = byte;
                ubx_parser.bytes_received = 2;
                ubx_parser.state = 2;
            } else if (byte == 0xB5) {
                ubx_parser.message_buffer[0] = byte;
                ubx_parser.bytes_received = 1;
            } else {
                ubx_parser.state = 0;
            }
            break;
        case 2:
            ubx_parser.msg_class = byte;
            ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
            ubx_parser.checksum_a = byte;
            ubx_parser.checksum_b = byte;
            ubx_parser.state = 3;
            break;
        case 3:
            ubx_parser.msg_id = byte;
            ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
            ubx_parser.checksum_a += byte;
            ubx_parser.checksum_b += ubx_parser.checksum_a;
            ubx_parser.state = 4;
            break;
        case 4:
            ubx_parser.payload_length = byte;
            ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
            ubx_parser.checksum_a += byte;
            ubx_parser.checksum_b += ubx_parser.checksum_a;
            ubx_parser.state = 5;
            break;
        case 5:
            ubx_parser.payload_length |= (byte << 8);
            ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
            ubx_parser.checksum_a += byte;
            ubx_parser.checksum_b += ubx_parser.checksum_a;
            if (ubx_parser.payload_length > 500) {
                ubx_parser.state = 0;
            } else {
                ubx_parser.state = (ubx_parser.payload_length > 0) ? 6 : 7;
            }
            break;
        case 6:
            ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
            ubx_parser.checksum_a += byte;
            ubx_parser.checksum_b += ubx_parser.checksum_a;
            if (ubx_parser.bytes_received >= (6 + ubx_parser.payload_length)) {
                ubx_parser.state = 7;
            }
            break;
        case 7:
            if (byte == ubx_parser.checksum_a) {
                ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
                ubx_parser.state = 8;
            } else {
                ubx_parser.state = 0;
            }
            break;
        case 8:
            if (byte == ubx_parser.checksum_b) {
                ubx_parser.message_buffer[ubx_parser.bytes_received++] = byte;
                GPS_HandleCompleteMessage();
            }
            ubx_parser.state = 0;
            break;
    }
}

static void GPS_HandleCompleteMessage(void) {
    if (ubx_parser.msg_class == 0x01) {
        uint8_t* payload = &ubx_parser.message_buffer[6];
        uint16_t payload_len = ubx_parser.payload_length;

        switch (ubx_parser.msg_id) {
            case 0x07: GPS_ParseAndExtractNAVPVT(payload, payload_len); break;
            case 0x36: GPS_ParseAndExtractNAVCOV(payload, payload_len); break;
            case 0x04: GPS_ParseAndExtractNAVDOP(payload, payload_len); break;
            case 0x12: GPS_ParseAndExtractNAVVELNED(payload, payload_len); break;
            case 0x03: GPS_ParseAndExtractNAVSTATUS(payload, payload_len); break;
        }
    }
}

/* === ENHANCED MESSAGE PARSING - Extract data instead of printing === */

static void GPS_ParseAndExtractNAVPVT(uint8_t* payload, uint16_t len) {
    if (len < 84) return;

    uint16_t year = *(uint16_t*)&payload[4];
    uint8_t month = payload[6], day = payload[7];
    uint8_t hour = payload[8], minute = payload[9], second = payload[10];
    uint8_t valid = payload[11], fixType = payload[20], numSV = payload[23];

    int32_t lon = *(int32_t*)&payload[24];
    int32_t lat = *(int32_t*)&payload[28];
    int32_t hMSL = *(int32_t*)&payload[36];
    uint32_t hAcc = *(uint32_t*)&payload[40];
    uint32_t vAcc = *(uint32_t*)&payload[44];

    int32_t velN = *(int32_t*)&payload[48];
    int32_t velE = *(int32_t*)&payload[52];
    int32_t velD = *(int32_t*)&payload[56];
    uint32_t sAcc = *(uint32_t*)&payload[68];

    // Extract to gps_data structure
    gps_data.latitude = lat * 1e-7f;
    gps_data.longitude = lon * 1e-7f;
    gps_data.altitude = hMSL / 1000.0f;
    gps_data.velN = velN / 1000.0f;
    gps_data.velE = velE / 1000.0f;
    gps_data.velD = velD / 1000.0f;
    gps_data.hAcc = hAcc / 1000.0f;
    gps_data.vAcc = vAcc / 1000.0f;
    gps_data.sAcc = sAcc / 1000.0f;

    gps_data.speed = sqrtf(gps_data.velN * gps_data.velN + gps_data.velE * gps_data.velE);
    gps_data.heading = atan2f(gps_data.velE, gps_data.velN);
    if (gps_data.heading < 0) gps_data.heading += 360.0f;
    gps_data.climb_rate = -gps_data.velD;

    gps_data.satellites = numSV;
    gps_data.fix_status = (fixType >= 3) ? 'A' : 'V';
    gps_data.data_valid = (fixType >= 3);
    gps_data.time_valid = (valid & 0x03) == 0x03;
    gps_data.date_valid = gps_data.time_valid;

    gps_data.gps_year = year;
    gps_data.gps_month = month;
    gps_data.gps_day = day;
    gps_data.gps_hour = hour;
    gps_data.gps_minute = minute;
    gps_data.gps_second = second;

    gps_data.last_update = HAL_GetTick();
}

static void GPS_ParseAndExtractNAVCOV(uint8_t* payload, uint16_t len) {
    if (len < 64) return;

    bool posCovValid = payload[5];
    bool velCovValid = payload[6];

    if (posCovValid) {
        gps_data.pos_cov_nn = *(float*)&payload[16];
        gps_data.pos_cov_ee = *(float*)&payload[28];
        gps_data.pos_cov_dd = *(float*)&payload[36];
        gps_data.pos_cov_valid = true;
    } else {
        gps_data.pos_cov_valid = false;
    }

    if (velCovValid) {
        gps_data.vel_cov_nn = *(float*)&payload[40];
        gps_data.vel_cov_ee = *(float*)&payload[52];
        gps_data.vel_cov_dd = *(float*)&payload[60];
        gps_data.vel_cov_valid = true;
    } else {
        gps_data.vel_cov_valid = false;
    }
}

static void GPS_ParseAndExtractNAVDOP(uint8_t* payload, uint16_t len) {
    if (len < 18) return;

    gps_data.gdop = (*(uint16_t*)&payload[4]) * 0.01f;
    gps_data.pdop = (*(uint16_t*)&payload[6]) * 0.01f;
    gps_data.tdop = (*(uint16_t*)&payload[8]) * 0.01f;
    gps_data.vdop = (*(uint16_t*)&payload[10]) * 0.01f;
    gps_data.hdop = (*(uint16_t*)&payload[12]) * 0.01f;
    gps_data.ndop = (*(uint16_t*)&payload[14]) * 0.01f;
    gps_data.edop = (*(uint16_t*)&payload[16]) * 0.01f;
}

static void GPS_ParseAndExtractNAVVELNED(uint8_t* payload, uint16_t len) {
    if (len < 36) return;

    uint32_t cAcc = *(uint32_t*)&payload[32];
    gps_data.course_acc = cAcc * 1e-5f;
}

static void GPS_ParseAndExtractNAVSTATUS(uint8_t* payload, uint16_t len) {
    if (len < 16) return;

    uint8_t flags = payload[5];
    uint8_t fixStat = payload[6];
    uint8_t flags2 = payload[7];
    uint32_t ttff = *(uint32_t*)&payload[8];

    bool diffSoln = (flags & 0x02) != 0;
    bool diffCorr = (fixStat & 0x01) != 0;
    uint8_t spoofDetState = (flags2 >> 3) & 0x03;
    uint8_t psmState = flags2 & 0x03;

    gps_data.differential_solution = diffSoln || diffCorr;
    gps_data.spoofing_state = spoofDetState;
    gps_data.power_state = psmState;
    gps_data.time_to_first_fix = ttff;
}


/**
 * @brief UART RX Event callback - handles GPS (USART3) DMA idle line detection
 * @note Called when UART idle line is detected (message boundary)
 * @note BLE (USART1) now uses interrupt-based reception (HAL_UART_RxCpltCallback)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;  /* Size parameter not used in circular mode */

    if (huart->Instance == USART3)
    {
        /* GPS: Update DMA write position on idle line detection */
        last_dma_write_pos = GPS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    }
    /* Note: BLE (USART1) no longer uses this callback - it uses HAL_UART_RxCpltCallback for interrupt-based RX */
}


/* === LEGACY COMPATIBILITY === */
bool InitializeGPS(void) { return GPS_Init(); }
bool ReadGPSData(void) { return GPS_Update(); }
