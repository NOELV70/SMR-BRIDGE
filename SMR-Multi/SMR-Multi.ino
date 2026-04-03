/*******************************************************************************
 * FILE:        SMR-Multi.ino
 * AUTHOR:      Noel Vellemans
 * VERSION:     6.2.0
 * LICENSE:     GNU General Public License v2.0 (GPLv2)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * -----------------------------------------------------------------------------
 * PROJECT CAPABILITIES AND ARCHITECTURE:
 *
 * 1. HARDWARE LAYER: 
 * Manipulates the ESP8266 U0C0 register to enable internal RX inversion.
 * This allows a direct electrical connection to the P1 port of a Smart Meter 
 * without requiring external NPN transistor inverters.
 *
 * 2. SERIAL-TO-TCP BRIDGE:
 * Establishes a high-performance transparent gateway. Incoming serial 
 * telegrams are aggregated into a 512-byte adaptive broadcast cache to 
 * minimize WiFi radio airtime and broadcast to up to 10 concurrent 
 * TCP clients on port 2001.
 *
 * 3. DUAL-WATCHDOG PROTECTION:
 * - Hardware: 8-second hardware timer to recover from CPU lockups.
 * - Software: Logic-based timeouts for WiFi loss (5 minutes), Serial silence 
 * (10 minutes), and TCP client inactivity (10 minutes).
 *
 * 4. DIAGNOSTIC INTERFACE:
 * - Dashboard: Real-time system health, uptime, and traffic metrics.
 * - RAW Page: Captures the most recent DSMR frame (delimited by '/' and '!')
 * for visual validation with 60-second auto-refresh and manual override.
 *
 * 5. MANAGEMENT:
 * - WiFiManager: Captive portal for credential provisioning.
 * - Auth: HTTP Digest authentication for all administrative actions.
 * - OTA: Support for both Arduino IDE and Web-browser firmware updates.
 * 
 * 6. MARSTEK EMULATION:
 * High-fidelity Shelly Pro 3EM emulation layer for native integration with 
 * Marstek B2500, Venus, and Jupiter battery storage systems. Skips the 
 * requirement for the Marstek P1 meter.
 * 
 * -----------------------------------------------------------------------------
 * CORE RELIABILITY: THE DUAL-WATCHDOG
 * 
 * To ensure the device never needs a manual power-plug reset, it uses two 
 * layers of protection:
 * 
 * • LAYER 1 (Hardware): An 8-second "Dead Man's Switch." If the code freezes, 
 *   the internal timer hits zero and forces a hard reboot.
 * 
 * • LAYER 2 (Software): Monitors the "health" of the connection. If WiFi is 
 *   lost for more than 5 minutes after the initial boot, the system triggers 
 *   a controlled reset to reconnect.
 * 
 * -----------------------------------------------------------------------------
 * TECHNICAL HIGHLIGHTS
 * 
 * • Zero-Hardware Inversion: Uses a clever register hack (U0C0) to invert the 
 *   Serial RX signal in software. This allows you to connect the Smart Meter 
 *   directly to the ESP8266 without an external transistor.
 * 
 * • Multi-Client Streaming: Broadcasts RAW P1 data to up to 10 TCP clients 
 *   simultaneously on port 2001 (ideal for Home Assistant, Domoticz, and a 
 *   raw terminal at the same time).
 * 
 * • Pro Diagnostics: A built-in web dashboard provides real-time uptime, RAM 
 *   usage, and a RAW Data viewer that captures and displays the exact telegram 
 *   frame from the meter.
 * 
 * • Secure Management: Features HTTP Digest Authentication to protect your 
 *   settings and supports OTA (Over-The-Air) updates so you can flash new 
 *   firmware without removing the device from the fuse box.
 * 
 * -----------------------------------------------------------------------------
 * VITAL STATS
 * 
 * Feature              Specification
 * TCP Port             2001
 * Max Clients          10
 * Default Admin        admin / admin
 * Hardware WDT         8 Seconds
 * WiFi Timeout         5 Minutes
 * Serial Timeout       10 Minutes
 * Client Timeout       10 Minutes
 ******************************************************************************/


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Updater.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// --- Forward Declarations for Shelly Emulator ---
// These must be at the top so the compiler recognizes the functions
// even if the header is included later in the file.
void setupShellyEmulator();
void loopShellyEmulator();
void handleShellyStatus();

//=============================================================================
// DSMR PARSER V2 - INTEGRATED WITH POWER FACTOR
//=============================================================================

// OBIS CODE DEFINITIONS - Stored in PROGMEM
const char OBIS_METER_ID[] PROGMEM       = "0-0:96.1.1";
const char OBIS_TIMESTAMP[] PROGMEM      = "0-0:1.0.0";
const char OBIS_POWER_LIMIT[] PROGMEM    = "0-0:17.0.0";
const char OBIS_ENERGY_DEL_LOW[] PROGMEM = "1-0:1.8.1";
const char OBIS_ENERGY_DEL_HIGH[] PROGMEM= "1-0:1.8.2";
const char OBIS_ENERGY_PROD_LOW[] PROGMEM= "1-0:2.8.1";
const char OBIS_ENERGY_PROD_HIGH[] PROGMEM="1-0:2.8.2";
const char OBIS_CURRENT_POWER[] PROGMEM  = "1-0:1.7.0";
const char OBIS_CURRENT_RETURN[] PROGMEM = "1-0:2.7.0";
const char OBIS_POWER_L1[] PROGMEM       = "1-0:21.7.0";
const char OBIS_POWER_L2[] PROGMEM       = "1-0:41.7.0";
const char OBIS_POWER_L3[] PROGMEM       = "1-0:61.7.0";
const char OBIS_RETURN_L1[] PROGMEM      = "1-0:22.7.0";
const char OBIS_RETURN_L2[] PROGMEM      = "1-0:42.7.0";
const char OBIS_RETURN_L3[] PROGMEM      = "1-0:62.7.0";
const char OBIS_VOLTAGE_L1[] PROGMEM     = "1-0:32.7.0";
const char OBIS_VOLTAGE_L2[] PROGMEM     = "1-0:52.7.0";
const char OBIS_VOLTAGE_L3[] PROGMEM     = "1-0:72.7.0";
const char OBIS_CURRENT_L1[] PROGMEM     = "1-0:31.7.0";
const char OBIS_CURRENT_L2[] PROGMEM     = "1-0:51.7.0";
const char OBIS_CURRENT_L3[] PROGMEM     = "1-0:71.7.0";
const char OBIS_PF_TOTAL[] PROGMEM       = "1-0:13.7.0";
const char OBIS_PF_L1[] PROGMEM          = "1-0:33.7.0";
const char OBIS_PF_L2[] PROGMEM          = "1-0:53.7.0";
const char OBIS_PF_L3[] PROGMEM          = "1-0:73.7.0";

// OBIS ID DEFINITIONS
enum obis_id_t : uint8_t {
    OBIS_ID_NONE = 0,
    OBIS_ID_METER_ID, OBIS_ID_TIMESTAMP, OBIS_ID_POWER_LIMIT,
    OBIS_ID_ENERGY_DEL_LOW, OBIS_ID_ENERGY_DEL_HIGH,
    OBIS_ID_ENERGY_PROD_LOW, OBIS_ID_ENERGY_PROD_HIGH,
    OBIS_ID_CURRENT_POWER, OBIS_ID_CURRENT_RETURN,
    OBIS_ID_POWER_L1, OBIS_ID_POWER_L2, OBIS_ID_POWER_L3,
    OBIS_ID_RETURN_L1, OBIS_ID_RETURN_L2, OBIS_ID_RETURN_L3,
    OBIS_ID_VOLTAGE_L1, OBIS_ID_VOLTAGE_L2, OBIS_ID_VOLTAGE_L3,
    OBIS_ID_CURRENT_L1, OBIS_ID_CURRENT_L2, OBIS_ID_CURRENT_L3,
    OBIS_ID_PF_TOTAL, OBIS_ID_PF_L1, OBIS_ID_PF_L2, OBIS_ID_PF_L3
};

// DSMR DATA STRUCTURE - Fixed-point for minimal RAM
typedef struct {
    // Energy counters (mWh = kWh * 1000000)
    uint64_t energy_delivered_low;
    uint64_t energy_delivered_high;
    uint64_t energy_produced_low;
    uint64_t energy_produced_high;

    // Instantaneous power (mW = kW * 1000000)
    int32_t current_power;
    int32_t current_return;

    // Per phase power (mW)
    int32_t power_l1, power_l2, power_l3;
    int32_t return_l1, return_l2, return_l3;

    // Per phase voltage (10mV = V * 100)
    int16_t voltage_l1, voltage_l2, voltage_l3;

    // Per phase current (10mA = A * 1000)
    int16_t current_l1, current_l2, current_l3;

    // Power factor stored as percentage * 100 (e.g., 0.955 -> 9550).
    // Widened to int32_t to prevent overflow (int16_t max 32767 was too small).
    // Negative values indicate leading (capacitive), positive = lagging (inductive).
    int32_t pf_total, pf_l1, pf_l2, pf_l3;

    // Meter info
    int32_t power_limit;
    char meter_id[33];
    char timestamp[14];
    char equipment_id[33];

    // Status
    uint8_t is_3phase    : 1;
    uint8_t frame_complete : 1;
    uint8_t has_pf_data  : 1;
} dsmr_data_t;

// LINE BUFFER PARSER
typedef struct {
    char     line_buffer[80];
    uint8_t  line_index;
    uint8_t  frame_active;
    // CRC16 validation fields
    uint16_t crc_calculated;
    uint8_t  crc_done;       // Set when '!' byte has been consumed into CRC
    dsmr_data_t *data;
} dsmr_parser_t;

// OBIS LOOKUP TABLE
typedef struct {
    const char *obis_string;
    uint8_t     obis_id;
} obis_entry_t;

const obis_entry_t OBIS_TABLE[] PROGMEM = {
    {OBIS_METER_ID,           OBIS_ID_METER_ID},
    {OBIS_TIMESTAMP,          OBIS_ID_TIMESTAMP},
    {OBIS_POWER_LIMIT,        OBIS_ID_POWER_LIMIT},
    {OBIS_ENERGY_DEL_LOW,     OBIS_ID_ENERGY_DEL_LOW},
    {OBIS_ENERGY_DEL_HIGH,    OBIS_ID_ENERGY_DEL_HIGH},
    {OBIS_ENERGY_PROD_LOW,    OBIS_ID_ENERGY_PROD_LOW},
    {OBIS_ENERGY_PROD_HIGH,   OBIS_ID_ENERGY_PROD_HIGH},
    {OBIS_CURRENT_POWER,      OBIS_ID_CURRENT_POWER},
    {OBIS_CURRENT_RETURN,     OBIS_ID_CURRENT_RETURN},
    {OBIS_POWER_L1,           OBIS_ID_POWER_L1},
    {OBIS_POWER_L2,           OBIS_ID_POWER_L2},
    {OBIS_POWER_L3,           OBIS_ID_POWER_L3},
    {OBIS_RETURN_L1,          OBIS_ID_RETURN_L1},
    {OBIS_RETURN_L2,          OBIS_ID_RETURN_L2},
    {OBIS_RETURN_L3,          OBIS_ID_RETURN_L3},
    {OBIS_VOLTAGE_L1,         OBIS_ID_VOLTAGE_L1},
    {OBIS_VOLTAGE_L2,         OBIS_ID_VOLTAGE_L2},
    {OBIS_VOLTAGE_L3,         OBIS_ID_VOLTAGE_L3},
    {OBIS_CURRENT_L1,         OBIS_ID_CURRENT_L1},
    {OBIS_CURRENT_L2,         OBIS_ID_CURRENT_L2},
    {OBIS_CURRENT_L3,         OBIS_ID_CURRENT_L3},
    {OBIS_PF_TOTAL,           OBIS_ID_PF_TOTAL},
    {OBIS_PF_L1,              OBIS_ID_PF_L1},
    {OBIS_PF_L2,              OBIS_ID_PF_L2},
    {OBIS_PF_L3,              OBIS_ID_PF_L3},
};
// Use count instead of null-sentinel loop guard.
#define OBIS_TABLE_COUNT (sizeof(OBIS_TABLE) / sizeof(obis_entry_t))

// PROGMEM String comparison
static bool strcmp_P_safe(const char *str, const char *pstr) {
    char c1, c2;
    do {
        c2 = pgm_read_byte(pstr++);
        c1 = *str++;
        if (c1 == '\0') return c2 == '\0';
    } while (c1 == c2);
    return false;
}

// Match OBIS code
static uint8_t match_obis_code(const char *obis) {
    // Iterate by explicit count, no null-pointer sentinel needed.
    for (uint8_t i = 0; i < OBIS_TABLE_COUNT; i++) {
        const char *obis_str = (const char*)pgm_read_dword(&OBIS_TABLE[i].obis_string);
        if (strcmp_P_safe(obis, obis_str)) {
            return pgm_read_byte(&OBIS_TABLE[i].obis_id);
        }
    }
    return OBIS_ID_NONE;
}

// CRC16/IBM (reflected) used by DSMR P1 telegrams.
// Polynomial 0x8005 in reflected form = 0xA001. Initial value 0x0000.
static uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA001;
        else         crc >>= 1;
    }
    return crc;
}

// Fast ASCII to fixed-point conversion.
// scale=6: result in µ-units  (raw, e.g. kWh -> µWh, kW -> µW)
// scale=3: result in m-units  (e.g. V   -> mV)
// scale=2: result in c-units  (e.g. V   -> cV, i.e. 10mV steps)
static int64_t ascii_to_fixed(const char *str, uint8_t scale) {
    int64_t result   = 0;
    int64_t fraction = 0;
    uint8_t decimal_seen = 0;
    uint8_t frac_digits  = 0;
    int8_t  sign = 1;

    if (*str == '-') { sign = -1; str++; }

    while (*str) {
        if (*str == '.') {
            decimal_seen = 1;
        } else if (*str >= '0' && *str <= '9') {
            if (!decimal_seen) {
                result = result * 10 + (*str - '0');
            } else if (frac_digits < 6) {
                fraction = fraction * 10 + (*str - '0');
                frac_digits++;
            }
        }
        str++;
    }

    while (frac_digits < 6) { fraction *= 10; frac_digits++; }

    result = result * 1000000 + fraction;

    if      (scale == 2) result /= 10000;
    else if (scale == 3) result /= 1000;
    // scale == 6: no division (µ-unit pass-through)

    return result * sign;
}

// extract_value no longer mutates the line buffer.
// Returns a pointer to a static internal buffer containing the null-terminated value.
static char* extract_value(const char *line) {
    static char value_buf[80];

    const char *start = strchr(line, '(');
    if (!start) return NULL;
    start++;

    const char *end  = strchr(start, ')');
    const char *unit = strchr(start, '*');

    size_t len;
    if (unit && (!end || unit < end)) {
        len = (size_t)(unit - start);
    } else if (end) {
        len = (size_t)(end - start);
    } else {
        return NULL;
    }

    if (len >= sizeof(value_buf)) len = sizeof(value_buf) - 1;
    memcpy(value_buf, start, len);
    value_buf[len] = '\0';

    return value_buf;
}

// Store parsed value
static void store_value(dsmr_data_t *data, uint8_t obis_type, const char *value_str) {
    if (obis_type == OBIS_ID_METER_ID) {
        strncpy(data->meter_id, value_str, sizeof(data->meter_id)-1);
        return;
    }
    if (obis_type == OBIS_ID_TIMESTAMP) {
        strncpy(data->timestamp, value_str, sizeof(data->timestamp)-1);
        return;
    }

    int64_t value;
    if (obis_type >= OBIS_ID_VOLTAGE_L1 && obis_type <= OBIS_ID_VOLTAGE_L3) {
        value = ascii_to_fixed(value_str, 2);
    } else if (obis_type >= OBIS_ID_CURRENT_L1 && obis_type <= OBIS_ID_CURRENT_L3) {
        value = ascii_to_fixed(value_str, 3);
    } else if (obis_type >= OBIS_ID_PF_TOTAL && obis_type <= OBIS_ID_PF_L3) {
        // Correct divisor. ascii_to_fixed(scale=6) gives µ-units, so
        // 0.955 -> 955000. Dividing by 100 gives 9550 = percentage * 100.
        // fixed_to_pf() then divides by 10000.0 to recover 0.955. ✓
        // pf_* fields are now int32_t so 9550 fits without overflow.
        value = ascii_to_fixed(value_str, 6) / 100;
        data->has_pf_data = 1;
    } else {
        value = ascii_to_fixed(value_str, 6);
    }

    switch (obis_type) {
        case OBIS_ID_POWER_LIMIT:      data->power_limit          = (int32_t)value;  break;
        case OBIS_ID_ENERGY_DEL_LOW:   data->energy_delivered_low  = (uint64_t)value; break;
        case OBIS_ID_ENERGY_DEL_HIGH:  data->energy_delivered_high = (uint64_t)value; break;
        case OBIS_ID_ENERGY_PROD_LOW:  data->energy_produced_low   = (uint64_t)value; break;
        case OBIS_ID_ENERGY_PROD_HIGH: data->energy_produced_high  = (uint64_t)value; break;
        case OBIS_ID_CURRENT_POWER:    data->current_power         = (int32_t)value;  break;
        case OBIS_ID_CURRENT_RETURN:   data->current_return        = (int32_t)value;  break;
        case OBIS_ID_POWER_L1:         data->power_l1              = (int32_t)value;  break;
        case OBIS_ID_POWER_L2:         data->power_l2              = (int32_t)value;  data->is_3phase = 1; break;
        case OBIS_ID_POWER_L3:         data->power_l3              = (int32_t)value;  data->is_3phase = 1; break;
        case OBIS_ID_RETURN_L1:        data->return_l1             = (int32_t)value;  break;
        case OBIS_ID_RETURN_L2:        data->return_l2             = (int32_t)value;  break;
        case OBIS_ID_RETURN_L3:        data->return_l3             = (int32_t)value;  break;
        case OBIS_ID_VOLTAGE_L1:       data->voltage_l1            = (int16_t)value;  break;
        case OBIS_ID_VOLTAGE_L2:       data->voltage_l2            = (int16_t)value;  data->is_3phase = 1; break;
        case OBIS_ID_VOLTAGE_L3:       data->voltage_l3            = (int16_t)value;  data->is_3phase = 1; break;
        case OBIS_ID_CURRENT_L1:       data->current_l1            = (int16_t)value;  break;
        case OBIS_ID_CURRENT_L2:       data->current_l2            = (int16_t)value;  data->is_3phase = 1; break;
        case OBIS_ID_CURRENT_L3:       data->current_l3            = (int16_t)value;  data->is_3phase = 1; break;
        case OBIS_ID_PF_TOTAL:         data->pf_total              = (int32_t)value;  break;
        case OBIS_ID_PF_L1:            data->pf_l1                 = (int32_t)value;  break;
        case OBIS_ID_PF_L2:            data->pf_l2                 = (int32_t)value;  break;
        case OBIS_ID_PF_L3:            data->pf_l3                 = (int32_t)value;  break;
    }
}

// Process one complete line.
// Note: process_line needs the full parser pointer (not just data) so it can
// access crc_calculated for CRC validation on the '!' line.
static void process_line(dsmr_parser_t *parser, char *line) {
    if (line[0] == '\0' || line[0] == '\r') return;

    if ((line[0] == '/' || (line[0] >= 'A' && line[0] <= 'Z')) && !strchr(line, ':')) {
        strncpy(parser->data->equipment_id, line, sizeof(parser->data->equipment_id)-1);
        return;
    }

    // Validate CRC before accepting the frame.
    // The '!' byte was already included in crc_calculated by dsmr_parse_byte.
    // Bytes after '!' (the 4 hex CRC digits) were excluded.
    // Line buffer contains "!XXXX" where XXXX is the received CRC in hex.
    if (line[0] == '!') {
        parser->frame_active = 0;
        if (strlen(line) >= 5) {
            // CRC present: validate it
            char hex[5] = { line[1], line[2], line[3], line[4], '\0' };
            uint16_t received_crc = (uint16_t)strtol(hex, NULL, 16);
            if (received_crc == parser->crc_calculated) {
                parser->data->frame_complete = 1;
            }
            // CRC mismatch: silently discard. dsmr_last_good is not updated.
        } else {
            // No CRC present (some non-standard meters omit it): accept the frame.
            parser->data->frame_complete = 1;
        }
        return;
    }

    char *paren = strchr(line, '(');
    if (!paren) return;

    *paren = '\0';
    uint8_t obis_type = match_obis_code(line);
    *paren = '(';

    if (obis_type == OBIS_ID_NONE) return;

    // extract_value now copies to a static buffer; line is not mutated.
    char *value = extract_value(line);
    if (value) {
        store_value(parser->data, obis_type, value);
    }
}

// Initialize parser
void dsmr_parser_init(dsmr_parser_t *parser, dsmr_data_t *data) {
    memset(parser, 0, sizeof(dsmr_parser_t));
    memset(data,   0, sizeof(dsmr_data_t));
    parser->data = data;
}

// Process one byte.
// The re-init pattern: after dsmr_parse_byte returns 1 (frame complete),
// the caller must call dsmr_parser_init again to reset frame_active and CRC
// state before the next telegram. This is done in processDataByte().
//
// CRC is accumulated here over all raw bytes (including \r\n) from
// the first '/' byte up to and including '!'. Bytes after '!' are the 4-char
// hex CRC value and are buffered but not hashed.
uint8_t dsmr_parse_byte(dsmr_parser_t *parser, uint8_t c) {
    if (!parser->frame_active) {
        if (c == '/' || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
            parser->frame_active    = 1;
            parser->crc_done        = 0;
            parser->crc_calculated  = crc16_update(0x0000, c);
            parser->line_index      = 0;
            parser->line_buffer[parser->line_index++] = c;
        }
        return 0;
    }

    // Accumulate CRC over all raw bytes up to and including '!'.
    // After '!', crc_done is set and further bytes (the hex CRC digits) are
    // buffered normally but excluded from the hash.
    if (!parser->crc_done) {
        parser->crc_calculated = crc16_update(parser->crc_calculated, c);
        if (c == '!') parser->crc_done = 1;
    }

    if (c == '\n') {
        parser->line_buffer[parser->line_index] = '\0';
        process_line(parser, parser->line_buffer);
        parser->line_index = 0;

        if (parser->data->frame_complete) {
            return 1;  // Signal: complete, CRC-validated frame ready
        }
        return 0;
    }

    if (c == '\r') return 0;  // Skip CR from buffer (CRC already updated above)

    if (parser->line_index < sizeof(parser->line_buffer) - 1) {
        parser->line_buffer[parser->line_index++] = c;
    }

    return 0;
}

// Helper functions to convert fixed-point to float
static inline float fixed_to_kwh(int64_t val)  { return val / 1000000.0f; }
static inline float fixed_to_kw(int32_t val)   { return val / 1000000.0f; }
static inline float fixed_to_v(int16_t val)    { return val / 100.0f; }
static inline float fixed_to_a(int16_t val)    { return val / 1000.0f; }
// pf fields are now int32_t. Divide by 10000 to recover decimal (0.0-1.0).
// e.g. 9550 / 10000.0 = 0.955
static inline float fixed_to_pf(int32_t val)   { return val / 10000.0f; }

// Calculate power factor if not provided by meter.
// real_power parameter must be the phase-specific real power (kW),
// NOT the overall net total. Callers are updated accordingly.
static float calculate_pf(float voltage, float current, float real_power) {
    if (voltage < 1.0f || current < 0.001f) return 0.0f;

    float apparent_power = (voltage * current) / 1000.0f;  // kVA
    if (apparent_power < 0.001f) return 0.0f;

    float pf = real_power / apparent_power;
    if (pf >  1.0f) pf =  1.0f;
    if (pf < -1.0f) pf = -1.0f;

    return pf;
}

// Helper functions for easy access to parsed data
float dsmr_get_total_consumed(const dsmr_data_t *data) {
    return fixed_to_kwh(data->energy_delivered_low + data->energy_delivered_high);
}

float dsmr_get_total_produced(const dsmr_data_t *data) {
    return fixed_to_kwh(data->energy_produced_low + data->energy_produced_high);
}

float dsmr_get_net_energy(const dsmr_data_t *data) {
    int64_t net = ((int64_t)data->energy_delivered_low + (int64_t)data->energy_delivered_high) -
                  ((int64_t)data->energy_produced_low  + (int64_t)data->energy_produced_high);
    return fixed_to_kwh(net);
}

float dsmr_get_current_power(const dsmr_data_t *data) {
    return fixed_to_kw(data->current_power - data->current_return);
}

bool dsmr_is_consuming(const dsmr_data_t *data) {
    return data->current_power > data->current_return;
}

float dsmr_get_net_power_l1(const dsmr_data_t *data) {
    return fixed_to_kw(data->power_l1 - data->return_l1);
}

float dsmr_get_net_power_l2(const dsmr_data_t *data) {
    return fixed_to_kw(data->power_l2 - data->return_l2);
}

float dsmr_get_net_power_l3(const dsmr_data_t *data) {
    return fixed_to_kw(data->power_l3 - data->return_l3);
}

String dsmr_get_direction_l1(const dsmr_data_t *data) {
    if (data->power_l1 == 0 && data->return_l1 == 0) return "IDLE";
    return (data->power_l1 > data->return_l1) ? "-> CONSUMING" : "<- PRODUCING";
}

String dsmr_get_direction_l2(const dsmr_data_t *data) {
    if (data->power_l2 == 0 && data->return_l2 == 0) return "IDLE";
    return (data->power_l2 > data->return_l2) ? "-> CONSUMING" : "<- PRODUCING";
}

String dsmr_get_direction_l3(const dsmr_data_t *data) {
    if (data->power_l3 == 0 && data->return_l3 == 0) return "IDLE";
    return (data->power_l3 > data->return_l3) ? "-> CONSUMING" : "<- PRODUCING";
}

//=============================================================================
// END DSMR PARSER
//=============================================================================

// Hardware register manipulation for RX inversion
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define U0C0        ESP8266_REG(0x020)
#define UCRXI       19

#define KERNEL_VERSION    "6.2.0"
#define KERNEL_CODENAME   "GOOSE-TRI-MODE"
// Magic key incremented to 0x52 because config struct changed
// (pf_* widened from int16_t to int32_t). All devices will factory-reset
// once on first boot of this version, which is intentional.
// Incremented to 0x53 for System Mode support
#define MAGIC_KEY         0x57
#define TCP_PORT          2001
#define MAX_TCP_CLIENTS   10
#define MAX_FRAME_SIZE    1500
#define SHELLY_UDP_PORT   5683

#define MODE_SETUP  0
#define MODE_CLIENT 1
#define MODE_ACP    2

struct AppConfig {
    char     wifiSsid[33];
    char     wifiPass[64];
    char     apPass[64];
    char     wwwUser[33];
    char     wwwPass[33];
    bool     dhcpMode;
    char     staticIP[16];
    char     gateway[16];
    char     subnet[16];
    bool     useTcpDataSource;
    char     dataSourceHost[64];
    uint16_t dataSourcePort;
    uint16_t tcpServerPort;
    uint16_t marstekPort;
    uint8_t  systemMode;
    bool     emuEnabled;
};

AppConfig   config;
String      deviceId;
ESP8266WebServer webServer(80);
DNSServer   dnsServer;
WiFiServer  tcpServer(TCP_PORT);
WiFiClient  dataSourceClient;
WiFiClient  TCPClient[MAX_TCP_CLIENTS];
bool        apMode = false;
String      lastFrameBuffer = "Waiting for P1 Data...";
char        tempBuffer[MAX_FRAME_SIZE];
uint16_t    tempBufferIndex = 0;
bool        tempOverflowed  = false;
bool        frameEndDetected = false;
int         activeClients   = 0;
char        broadcastBuf[512];
uint16_t    broadcastLen    = 0;
unsigned long lastBroadcastByteTime = 0;

/**
 * Flushes the accumulated broadcast buffer to all connected TCP clients.
 * This significantly reduces the overhead compared to byte-by-byte sending.
 */
void flushBroadcast() {
    if (broadcastLen == 0) return;
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (TCPClient[i].connected()) {
            // Only drop the client if the buffer is completely backed up (0 bytes).
            // If there is any space, write() will block slightly but won't hang the system
            // unless the connection is truly dead.
            if (TCPClient[i].availableForWrite() > 0) {
                TCPClient[i].write((const uint8_t*)broadcastBuf, broadcastLen);
            } else {
                Serial.printf("[TCP] Client %d totally congested, dropping.\n", i);
                TCPClient[i].stop();
            }
        }
    }
    broadcastLen = 0;
}

// DSMR Parser instances
dsmr_data_t  dsmr_data;
dsmr_data_t  dsmr_last_good;
dsmr_parser_t dsmr_parser;

// Watchdog and monitoring variables
unsigned long lastDataReceived = 0;
unsigned long lastClientCheck  = 0;
unsigned long lastWiFiCheck    = 0;
unsigned long bootTime         = 0;
const unsigned long WATCHDOG_TIMEOUT   = 600000;
const unsigned long WIFI_TIMEOUT       = 300000;
const unsigned long BOOT_GRACE_PERIOD  = 300000;

/* --- UI UTILITIES --- */
String customUrlEncode(String str) {
    String out = "";
    for (int i = 0; i < (int)str.length(); i++) {
        char c = str.charAt(i);
        if (isalnum(c)) out += c;
        else { out += '%'; char b[3]; sprintf(b, "%02X", (unsigned char)c); out += b; }
    }
    return out;
}

String htmlEscape(const String& str) {
    String out;
    out.reserve(str.length() + 16);
    for (unsigned int i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        if      (c == '&')  out += "&amp;";
        else if (c == '<')  out += "&lt;";
        else if (c == '>')  out += "&gt;";
        else if (c == '"')  out += "&quot;";
        else if (c == '\'') out += "&#39;";
        else                out += c;
    }
    return out;
}

const char* dashStyle =
"<style>"
"body{background:#0c0c0c;color:#eee;font-family:'Segoe UI',sans-serif;text-align:center;padding:20px;}"
".container{background:#181818;border:2px solid #ffcc00;display:inline-block;padding:35px;border-radius:15px;width:100%;max-width:520px;box-shadow:0 15px 35px rgba(0,0,0,0.7);}"
"h1{color:#ffcc00;border-bottom:2px solid #ffcc00;padding-bottom:12px;margin-top:0;text-transform:uppercase;letter-spacing:2px;}"
".stat{background:#222;padding:15px;margin:12px 0;border-radius:8px;border-left:6px solid #ffcc00;text-align:left;color:#ffcc00;font-family:'Courier New',monospace;font-size:0.9em;}"
".diag{border-left-color:#00ffcc;color:#00ffcc;}"
".pf-good{border-left-color:#00ff00;color:#00ff00;}"
".pf-fair{border-left-color:#ffaa00;color:#ffaa00;}"
".pf-poor{border-left-color:#ff3300;color:#ff3300;}"
".btn{color:#121212;background:#ffcc00;padding:14px;border-radius:8px;font-weight:bold;text-decoration:none;display:block;margin-top:12px;border:none;cursor:pointer;text-align:center;font-size:1em;width:100%;box-sizing:border-box;}"
".btn-raw{background:#006400;color:#fff;}"
".btn-red{background:#cc3300;color:#fff;}"
"input,select{width:100%;padding:14px;margin:10px 0;background:#000;color:#fff;border:1px solid #333;border-radius:6px;box-sizing:border-box;}"
".footer{color:#444;font-size:0.95em;margin-top:30px;font-family:monospace;border-top:1px solid #222;padding-top:15px;line-height:1.6;}"
".net-item{padding:15px;border-bottom:1px solid #333;text-align:left;cursor:pointer;}"
".net-item:hover{background:#222;}"
"</style>";

const char* ipScript =
"<script>"
"function toggleIP(){var d=document.getElementById('dhcp').value=='0';"
"document.getElementById('staticFields').style.display=d?'block':'none';}"
"</script>";

const char* modeScript =
"<script>"
"function toggleMode(){"
" var m=document.getElementById('sys_mode').value;"
" var isClient=(m=='1');"
" document.getElementById('clientFields').style.display=isClient?'block':'none';"
" document.getElementById('apFields').style.display=(m=='2')?'block':'none';"
"}"
"</script>";

String modeSelectHtml() {
    String s = "<strong>OPERATING MODE:</strong><select name='sys_mode' id='sys_mode' onchange='toggleMode()'>";
    s += "<option value='0' " + String(config.systemMode == MODE_SETUP ? "selected" : "") + ">SETUP (Config Only)</option>";
    s += "<option value='1' " + String(config.systemMode == MODE_CLIENT ? "selected" : "") + ">CLIENT (Connect to WiFi)</option>";
    s += "<option value='2' " + String(config.systemMode == MODE_ACP ? "selected" : "") + ">ACP (Standalone AP)</option></select>";
    return s;
}

String getFooter() {
    return "<div class='footer'>KERNEL: " + String(KERNEL_VERSION) + " [" + String(KERNEL_CODENAME) + "]<br>"
           "BUILD: " + String(__DATE__) + " " + String(__TIME__) + "<br>"
           "AUTHOR: NOEL VELLEMANS</div>";
}

String formatUptime() {
    unsigned long s = millis() / 1000;
    return String(s/86400) + "d " + String((s%86400)/3600) + "h " +
           String((s%3600)/60) + "m " + String(s%60) + "s";
}

String ipFieldsHtml() {
    String s = "<select name='dhcp' id='dhcp' onchange='toggleIP()'>";
    s += "<option value='1' " + String(config.dhcpMode ? "selected" : "") + ">DHCP (Automatic)</option>";
    s += "<option value='0' " + String(!config.dhcpMode ? "selected" : "") + ">Static (Fixed IP)</option></select>";
    s += "<div id='staticFields' style='display:" + String(config.dhcpMode ? "none" : "block") + "'>";
    s += "<input name='ip' placeholder='IP Address' value='" + String(config.staticIP) + "'>";
    s += "<input name='gw' placeholder='Gateway' value='" + String(config.gateway) + "'>";
    s += "<input name='sn' placeholder='Subnet Mask' value='" + String(config.subnet) + "'></div>";
    return s;
}

String getPFClass(float pf) {
    float a = fabsf(pf);
    if (a >= 0.95f) return "pf-good";
    if (a >= 0.85f) return "pf-fair";
    return "pf-poor";
}

String formatPF(float pf) {
    float  a = fabsf(pf);
    String q;
    if      (a >= 0.95f) q = "EXCELLENT";
    else if (a >= 0.90f) q = "GOOD";
    else if (a >= 0.85f) q = "FAIR";
    else if (a >= 0.70f) q = "POOR";
    else                 q = "CRITICAL";
    return String(a, 3) + " (" + q + ", " + (pf >= 0 ? "LAG" : "LEAD") + ")";
}

String getPFColor(float pf) {
    float a = fabsf(pf);
    if (a >= 0.95f) return "#00ff00";
    if (a >= 0.85f) return "#ffaa00";
    return "#ff3300";
}

// Include the emulator here so it has access to the types and variables defined above
#include "ShellyEmulator.h"

//=============================================================================
// LIVE PAGE HANDLER - Chunked transfer to avoid large heap String
//=============================================================================

// Enable runtime heap logging to Serial for quick before/after measurements.
#define ENABLE_HEAP_LOG 1

static inline void logHeap(const char *tag) {
#if ENABLE_HEAP_LOG
    Serial.printf("[HEAP] %s: %u bytes free\n", tag, (unsigned int)ESP.getFreeHeap());
#endif
}
void handleLive() {
    if (!webServer.authenticate(config.wwwUser, config.wwwPass))
        return webServer.requestAuthentication();

    // Use chunked transfer so we never hold the whole page in RAM.
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma",  "no-cache");
    webServer.sendHeader("Expires", "0");
    webServer.send(200, "text/html", "");

    // Shared buffer for HTML generation (Max usage ~320 bytes in Power Analysis)
    char buf[400];

    // Reset client watchdog so viewing the UI prevents "No Clients" reboot
    lastClientCheck = millis();

    // Heap snapshot at handler entry
    logHeap("handleLive:start");

    // ---- HEAD ----
    webServer.sendContent(F("<html><head><title>Live P1</title>"
                            "<meta charset='UTF-8'>"
                            "<meta name='viewport' content='width=device-width'>"));
    webServer.sendContent(dashStyle);
    webServer.sendContent(F("</head><body><div class='container'><h1>LIVE DATA</h1>"));

    // ---- TIMESTAMP / METER ID ----
    {
        snprintf(buf, sizeof(buf),
                 "<div class='stat diag'>TIMESTAMP: %s<br>METER ID: %s</div>",
                 dsmr_last_good.timestamp, dsmr_last_good.meter_id);
        webServer.sendContent(buf);
    }

    float net = dsmr_get_current_power(&dsmr_last_good);
    // ---- CURRENT NET POWER ----
    {
        const char *col   = (net >= 0) ? "#ffcc00" : "#00ffcc";
        const char *label = (net >= 0) ? "CONSUMPTION: " : "INJECTION: ";
        snprintf(buf, sizeof(buf),
                 "<div class='stat' style='border-left-color:%s;color:%s;'>"
                 "CURRENT %s%.3f kW</div>",
                 col, col, label, fabsf(net));
        webServer.sendContent(buf);
    }

    // ---- ENERGY TOTALS ----
    {
        snprintf(buf, sizeof(buf),
                 "<div class='stat'>TOTAL CONSUMED: %.3f kWh<br>"
                 "TOTAL PRODUCED: %.3f kWh</div>",
                 dsmr_get_total_consumed(&dsmr_last_good),
                 dsmr_get_total_produced(&dsmr_last_good));
        webServer.sendContent(buf);
    }

    // Pre-calculate all phase values once
    float p1  = dsmr_get_net_power_l1(&dsmr_last_good);
    float p2  = dsmr_get_net_power_l2(&dsmr_last_good);
    float p3  = dsmr_get_net_power_l3(&dsmr_last_good);
    float v1  = fixed_to_v(dsmr_last_good.voltage_l1);
    float v2  = fixed_to_v(dsmr_last_good.voltage_l2);
    float v3  = fixed_to_v(dsmr_last_good.voltage_l3);
    float c1  = fixed_to_a(dsmr_last_good.current_l1);
    float c2  = fixed_to_a(dsmr_last_good.current_l2);
    float c3  = fixed_to_a(dsmr_last_good.current_l3);
    float va1 = (v1 * c1) / 1000.0f;
    float va2 = (v2 * c2) / 1000.0f;
    float va3 = (v3 * c3) / 1000.0f;
    float total_va = va1 + va2 + va3;

    // Power factor: from meter if available, calculated otherwise
    float pf1, pf2, pf3;
    if (dsmr_last_good.has_pf_data) {
        pf1 = fixed_to_pf(dsmr_last_good.pf_l1);
        pf2 = fixed_to_pf(dsmr_last_good.pf_l2);
        pf3 = fixed_to_pf(dsmr_last_good.pf_l3);
    } else {
        if (!dsmr_last_good.is_3phase) {
            // Use L1 phase power, not the overall net total.
            pf1 = calculate_pf(v1, c1, fabsf(p1));
            pf2 = 0.0f; pf3 = 0.0f;
        } else {
            pf1 = calculate_pf(v1, c1, fabsf(p1));
            pf2 = calculate_pf(v2, c2, fabsf(p2));
            pf3 = calculate_pf(v3, c3, fabsf(p3));
        }
    }

    // ---- POWER ANALYSIS SUMMARY ----
    {
        float overall_pf = (total_va > 0.001f) ? (fabsf(net) / total_va) : 0.0f;
        snprintf(buf, sizeof(buf),
                 "<div class='stat' style='border-left-color:#ff6600;color:#ff6600;'>"
                 "<strong>POWER ANALYSIS:</strong><br>"
                 "Total Real Power (W): %.3f kW<br>"
                 "Total Apparent Power (VA): %.3f kVA<br>"
                 "Overall Power Factor: %.3f",
                 net, total_va, overall_pf);
        webServer.sendContent(buf);

        if (overall_pf < 0.90f) {
            snprintf(buf, sizeof(buf),
                     "<br><span style='color:#ff3300;'>! Wasted Capacity: %.3f kVA (reactive)</span>",
                     total_va - fabsf(net));
            webServer.sendContent(buf);
        }
        webServer.sendContent(F("</div>"));
    }

    // ---- PHASE METRICS ----
    webServer.sendContent(F("<div class='stat diag'><strong>PHASE METRICS:</strong><br>"));

    if (dsmr_last_good.is_3phase) {
        // L1
        {
            snprintf(buf, sizeof(buf),
                     "<strong>L1:</strong> %.1fV | %s%.3fA | %.3fkVA -&gt; %s%.3fkW",
                     v1,
                     (p1 >= 0) ? "+" : "-", c1,
                     va1,
                     (p1 >= 0) ? "+" : "", p1);
            webServer.sendContent(buf);
            if (va1 > 0.001f) {
                snprintf(buf, sizeof(buf), " (PF=%.3f)", pf1);
                webServer.sendContent(buf);
            }
            webServer.sendContent(F("<br>"));
        }
        // L2
        {
            snprintf(buf, sizeof(buf),
                     "<strong>L2:</strong> %.1fV | %s%.3fA | %.3fkVA -&gt; %s%.3fkW",
                     v2,
                     (p2 >= 0) ? "+" : "-", c2,
                     va2,
                     (p2 >= 0) ? "+" : "", p2);
            webServer.sendContent(buf);
            if (va2 > 0.001f) {
                snprintf(buf, sizeof(buf), " (PF=%.3f)", pf2);
                webServer.sendContent(buf);
            }
            webServer.sendContent(F("<br>"));
        }
        // L3
        {
            snprintf(buf, sizeof(buf),
                     "<strong>L3:</strong> %.1fV | %s%.3fA | %.3fkVA -&gt; %s%.3fkW",
                     v3,
                     (p3 >= 0) ? "+" : "-", c3,
                     va3,
                     (p3 >= 0) ? "+" : "", p3);
            webServer.sendContent(buf);
            if (va3 > 0.001f) {
                snprintf(buf, sizeof(buf), " (PF=%.3f)", pf3);
                webServer.sendContent(buf);
            }
            webServer.sendContent(F("<br><br>"));
        }

        // Verification
        {
            float sum_power = p1 + p2 + p3;
            float diff      = fabsf(sum_power - net);
            snprintf(buf, sizeof(buf),
                     "<strong>VERIFICATION:</strong><br>"
                     "Sum of phases: %.3f kW<br>"
                     "Meter total: %.3f kW ",
                     sum_power, net);
            webServer.sendContent(buf);
            if (diff < 0.010f) {
                webServer.sendContent(F("<span style='color:#00ff00;'>✓ Match!</span>"));
            } else {
                snprintf(buf, sizeof(buf),
                         "<span style='color:#ffaa00;'>! Delta=%.3fkW</span>", diff);
                webServer.sendContent(buf);
            }
        }
        webServer.sendContent(F("</div>"));

        // Phase PF block
        {
            float min_pf = fabsf(pf1);
            if (fabsf(pf2) < min_pf) min_pf = fabsf(pf2);
            if (fabsf(pf3) < min_pf) min_pf = fabsf(pf3);
            String pfClass = (min_pf < 0.85f) ? "pf-poor" :
                             (min_pf < 0.95f) ? "pf-fair" : "pf-good";

            webServer.sendContent("<div class='stat " + pfClass + "'><strong>PHASE POWER FACTOR:</strong><br>");

            snprintf(buf, sizeof(buf), "<span style='color:%s'>L1: ", getPFColor(pf1).c_str());
            webServer.sendContent(buf);
            webServer.sendContent(formatPF(pf1));
            snprintf(buf, sizeof(buf), "</span><br><span style='color:%s'>L2: ", getPFColor(pf2).c_str());
            webServer.sendContent(buf);
            webServer.sendContent(formatPF(pf2));
            snprintf(buf, sizeof(buf), "</span><br><span style='color:%s'>L3: ", getPFColor(pf3).c_str());
            webServer.sendContent(buf);
            webServer.sendContent(formatPF(pf3));
            webServer.sendContent(F("</span></div>"));
        }
    } else {
        // Single-phase
        {
            snprintf(buf, sizeof(buf),
                     "MAIN: %.1fV | %.3fA | %.3fkVA -&gt; %.3fkW",
                     v1, c1, va1, net);
            webServer.sendContent(buf);
            if (va1 > 0.001f) {
                snprintf(buf, sizeof(buf), " (PF=%.3f)", pf1);
                webServer.sendContent(buf);
            }
            webServer.sendContent(F("</div>"));
        }
        webServer.sendContent("<div class='stat " + getPFClass(pf1) + "'><strong>POWER FACTOR:</strong> ");
        webServer.sendContent(formatPF(pf1));
        webServer.sendContent(F("</div>"));
    }

#if(0)    // ---- EDUCATIONAL GUIDE ----
    webServer.sendContent(F(
        "<div class='stat' style='font-size:0.75em;color:#999;border-left-color:#444;'>"
        "<strong>UNDERSTANDING POWER &amp; CURRENT:</strong><br>"
        "&bull; <strong>+ (Positive):</strong> Consuming from grid<br>"
        "&bull; <strong>- (Negative):</strong> Injecting to grid<br>"
        "&bull; <strong>Current sign:</strong> Follows power direction<br>"
        "&bull; <strong>Real Power (kW):</strong> Energy you actually use (billed)<br>"
        "&bull; <strong>Apparent Power (kVA):</strong> V x I = Grid capacity used<br>"
        "&bull; <strong>Power Factor (PF):</strong> Real Power / Apparent Power<br>"
        "&bull; <strong>Good PF (&gt;0.95):</strong> Efficient, W ~= VA<br>"
        "&bull; <strong>Poor PF (&lt;0.70):</strong> Wastes grid capacity<br>"
        "&bull; <strong>Sum Rule:</strong> Real power (kW) adds linearly across phases<br>"
    ));
    webServer.sendContent(dsmr_last_good.has_pf_data
        ? "PF DATA: From Meter" : "PF DATA: Calculated from VxI/P");
    webServer.sendContent(F("</div>"));
#endif

    // ---- BUTTONS + FOOTER ----
    webServer.sendContent(F(
        "<a href='/live-data' class='btn' style='background:#00aa00;color:#fff;margin-top:20px;'>REFRESH NOW</a>"
        "<a href='/' class='btn' style='background:#333;color:#fff;'>BACK TO DASHBOARD</a>"
    ));
    webServer.sendContent(getFooter());
    webServer.sendContent(F("</div></body></html>"));
    // Heap snapshot before finishing handler
    logHeap("handleLive:end");
    webServer.sendContent("");  // Terminate chunked encoding
}

void handleRoot() {
    bool isSetupUI = (config.systemMode == MODE_SETUP) || (config.systemMode == MODE_CLIENT && apMode);

    // Reset client watchdog so viewing the UI prevents "No Clients" reboot
    lastClientCheck = millis();

    // Heap snapshot at entry
    logHeap("handleRoot:start");

    if (apMode &&
        !webServer.hostHeader().equalsIgnoreCase(WiFi.softAPIP().toString()) &&
        !webServer.hostHeader().equalsIgnoreCase(deviceId + ".local")) {
        webServer.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
        webServer.send(302, "text/plain", "");
        return;
    }

    if (!isSetupUI && !webServer.authenticate(config.wwwUser, config.wwwPass))
        return webServer.requestAuthentication();

    String pageTitle = deviceId + (isSetupUI ? " - Setup" : " - Dashboard");
    String h = "<html><head><title>" + pageTitle + "</title>"
               "<meta charset='UTF-8'><meta name='viewport' content='width=device-width'>"
               + String(dashStyle) + ipScript + modeScript + "</head><body><div class='container'>";
    h += "<h1>" + pageTitle + "</h1>";

    if (isSetupUI) {
        h += "<div class='stat diag'>SETUP/RECOVERY MODE<br>IP: " + WiFi.softAPIP().toString() + "</div>";
        h += "<a href='/scan' class='btn' style='margin-bottom:12px;'>SCAN WIFI NETWORKS</a>";
        h += "<form method='POST' action='/saveConfig'>"
             + modeSelectHtml() +
             "<div id='clientFields' style='display:" + String(config.systemMode == MODE_CLIENT ? "block" : "none") + "'>"
             "<input name='ssid' id='ssid' placeholder='WiFi SSID'>"
             "<input name='pass' type='password' placeholder='WiFi Password'>"
             "<hr style='border:1px solid #333;margin:20px 0;'><strong>CLIENT NETWORK SETTINGS:</strong>" + ipFieldsHtml() + 
             "</div>";
        h += "<div style='margin-top:10px;'><strong>TCP SERVER PORT:</strong><input name='srv_port' type='number' placeholder='TCP Server Port (Default: 2001)' value='" + String(config.tcpServerPort) + "'></div>";
        h += "<button class='btn' style='margin-top:10px;'>SAVE & REBOOT</button></form>";
        h += "<a href='/update' class='btn' style='background:#004d40;color:#fff;'>FLASH FIRMWARE (OTA)</a>";
        h += "<form method='POST' action='/factReset' onsubmit=\"return confirm('ERASE ALL?')\"><button class='btn btn-red'>FACTORY RESET</button></form>";
    } else {
        String ip = (config.systemMode == MODE_ACP) ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
        h += "<div class='stat'>IP: " + ip + ":" + String(config.tcpServerPort) + "<br>STREAMS: " + String(activeClients) + " of " + String(MAX_TCP_CLIENTS) + " (MAX)</div>";
        h += "<div class='stat diag'>UPTIME: " + formatUptime() + "<br>HEAP: " + String(ESP.getFreeHeap()) + " Bytes<br>SIGNAL: " + String(WiFi.RSSI()) + " dBm<br>LAST REBOOT: " + ESP.getResetReason() + "</div>";
        h += "<a class='btn' href='/live-data' style='background:#ffcc00;color:#000;'>VIEW LIVE DATA</a>";
        h += "<a class='btn btn-raw' href='/raw'>VIEW RAW P1 DATA</a>";
        h += "<a class='btn' href='/settings' style='margin-top:12px;'>SYSTEM SETTINGS</a>";
    }
    h += getFooter() + "</div>"
         "<script>var p=new URLSearchParams(window.location.search);"
         "if(p.has('s'))document.getElementById('ssid').value=decodeURIComponent(p.get('s'));"
         "</script></body></html>";

    // Heap before sending the generated page
    logHeap("handleRoot:before_send");
    webServer.send(200, "text/html", h);
    // Heap after sending the generated page
    logHeap("handleRoot:end");
}

void setup() {
    ESP.wdtEnable(8000);
    ESP.wdtFeed();

    Serial.begin(115200);
    delay(10);
    ESP.wdtFeed();

    Serial.println("\r\n\n==============================================");
    Serial.println("  SMR BRIDGE KERNEL " + String(KERNEL_VERSION));
    Serial.print("  CODENAME : "); Serial.println(KERNEL_CODENAME);
    Serial.print("  BUILD    : "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
    Serial.println("  WATCHDOG : ENABLED");

    dsmr_parser_init(&dsmr_parser, &dsmr_data);

    EEPROM.begin(512);
    ESP.wdtFeed();

    if (EEPROM.read(0) != MAGIC_KEY) {
        // Config struct changed (pf fields widened). Fresh defaults on new magic key.
        memset(&config, 0, sizeof(AppConfig));
        config.dhcpMode = true;
        strcpy(config.wwwUser, "admin"); strcpy(config.wwwPass, "admin");
        strcpy(config.staticIP, "192.168.4.1");
        strcpy(config.gateway,  "192.168.4.1");
        strcpy(config.subnet,   "255.255.255.0");
        config.useTcpDataSource = false;
        config.dataSourcePort   = 2001;
        strcpy(config.dataSourceHost, "");
        strcpy(config.apPass, "");
        config.tcpServerPort    = 2001;
        config.marstekPort      = 2220; // 1010 for legacy Marstek, 2220 for modern firmware
        config.emuEnabled       = true;
        config.systemMode       = MODE_SETUP;
        EEPROM.write(0, MAGIC_KEY);
        EEPROM.put(1, config);
        EEPROM.commit();
    } else {
        EEPROM.get(1, config);
    }
    
    // Set a short timeout for TCP operations to prevent 5+ second hangs 
    // during network hiccups.
    dataSourceClient.setTimeout(1000);

    if (config.tcpServerPort == 0) config.tcpServerPort = 2001;

    if (!config.useTcpDataSource) {
        U0C0 |= BIT(UCRXI);
        Serial.println("  RX INVERT: ENABLED (P1 Port Mode)");
    } else {
        Serial.println("  RX INVERT: DISABLED (TCP Source Mode)");
    }
    Serial.println("  PARSER   : DSMR V2 LINE-BUFFERED + CRC16 + PF");
    Serial.println("==============================================");

    delay(10);

    uint8_t mac[6]; WiFi.macAddress(mac); char devName[32];
    const char* prefix = "SMR-BRIDGE";
    if (config.systemMode == MODE_SETUP)       prefix = "SMR-SETUP";
    else if (config.systemMode == MODE_CLIENT) prefix = "SMR-CLT-BRIDGE";
    else if (config.systemMode == MODE_ACP)    prefix = "SMR-ACP-BRIDGE";

    sprintf(devName, "%s-%02X%02X%02X", prefix, mac[3], mac[4], mac[5]);
    deviceId = String(devName);

    WiFi.hostname(deviceId);
    ESP.wdtFeed();

    if (config.systemMode == MODE_CLIENT) {
        WiFi.mode(WIFI_STA);
        if (!config.dhcpMode) {
            IPAddress _ip, _gw, _sn;
            _ip.fromString(config.staticIP);
            _gw.fromString(config.gateway);
            _sn.fromString(config.subnet);
            WiFi.config(_ip, _gw, _sn);
            Serial.print("[NET] Static IP: "); Serial.println(config.staticIP);
        }
        WiFi.begin(config.wifiSsid, config.wifiPass);
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 60) {
            delay(1000); Serial.print("."); retry++; ESP.wdtFeed();
        }
    } else {
        // Setup or ACP mode -> Force AP
        apMode = true;
    }

    if (config.systemMode == MODE_CLIENT && WiFi.status() == WL_CONNECTED) {
         Serial.println("\n[NET] ONLINE: " + WiFi.localIP().toString());
         MDNS.begin("smr");
         // Init operational timers
         bootTime          = millis();
         lastDataReceived  = bootTime;
         lastClientCheck   = bootTime;
         lastWiFiCheck     = bootTime;
    } else {
        // Fallback to AP (Client Failed, Setup, or ACP)
        apMode = true;
        WiFi.mode(config.systemMode == MODE_CLIENT ? WIFI_AP_STA : WIFI_AP);
        
        if (config.systemMode == MODE_CLIENT) {
            // Use a distinct name so user knows it failed to connect
            char fallbackName[32];
            sprintf(fallbackName, "SMR-FALLBACK-%02X%02X%02X", mac[3], mac[4], mac[5]);
            if (strlen(config.apPass) >= 8) WiFi.softAP(fallbackName, config.apPass);
            else WiFi.softAP(fallbackName);
        } else {
            // Setup or ACP modes use their native names
            if (strlen(config.apPass) >= 8) WiFi.softAP(deviceId.c_str(), config.apPass);
            else WiFi.softAP(deviceId.c_str());
        }
        
        dnsServer.start(53, "*", WiFi.softAPIP());
        Serial.println("\n[NET] AP Mode: " + deviceId);
        Serial.println("[NET] AP IP:   " + WiFi.softAPIP().toString());
        
        if (config.systemMode == MODE_ACP) {
            // ACP is an operational mode
            bootTime          = millis();
            lastDataReceived  = bootTime;
            lastClientCheck   = bootTime;
            // WiFi check ignored in ACP
        }
    }

    ESP.wdtFeed();

    /* --- WEB ROUTES --- */
    webServer.on("/",         handleRoot);
    webServer.on("/live-data",handleLive);
    webServer.on("/ncsi.txt", [](){ webServer.send(200, "text/plain", "Microsoft NCSI"); });
    webServer.on("/generate_204", handleRoot);

    webServer.on("/scan", [](){
        // Heap snapshot for scan handler
        logHeap("scan:start");

        String h = "<html><head><title>WiFi Scan</title>" + String(dashStyle) + "</head>"
                   "<body><div class='container'><h1>WIFI SCAN</h1>"
                   "<div class='stat diag' style='text-align:center;'>Scanning, please wait...</div>"
                   "<script>setTimeout(function(){ location='/scanresults'; }, 2000);</script>";
        String backLink = apMode ? "/" : "/config/wifi";
        h += "<a href='" + backLink + "' class='btn' style='background:#333;color:#fff;margin-top:20px;'>CANCEL</a></div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/scanresults", [](){
        // Heap snapshot for scan results handler
        logHeap("scanresults:start");

        int n = WiFi.scanNetworks();
        String h = "<html><head><title>WiFi Scan Results</title>" + String(dashStyle) + "</head><body><div class='container'><h1>WIFI SCAN RESULTS</h1>";
        if (n == 0) {
            h += "<div class='stat' style='color:#ff3300;text-align:center;'>NO NETWORKS FOUND</div>"
                 "<a href='/scan' class='btn' style='margin-top:20px;'>SCAN AGAIN</a>";
        } else if (n > 0) {
            h += "<div class='stat diag' style='text-align:center;'>Found " + String(n) + " network(s)</div>";
            for (int i = 0; i < n; i++) {
                String ca = apMode
                    ? "location='/?s="           + customUrlEncode(WiFi.SSID(i)) + "'"
                    : "location='/config/wifi?s=" + customUrlEncode(WiFi.SSID(i)) + "'";
                h += "<div class='net-item' onclick=\"" + ca + "\"><strong>" + htmlEscape(WiFi.SSID(i)) + "</strong><br><small>Signal: " + String(WiFi.RSSI(i)) + "dBm</small></div>";
            }
            h += "<a href='/scan' class='btn' style='margin-top:20px;'>SCAN AGAIN</a>";
        } else {
            h += "<div class='stat' style='color:#ff9900;text-align:center;'>SCAN FAILED</div>"
                 "<a href='/scan' class='btn' style='margin-top:20px;'>SCAN AGAIN</a>";
        }
        String backLink = apMode ? "/" : "/config/wifi";
        h += "<a href='" + backLink + "' class='btn' style='background:#333;color:#fff;margin-top:12px;'>BACK</a></div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/raw", [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        // Heap snapshot for raw page
        logHeap("raw:start");

        String h = "<html><head><title>RAW P1 Data</title><meta charset='UTF-8'>"
                   "<meta http-equiv='refresh' content='5'><meta name='viewport' content='width=device-width'>"
                   + String(dashStyle) + "</head><body><div class='container'><h1>RAW P1 DATA</h1>"
                   "<div class='stat' style='font-family:monospace;white-space:pre-wrap;text-align:left;font-size:0.85em;'>"
                   + lastFrameBuffer + "</div>"
                   "<a href='/raw' class='btn' style='background:#00aa00;color:#fff;margin-top:20px;'>REFRESH NOW</a>"
                   "<a href='/' class='btn' style='background:#333;color:#fff;margin-top:12px;'>BACK TO DASHBOARD</a>"
                   + getFooter() + "</div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/settings", [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        // Heap snapshot for settings page
        logHeap("settings:start");

        String h = "<html><head><title>Settings</title>" + String(dashStyle) + "</head><body><div class='container'><h1>SETTINGS</h1>"
                   "<a href='/config/wifi' class='btn'>WIFI & NETWORK</a>"
                   "<a href='/config/emu' class='btn' style='background:#6a1b9a;color:#fff;'>EMULATION MODES</a>"
                   "<a href='/config/auth' class='btn'>ADMIN SECURITY</a>"
                   "<a href='/config/source' class='btn' style='background:#2E8B57;color:#fff;'>DATA SOURCE</a>"
                   "<a href='/update' class='btn' style='background:#004d40;color:#fff;'>FLASH FIRMWARE (OTA)</a>"
                   "<form method='POST' action='/factReset' onsubmit=\"return confirm('Reset Everything?')\"><button class='btn btn-red'>FACTORY RESET</button></form>"
                   "<form method='POST' action='/reboot' onsubmit=\"return confirm('Reboot device?')\"><button class='btn' style='margin-top:8px;background:#ff5722;color:#fff;'>REBOOT</button></form>"
                   "<a href='/' class='btn' style='background:#333;color:#fff;'>BACK</a>" + getFooter() + "</div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/config/wifi", [](){
        bool isSetupUI = (config.systemMode == MODE_SETUP) || (config.systemMode == MODE_CLIENT && apMode);
        if (!isSetupUI && !webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        // Heap snapshot for config wifi page
        logHeap("config_wifi:start");

        String h = "<html><head><title>WiFi & Network</title>" + String(dashStyle) + ipScript + modeScript + "</head><body><div class='container'><h1>WIFI & NETWORK</h1>"
                   "<a href='/scan' class='btn' style='margin-bottom:12px;'>SCAN WIFI NETWORKS</a>"
                   "<form method='POST' action='/saveConfig'>"
                   + modeSelectHtml() +
                   "<div id='apFields' style='display:" + String(config.systemMode == MODE_ACP ? "block" : "none") + "'>"
                   "<hr style='border:1px solid #333;margin:12px 0;'><strong>ACCESS POINT (AP) SETTINGS:</strong>"
                   "<input name='ap_pass' type='password' placeholder='AP Password (optional, min 8)' value='" + htmlEscape(config.apPass) + "'>"
                   "</div>"
                   "<div id='clientFields' style='display:" + String(config.systemMode == MODE_CLIENT ? "block" : "none") + "'>"
                   "<input name='ssid' id='ssid' placeholder='WiFi SSID' value='" + htmlEscape(config.wifiSsid) + "'>"
                   "<input name='pass' type='password' placeholder='WiFi Password'>";
        h += "<hr style='border:1px solid #333;margin:20px 0;'><strong>CLIENT NETWORK SETTINGS:</strong>" + ipFieldsHtml();
        h += "</div>";
        h += "<div style='margin-top:10px;'><strong>TCP SERVER PORT:</strong>";
        h += "<input name='srv_port' type='number' placeholder='Port (Default: 2001)' value='" + String(config.tcpServerPort) + "'></div>";
        h += "<button class='btn'>SAVE SETTINGS</button></form>";
        h += "<a href='/settings' class='btn' style='background:#333;color:#fff;'>BACK</a>" + getFooter() + "</div>";
        h += "<script>var p=new URLSearchParams(window.location.search);";
        h += "if(p.has('s'))document.getElementById('ssid').value=decodeURIComponent(p.get('s'));</script>";
        h += "</body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/config/auth", [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        // Heap snapshot for config auth page
        logHeap("config_auth:start");

        String h = "<html><head><title>Admin Security</title>" + String(dashStyle) + "</head><body><div class='container'><h1>ADMIN SECURITY</h1>"
                   "<form method='POST' action='/savePass'>"
                   "<input name='user' placeholder='Username' value='" + htmlEscape(config.wwwUser) + "'>"
                   "<input name='pass' type='password' placeholder='New Password'>"
                   "<button class='btn'>UPDATE CREDENTIALS</button></form>"
                   "<a href='/settings' class='btn' style='background:#333;color:#fff;'>BACK</a>" + getFooter() + "</div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/config/source", [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        // Heap snapshot for config source page
        logHeap("config_source:start");

        String h = "<html><head><title>Data Source</title>" + String(dashStyle);
        h += "<script>function toggleSource(){var d=document.getElementById('source_mode').value=='1';"
             "document.getElementById('tcpSourceFields').style.display=d?'block':'none';}</script>";
        h += "</head><body onload='toggleSource()'><div class='container'><h1>DATA SOURCE</h1>"
             "<div class='stat diag'>Select where to get the P1 data from.</div>"
             "<form method='POST' action='/saveSource'>"
             "<strong>DATA INPUT METHOD:</strong>"
             "<select name='source_mode' id='source_mode' onchange='toggleSource()'>"
             "<option value='0' " + String(!config.useTcpDataSource ? "selected" : "") + ">Local Serial (P1 Port)</option>"
             "<option value='1' " + String(config.useTcpDataSource  ? "selected" : "") + ">Remote TCP Stream</option></select>"
             "<div id='tcpSourceFields' style='display:" + String(config.useTcpDataSource ? "block" : "none") + "'>"
             "<input name='host' placeholder='Remote Host or IP' value='" + htmlEscape(config.dataSourceHost) + "'>"
             "<input name='port' type='number' placeholder='Port' value='" + String(config.dataSourcePort) + "'></div>"
             "<button class='btn'>SAVE & REBOOT</button></form>"
             "<a href='/settings' class='btn' style='background:#333;color:#fff;'>BACK</a>" + getFooter() + "</div></body></html>";
        webServer.send(200, "text/html", h);
    });

    // /saveConfig has no auth check intentionally - it is the setup
    // endpoint used before credentials are configured (AP mode). If called in
    // client mode without auth the worst case is a settings change + reboot,
    // which requires LAN access and knowledge of the endpoint.
    webServer.on("/saveConfig", HTTP_POST, [](){
        config.systemMode = (uint8_t)webServer.arg("sys_mode").toInt();
        strncpy(config.wifiSsid, webServer.arg("ssid").c_str(), 32);
        strncpy(config.wifiPass, webServer.arg("pass").c_str(), 63);
        strncpy(config.apPass, webServer.arg("ap_pass").c_str(), 63);
        config.dhcpMode = (webServer.arg("dhcp") == "1");
        strncpy(config.staticIP, webServer.arg("ip").c_str(), 15);
        strncpy(config.gateway,  webServer.arg("gw").c_str(), 15);
        strncpy(config.subnet,   webServer.arg("sn").c_str(), 15);
        config.tcpServerPort = (uint16_t)webServer.arg("srv_port").toInt();
        if (config.tcpServerPort == 0) config.tcpServerPort = 2001;
        EEPROM.put(1, config); EEPROM.commit();
        webServer.send(200, "text/plain", "Saved. Rebooting...");
        delay(1000); ESP.restart();
    });

    webServer.on("/savePass", HTTP_POST, [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        strncpy(config.wwwUser, webServer.arg("user").c_str(), 32);
        if (webServer.arg("pass").length() > 0)
            strncpy(config.wwwPass, webServer.arg("pass").c_str(), 32);
        EEPROM.put(1, config); EEPROM.commit();
        webServer.send(200, "text/plain", "Saved. Please log in again.");
    });

    webServer.on("/saveSource", HTTP_POST, [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        config.useTcpDataSource = (webServer.arg("source_mode") == "1");
        strncpy(config.dataSourceHost, webServer.arg("host").c_str(), sizeof(config.dataSourceHost)-1);
        config.dataSourcePort = (uint16_t)webServer.arg("port").toInt();
        EEPROM.put(1, config); EEPROM.commit();
        webServer.send(200, "text/plain", "Saved. Rebooting...");
        delay(1000); ESP.restart();
    });

    webServer.on("/config/emu", [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        logHeap("config_emu:start");

        String h = "<html><head><title>Emulation Modes</title>" + String(dashStyle) + "</head><body><div class='container'><h1>EMULATION MODES</h1>"
                   "<div class='stat diag'>Configure third-party device emulation (e.g. Marstek/Shelly).</div>"
                   "<form method='POST' action='/saveEmu'>"
                   "<div><label style='display:flex;align-items:center;justify-content:center;'>"
                   "<input type='checkbox' name='emu_on' value='1' style='width:auto;margin-right:10px;'" + String(config.emuEnabled ? "checked" : "") + ">Enable Shelly/Marstek Emulation</label></div>"
                   "<div style='margin-top:20px;'><strong>EMULATOR PORT:</strong>"
                   "<input name='emu_port' type='number' placeholder='Port (Default: 2220)' value='" + String(config.marstekPort) + "'></div>"
                   "<div style='font-size:0.8em;color:#888;margin-bottom:20px; text-align:left;'>"
                   "&bull; Port 2220: Modern Marstek (Shelly Pro 3EM Gen2)<br>"
                   "&bull; Port 1010: Legacy Marstek (Shelly 3EM Gen1)</div>"
                   "<button class='btn'>SAVE & REBOOT</button></form>"
                   "<a href='/settings' class='btn' style='background:#333;color:#fff;'>BACK</a>" + getFooter() + "</div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/saveEmu", HTTP_POST, [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        config.emuEnabled = (webServer.arg("emu_on") == "1");
        config.marstekPort = (uint16_t)webServer.arg("emu_port").toInt();
        if (config.marstekPort == 0) config.marstekPort = 2220;
        EEPROM.put(1, config); EEPROM.commit();
        webServer.send(200, "text/plain", "Emulation settings saved. Rebooting...");
        delay(1000); ESP.restart();
    });

    webServer.on("/factReset", HTTP_POST, [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        for (int i = 0; i < 512; i++) EEPROM.write(i, 0);
        EEPROM.commit(); ESP.restart();
    });

    webServer.on("/reboot", HTTP_POST, [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        webServer.send(200, "text/plain", "Rebooting...");
        delay(100);
        ESP.restart();
    });

    webServer.on("/update", HTTP_GET, [](){
        if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        webServer.send(200, "text/html",
            "<html><head>" + String(dashStyle) + "</head><body><div class='container'><h1>UPDATE</h1>"
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='update'><button class='btn'>FLASH</button></form>"
            "</div></body></html>");
    });

    webServer.on("/update", HTTP_POST,
        [](){
            if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
            webServer.send(200, "text/plain", "OK"); delay(1000); ESP.restart();
        },
        [](){
            if (!webServer.authenticate(config.wwwUser, config.wwwPass)) return;
            HTTPUpload& u = webServer.upload();
            if      (u.status == UPLOAD_FILE_START)  Update.begin((ESP.getFreeSketchSpace()-0x1000) & 0xFFFFF000);
            else if (u.status == UPLOAD_FILE_WRITE)  Update.write(u.buf, u.currentSize);
            else if (u.status == UPLOAD_FILE_END)    Update.end(true);
        }
    );

    webServer.onNotFound(handleRoot);
    webServer.begin();

    // Initialize the Shelly/Marstek Emulator
    if (config.emuEnabled) setupShellyEmulator();

    tcpServer.begin(config.tcpServerPort);
}

void processDataByte(char c) {
    if (!apMode) lastDataReceived = millis();

    if (dsmr_parse_byte(&dsmr_parser, (uint8_t)c)) {
        // Frame complete and CRC-validated.
        // Only update dsmr_last_good when frame_complete is set
        // (already guaranteed by dsmr_parse_byte returning 1, but being explicit
        // here in case the guard logic ever changes in the parser).
        if (dsmr_data.meter_id[0] != '\0' && dsmr_data.frame_complete) {
            dsmr_last_good = dsmr_data;
        }
        dsmr_parser_init(&dsmr_parser, &dsmr_data);
    }

    // 1. Buffer Full / Delimiter Logic
    // Ensure we don't overflow the buffer. If full, flush before adding.
    if (broadcastLen >= sizeof(broadcastBuf)) {
        flushBroadcast();
    }
    
    broadcastBuf[broadcastLen++] = c;
    lastBroadcastByteTime = millis();

    // If we reached the end of the telegram ('!'), flush immediately for low latency.
    if (c == '!') {
        flushBroadcast();
    }

    // A '/' always marks the start of a new, clean frame. Reset raw buffer.
    if (c == '/') {
        if (tempBufferIndex > 0) Serial.println(F("[RAW] Discarding incomplete frame, new one starting."));
        tempBufferIndex = 0;
        tempOverflowed = false;
        frameEndDetected = false;
    }

    // If we are in an overflow state, we ignore all incoming bytes until the next '/'
    // which will reset the tempOverflowed flag.
    if (tempOverflowed) {
        return;
    }

    // Add the current byte to our temporary buffer if there is space.
    if (tempBufferIndex < (MAX_FRAME_SIZE-1)) {
        tempBuffer[tempBufferIndex++] = c;
    }

    // Now, check if adding that byte caused an overflow.
    if (tempBufferIndex >= MAX_FRAME_SIZE) {
        Serial.println(F("[RAW] Buffer Overflow! Discarding frame. Waiting for next '/'."));
        tempBufferIndex = 0;
        tempOverflowed = true;  // Set the overflow state.
        frameEndDetected = false;
        return; // Ignore the rest of this broken frame.
    }

    if (c == '!') {
        frameEndDetected = true;
    }

    if (frameEndDetected && c == '\n') {
        tempBuffer[tempBufferIndex] = '\0'; // Null-terminate C-string
        lastFrameBuffer = tempBuffer;
        Serial.printf("[RAW] Frame captured (%u bytes)\n", tempBufferIndex);
        tempBufferIndex = 0;
        tempOverflowed = false;
        frameEndDetected = false;
        ESP.wdtFeed();
    }
}

void loop() {
    ESP.wdtFeed();

    // If we were in AP mode but the station has now connected, transition to client mode.
    // This handles the case where the target WiFi network was down at boot but came
    // back online later. The ESP8266 background process will connect automatically.
    // NOTE: WiFi.status() reflects the STATION interface (connection to Router).
    // It does not trigger if a user connects to our SoftAP.
    if (config.systemMode == MODE_CLIENT && apMode && WiFi.status() == WL_CONNECTED) {
        Serial.println(F("\n[NET] Station connected to Uplink, transitioning from AP to Client mode."));
        apMode = false;

        dnsServer.stop();
        
        // Only disable SoftAP if no one is using it. If a user is configuring
        // the device (e.g. scanning), cutting the AP would look like a failure.
        if (WiFi.softAPgetStationNum() == 0) {
            WiFi.mode(WIFI_STA);
        }

        Serial.println("[NET] ONLINE: " + WiFi.localIP().toString());
        MDNS.begin("smr");

        // Initialize client-mode timers now that we are online
        bootTime          = millis();
        lastDataReceived  = bootTime;
        lastClientCheck   = lastDataReceived;
        lastWiFiCheck     = lastClientCheck;      
    }

    if (apMode) dnsServer.processNextRequest();
    MDNS.update();
    webServer.handleClient();
    if (config.emuEnabled) loopShellyEmulator();

    // 2. Idle Timeout Logic
    // If data is sitting in the buffer but no new data has arrived for 100ms, flush it.
    if (broadcastLen > 0 && (millis() - lastBroadcastByteTime > 100)) {
        Serial.printf("[TCP] Idle flush (%d bytes)\n", broadcastLen);
        flushBroadcast();
    }

    // SOFTWARE WATCHDOG - client mode only
    // Run watchdog if we are operating (ACP Mode or Client Connected)
    // In ACP mode, apMode is true, but we are operating.
    if ((config.systemMode == MODE_ACP) || (!apMode)) {
        unsigned long now = millis();
        bool bootGracePassed = (now - bootTime) > BOOT_GRACE_PERIOD;

        // WiFi Watchdog: Only for Client Mode
        if (config.systemMode == MODE_CLIENT && bootGracePassed && WiFi.status() != WL_CONNECTED) {
            if (now - lastWiFiCheck > WIFI_TIMEOUT) {
                Serial.println("\n[WATCHDOG] WiFi lost 5 min. Rebooting...");
                delay(1000); ESP.restart();
            }
        } else {
            lastWiFiCheck = now;
        }

        if (now - lastDataReceived > WATCHDOG_TIMEOUT) {
            Serial.println("\n[WATCHDOG] No data 10 min. Rebooting...");
            delay(1000); ESP.restart();
        }

        if (activeClients == 0 && now - lastClientCheck > WATCHDOG_TIMEOUT) {
            Serial.println("\n[WATCHDOG] No clients 10 min. Rebooting...");
            delay(1000); ESP.restart();
        }

        if (activeClients > 0) lastClientCheck = now;
    }

    // Accept new TCP clients
    if (tcpServer.hasClient()) {
        WiFiClient newClient = tcpServer.accept();
        
        // Disable Nagle's algorithm to ensure bytes are sent immediately 
        // and don't contribute to buffer pressure unnecessarily.
        newClient.setNoDelay(true);
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (!TCPClient[i] || !TCPClient[i].connected()) {
                TCPClient[i] = newClient; break;
            }
        }
    }

    // --- DATA SOURCE ---
    if (config.useTcpDataSource && ((config.systemMode == MODE_ACP) || !apMode)) {
        if (!dataSourceClient.connected()) {
            static unsigned long lastTcpConnectAttempt = 0;
            if (millis() - lastTcpConnectAttempt > 5000) {
                lastTcpConnectAttempt = millis();
                Serial.printf("[TCP-SRC] Connecting to %s:%u...\n", config.dataSourceHost, config.dataSourcePort);
                if (dataSourceClient.connect(config.dataSourceHost, config.dataSourcePort)) {
                    dataSourceClient.setNoDelay(true);
                    Serial.println("[TCP-SRC] Connection established. Waiting for start of next data frame ('/')...");
                } else {
                    // "Connecting..." message is sufficient, no need to log "Failed" every 5 seconds.
                }
            }
        }
        // Only process data if we are actually connected.
        if (dataSourceClient.connected()) {
            int limit = 512; // Process max bytes per loop to keep UI responsive
            while (dataSourceClient.available() && limit-- > 0) {
                processDataByte(dataSourceClient.read());
                if (limit % 64 == 0) yield(); // Give the WiFi stack time to breathe
            }
        }
    } else {
        int limit = 512;
        while (Serial.available() && limit-- > 0) {
            processDataByte(Serial.read());
            if (limit % 64 == 0) yield();
        }
    }

    // Count active TCP streaming clients
    int count = 0;
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (TCPClient[i].connected()) count++;
    }

    // Reset the client watchdog timer when the last client disconnects,
    // so the 10-minute countdown starts from the moment of disconnect rather
    // than from boot (or the last time we happened to update lastClientCheck).
    static int prevActiveClients = 0;
    if (count == 0 && prevActiveClients > 0) {
        lastClientCheck = millis();  // Start fresh countdown from now
    }
    prevActiveClients = count;
    activeClients     = count;
}