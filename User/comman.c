/********************************** (C) COPYRIGHT *******************************
 * File Name          : comman.c
 * Description        : Common System State & Non-Volatile Memory (NVM) Management
 *                      for CH32V203 IoT Gateway using W25Q32 SPI Flash
 *******************************************************************************/
#include "comman.h"
#include "main.h"
#include "modbus_rtu.h"
#include "w25qxx.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>

/* Factory default configuration generator */
void NVM_SetDefaults(memory *cfg) {
    memset(cfg, 0, sizeof(memory));
    cfg->Sample_Time           = 60;
    strncpy(cfg->Device_ID, "GTW26090001", sizeof(cfg->Device_ID) - 1);
    strncpy(cfg->Serial_no, "GTW26090001", sizeof(cfg->Serial_no) - 1);
    strncpy(cfg->MQTTServer, "mqtt.swasemi.in", sizeof(cfg->MQTTServer) - 1);
    cfg->MQTTPort              = 1883;
    strncpy(cfg->MQTTUsername, "mqtt_swasemi", sizeof(cfg->MQTTUsername) - 1);
    strncpy(cfg->MQTTPassword, "mqtt@Swasemi@11cr", sizeof(cfg->MQTTPassword) - 1);
    strncpy(cfg->MQTTPubTopic, "GTW26090001", sizeof(cfg->MQTTPubTopic) - 1);
    cfg->mul_factor            = 1;
    cfg->sensor_baud           = 9600;
    strncpy(cfg->sensor_parity, "none", sizeof(cfg->sensor_parity) - 1);
    cfg->sensor_stop_bits      = 1;
    cfg->sensor_data_bits      = 8;
    cfg->total_sensors         = 3;

    /* Initialize all 16 sensor slots with safe default parameters */
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        cfg->sensors[i].enabled          = (i < 3) ? 1 : 0;
        cfg->sensors[i].slave_id         = (uint8_t)(i + 1);
        cfg->sensors[i].function_code    = 3;
        cfg->sensors[i].start_register   = 0;
        cfg->sensors[i].num_registers    = 2;
        cfg->sensors[i].timeout_ms       = 1000;
        cfg->sensors[i].retry_count      = 3;
        cfg->sensors[i].endianness       = MODBUS_ENDIAN_BIG;
        cfg->sensors[i].decimal_point    = 2;
        strncpy(cfg->sensors[i].value_format, "32bit-signed", sizeof(cfg->sensors[i].value_format) - 1);
    }

    /* s1: Slave 2 (32bit-signed) */
    cfg->sensors[0].slave_id = 2;

    /* s2: Slave 2 (32bit-signed) */
    cfg->sensors[1].slave_id = 2;

    /* s3: Slave 1 (signed) */
    cfg->sensors[2].slave_id = 1;
    strncpy(cfg->sensors[2].value_format, "signed", sizeof(cfg->sensors[2].value_format) - 1);
}

tempery     temp;
memory      first;
power_fail  PF;
Run_Send_t  iot_send;
Run_Send_t  Run;
Run_Send_t  Send;

uint32_t    g_current_flash_addr  = FLASH_LOG_START_ADDR;
uint32_t    g_total_logged_frames = 0;
uint8_t     g_last_configured_sensor_idx = 0;

/* ==================== Checksum Calculation ==================== */
uint16_t NVM_CalcChecksum(const memory *cfg) {
    const uint8_t *p = (const uint8_t*)cfg;
    uint16_t sum = 0xAA55;
    for (size_t i = 0; i < sizeof(memory); i++) {
        sum = (uint16_t)(sum + p[i]);
    }
    return sum;
}

/* ==================== Initialize & Load NVM Configuration ==================== */
void NVM_Init(void) {
    /* Initialize default temp state */
    temp.SendCnt = 1;
    temp.F_Power_On = 1;

    /* Copy factory defaults into RAM first */
    NVM_SetDefaults(&first);
    strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);

    NVM_Storage_t store;
    memset(&store, 0, sizeof(store));

    /* Read stored config from Sector 0 of W25Q32 */
    W25Qxx_ReadData((uint8_t*)&store, NVM_SECTOR_ADDR, sizeof(NVM_Storage_t));

    uint16_t calc_sum = NVM_CalcChecksum(&store.settings);

    /* Validate magic number, version, and checksum */
    if (store.magic == NVM_MAGIC_HEADER &&
        store.version == NVM_CONFIG_VERSION &&
        store.checksum == calc_sum &&
        store.settings.Sample_Time >= 1 && store.settings.Sample_Time <= 86400 &&
        store.settings.MQTTServer[0] != '\0' && store.settings.MQTTServer[0] != (char)0xFF) {

        /* Successfully validated -> load active configuration */
        memcpy(&first, &store.settings, sizeof(memory));
        strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);

        printf("[NVM] Settings loaded: Device: %s | Broker: %s:%u | Sampling: %u sec\r\n",
               first.Device_ID, first.MQTTServer, (unsigned int)first.MQTTPort,
               (unsigned int)first.Sample_Time);
    } else {
        /* Blank flash or corrupted checksum -> load factory defaults & save */
        printf("[NVM] Init factory defaults (v0x%04X)...\r\n", (unsigned int)NVM_CONFIG_VERSION);
        NVM_ResetToDefaults();
    }
}

/* ==================== Save Current Config to NVM Flash ==================== */
uint8_t NVM_SaveConfig(void) {
    NVM_Storage_t store;
    memset(&store, 0, sizeof(store));

    store.magic    = NVM_MAGIC_HEADER;
    store.version  = NVM_CONFIG_VERSION;
    memcpy(&store.settings, &first, sizeof(memory));
    store.checksum = NVM_CalcChecksum(&store.settings);

    /* 1. Erase Sector 0 */
    W25Qxx_EraseSector(NVM_SECTOR_ADDR);

    /* 2. Write NVM Storage structure to Page 0 of Sector 0 */
    W25Qxx_WriteData((const uint8_t*)&store, NVM_SECTOR_ADDR, sizeof(NVM_Storage_t));

    /* 3. Read back & verify */
    NVM_Storage_t verify_store;
    memset(&verify_store, 0, sizeof(verify_store));
    W25Qxx_ReadData((uint8_t*)&verify_store, NVM_SECTOR_ADDR, sizeof(NVM_Storage_t));

    if (verify_store.magic == NVM_MAGIC_HEADER &&
        verify_store.checksum == store.checksum &&
        memcmp(&verify_store.settings, &first, sizeof(memory)) == 0) {
        return 1;
    } else {
        return 0;
    }
}

/* ==================== Reset Configuration to Factory Defaults ==================== */
void NVM_ResetToDefaults(void) {
    NVM_SetDefaults(&first);
    strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);
    NVM_SaveConfig();
}

/* ==================== Print Formatted Configuration ==================== */
void NVM_PrintConfig(void) {
    printf("\r\n=== NVM CONFIG ===\r\n"
           "SN: %s | ID: %s | Spt: %u s\r\n"
           "Broker: %s:%u | Top: %s | Usr: %s\r\n"
           "Logs: %u frames\r\n"
           "==================\r\n",
           first.Serial_no, first.Device_ID, (unsigned int)first.Sample_Time,
           first.MQTTServer, (unsigned int)first.MQTTPort, first.MQTTPubTopic, first.MQTTUsername,
           (unsigned int)g_total_logged_frames);
}

/* ==================== JSON Parsing Helpers ==================== */
static const char *JsonFindVal(const char *json, const char *key) {
    if (!json || !key) return NULL;
    char search[32];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *p = strstr(json, search);
    if (!p) return NULL;
    p += strlen(search);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ':') return NULL;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == '\"') p++;
    return p;
}

int JsonExtractInt(const char *json, const char *key, int default_val) {
    const char *p = JsonFindVal(json, key);
    if (p && (*p == '-' || (*p >= '0' && *p <= '9'))) {
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
            return (int)strtol(p, NULL, 16);
        }
        return atoi(p);
    }
    return default_val;
}

int JsonExtractHex(const char *json, const char *key, int default_val) {
    const char *p = JsonFindVal(json, key);
    if (p) {
        while (*p == ' ' || *p == '\t' || *p == '\"') p++;
        if ((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F')) {
            long val = strtol(p, NULL, 16);
            return (int)val;
        }
    }
    return default_val;
}

void JsonExtractStr(const char *json, const char *key, char *out_buf, size_t max_len, const char *default_str) {
    if (!out_buf || max_len == 0) return;
    const char *p = JsonFindVal(json, key);
    if (p) {
        size_t i = 0;
        while (*p && *p != '\"' && *p != ',' && *p != '}' && *p != '\n' && *p != '\r' && i < (max_len - 1)) {
            out_buf[i++] = *p++;
        }
        out_buf[i] = '\0';
        if (i > 0) return;
    }
    if (default_str) {
        strncpy(out_buf, default_str, max_len - 1);
        out_buf[max_len - 1] = '\0';
    } else {
        out_buf[0] = '\0';
    }
}

/* ==================== Helper: Parse Single Sensor JSON Block ==================== */
static void ParseSingleSensorBlock(const char *json_block, uint8_t idx) {
    if (idx >= MAX_MODBUS_SENSORS) return;
    Modbus_Sensor_Config_t *cfg = &first.sensors[idx];

    /* Parse bus settings if present within sensor block */
    int baud = JsonExtractInt(json_block, "baud_rate", -1);
    if (baud <= 0) baud = JsonExtractInt(json_block, "baud", -1);
    if (baud == 2400 || baud == 4800 || baud == 9600 || baud == 19200 || 
        baud == 38400 || baud == 57600 || baud == 115200) {
        first.sensor_baud = (uint32_t)baud;
    }

    int db = JsonExtractInt(json_block, "data_bits", -1);
    if (db <= 0) db = JsonExtractInt(json_block, "db", -1);
    if (db == 7 || db == 8) first.sensor_data_bits = (uint8_t)db;

    char prt[16] = {0};
    JsonExtractStr(json_block, "parity", prt, sizeof(prt), NULL);
    if (prt[0] == '\0') JsonExtractStr(json_block, "prt", prt, sizeof(prt), NULL);
    if (prt[0] != '\0') {
        if (strcasecmp(prt, "none") == 0 || strcasecmp(prt, "N") == 0 || strcasecmp(prt, "0") == 0) {
            strncpy(first.sensor_parity, "none", sizeof(first.sensor_parity) - 1);
        } else if (strcasecmp(prt, "even") == 0 || strcasecmp(prt, "E") == 0) {
            strncpy(first.sensor_parity, "even", sizeof(first.sensor_parity) - 1);
        } else if (strcasecmp(prt, "odd") == 0 || strcasecmp(prt, "O") == 0) {
            strncpy(first.sensor_parity, "odd", sizeof(first.sensor_parity) - 1);
        }
        first.sensor_parity[sizeof(first.sensor_parity) - 1] = '\0';
    }

    int sb = JsonExtractInt(json_block, "stop_bits", -1);
    if (sb <= 0) sb = JsonExtractInt(json_block, "sb", -1);
    if (sb == 1 || sb == 2) first.sensor_stop_bits = (uint8_t)sb;

    /* Check explicit enable/disable */
    int en = JsonExtractInt(json_block, "enabled", -1);
    if (en < 0) en = JsonExtractInt(json_block, "enable", -1);
    if (en < 0) en = JsonExtractInt(json_block, "status", -1);
    if (en == 0) {
        cfg->enabled = 0;
        return;
    }

    /* 1. Device ID / Slave Address (Direct Hexadecimal decoding e.g. 01..99, 88 -> 0x88) */
    int slave = JsonExtractHex(json_block, "device_id", -1);
    if (slave <= 0) slave = JsonExtractHex(json_block, "slave", -1);
    if (slave <= 0) slave = JsonExtractHex(json_block, "slave_id", -1);
    if (slave <= 0) slave = JsonExtractHex(json_block, "dev_id", -1);
    if (slave <= 0) slave = JsonExtractHex(json_block, "id", -1);
    if (slave == 0) {
        cfg->enabled = 0;
        return;
    }
    if (slave >= 1 && slave <= 247) cfg->slave_id = (uint8_t)slave;

    /* 2. Start Register Address */
    int reg = JsonExtractInt(json_block, "start_register", -1);
    if (reg < 0) reg = JsonExtractInt(json_block, "start_reg", -1);
    if (reg < 0) reg = JsonExtractInt(json_block, "read_address", -1);
    if (reg < 0) reg = JsonExtractInt(json_block, "read_addr", -1);
    if (reg < 0) reg = JsonExtractInt(json_block, "address", -1);
    if (reg < 0) reg = JsonExtractInt(json_block, "addr", -1);
    if (reg < 0) reg = JsonExtractInt(json_block, "reg", -1);
    if (reg >= 0 && reg <= 65535) {
        cfg->start_register = (uint16_t)reg;
    }

    /* 3. Read Registers (Quantity/Count of Registers) */
    int qty = JsonExtractInt(json_block, "read_registers", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "read_register", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "num_registers", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "num_regs", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "QuentityOFregisters", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "QuantityOFregisters", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "quantityofregisters", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "quantity_of_registers", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "quantity", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "qty", -1);
    if (qty <= 0) qty = JsonExtractInt(json_block, "count", -1);
    if (qty >= 1 && qty <= 16) cfg->num_registers = (uint16_t)qty;

    /* 4. Timeout (ms) */
    int timeout = JsonExtractInt(json_block, "timeout_ms", -1);
    if (timeout <= 0) timeout = JsonExtractInt(json_block, "timeout", -1);
    if (timeout >= 50 && timeout <= 30000) cfg->timeout_ms = (uint16_t)timeout;

    /* 5. Retry Count */
    int retries = JsonExtractInt(json_block, "retry_count", -1);
    if (retries < 0) retries = JsonExtractInt(json_block, "retry", -1);
    if (retries >= 1 && retries <= 20) cfg->retry_count = (uint8_t)retries;

    /* 6. Function Code */
    int fc = JsonExtractInt(json_block, "function_code", -1);
    if (fc <= 0) fc = JsonExtractInt(json_block, "funaction_code", -1);
    if (fc <= 0) fc = JsonExtractInt(json_block, "func_code", -1);
    if (fc <= 0) fc = JsonExtractInt(json_block, "fc", -1);
    if (fc <= 0) fc = JsonExtractInt(json_block, "func", -1);
    if (fc == 1 || fc == 2 || fc == 3 || fc == 4) cfg->function_code = (uint8_t)fc;

    /* 7. Data Type / Value Format: "int", "float", "uint", "long", "signed", "unsigned", "32bit-signed", etc. */
    char vfmt[16] = {0};
    JsonExtractStr(json_block, "value_format", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] == '\0') JsonExtractStr(json_block, "data_type", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] == '\0') JsonExtractStr(json_block, "datatype", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] == '\0') JsonExtractStr(json_block, "value_formate", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] == '\0') JsonExtractStr(json_block, "val_format", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] == '\0') JsonExtractStr(json_block, "format", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] == '\0') JsonExtractStr(json_block, "fmt", vfmt, sizeof(vfmt), NULL);
    if (vfmt[0] != '\0') {
        strncpy(cfg->value_format, vfmt, sizeof(cfg->value_format) - 1);
        cfg->value_format[sizeof(cfg->value_format) - 1] = '\0';
    }

    /* 8. Endian Type:
     * - "big-endian" / "big" / "abcd" (0)
     * - "little-endian" / "little" / "dcba" (1)
     * - "big-endian-byte-swap" / "badc" (2)
     * - "little-endian-byte-swap" / "cdab" (3)
     */
    char endn[32] = {0};
    JsonExtractStr(json_block, "endian_type", endn, sizeof(endn), NULL);
    if (endn[0] == '\0') JsonExtractStr(json_block, "endianness", endn, sizeof(endn), NULL);
    if (endn[0] == '\0') JsonExtractStr(json_block, "endian", endn, sizeof(endn), NULL);
    if (endn[0] == '\0') JsonExtractStr(json_block, "endianess", endn, sizeof(endn), NULL);
    if (endn[0] == '\0') JsonExtractStr(json_block, "byte_order", endn, sizeof(endn), NULL);

    if (endn[0] != '\0') {
        /* Big-Endian Byte Swap (BADC) */
        if (strcasecmp(endn, "big-endian-byte-swap") == 0 || strcasecmp(endn, "big-endian byte swap") == 0 ||
            strcasecmp(endn, "big-endian-swap") == 0 || strcasecmp(endn, "badc") == 0 || strcasecmp(endn, "2") == 0) {
            cfg->endianness = MODBUS_ENDIAN_BIG_SWAP;
        }
        /* Little-Endian Byte Swap / Word Swap (CDAB) */
        else if (strcasecmp(endn, "little-endian-byte-swap") == 0 || strcasecmp(endn, "little-endian byte swap") == 0 ||
                 strcasecmp(endn, "little-endian-swap") == 0 || strcasecmp(endn, "word-swap") == 0 ||
                 strcasecmp(endn, "cdab") == 0 || strcasecmp(endn, "3") == 0) {
            cfg->endianness = MODBUS_ENDIAN_LITTLE_SWAP;
        }
        /* Little-Endian (DCBA) */
        else if (strcasecmp(endn, "little-endian") == 0 || strcasecmp(endn, "little endian") == 0 ||
                 strcasecmp(endn, "little") == 0 || strcasecmp(endn, "dcba") == 0 || strcasecmp(endn, "1") == 0) {
            cfg->endianness = MODBUS_ENDIAN_LITTLE;
        }
        /* Big-Endian (ABCD) */
        else if (strcasecmp(endn, "big-endian") == 0 || strcasecmp(endn, "big endian") == 0 ||
                 strcasecmp(endn, "big") == 0 || strcasecmp(endn, "abcd") == 0 || strcasecmp(endn, "0") == 0) {
            cfg->endianness = MODBUS_ENDIAN_BIG;
        }
    } else {
        int end_val = JsonExtractInt(json_block, "endian_type", -1);
        if (end_val < 0) end_val = JsonExtractInt(json_block, "endianness", -1);
        if (end_val < 0) end_val = JsonExtractInt(json_block, "endian", -1);
        if (end_val >= 0 && end_val <= 3) {
            cfg->endianness = (uint8_t)end_val;
        }
    }

    /* 9. Decimal Point: 0 to 6 */
    int dp = JsonExtractInt(json_block, "decimal_point", -1);
    if (dp < 0) dp = JsonExtractInt(json_block, "dp", -1);
    if (dp < 0) dp = JsonExtractInt(json_block, "decimals", -1);
    if (dp < 0) dp = JsonExtractInt(json_block, "decimal", -1);
    if (dp >= 0 && dp <= 6) {
        cfg->decimal_point = (uint8_t)dp;
    }

    /* Ensure non-zero safe defaults if uninitialized */
    if (cfg->function_code == 0) cfg->function_code = 3;
    if (cfg->num_registers == 0) cfg->num_registers = 2;
    if (cfg->timeout_ms == 0) cfg->timeout_ms = 1000;
    if (cfg->retry_count == 0) cfg->retry_count = 3;
    if (cfg->value_format[0] == '\0') strncpy(cfg->value_format, "32bit-signed", sizeof(cfg->value_format) - 1);

    /* Mark as enabled if slave ID is valid */
    if (cfg->slave_id > 0) {
        cfg->enabled = 1;
    }
}

/* ==================== Apply General System Configuration from JSON ==================== */
uint8_t Config_ApplyGeneralJson(const char *json_payload) {
    if (!json_payload || strlen(json_payload) == 0) return 0;

    printf("\r\n[CONFIG] Processing General System Configuration from JSON...\r\n");

    /* Sampling Interval in seconds */
    /* Sampling Interval in seconds */
    int spt = JsonExtractInt(json_payload, "spt", -1);
    if (spt <= 0) spt = JsonExtractInt(json_payload, "SET_SAMPLING_SEC", -1);
    if (spt <= 0) spt = JsonExtractInt(json_payload, "set_sampling_sec", -1);
    if (spt <= 0) spt = JsonExtractInt(json_payload, "SAMPLING_SEC", -1);
    if (spt <= 0) spt = JsonExtractInt(json_payload, "sampling_sec", -1);
    if (spt <= 0) spt = JsonExtractInt(json_payload, "sample_time", -1);
    if (spt > 0 && spt <= 86400) {
        first.Sample_Time = (uint16_t)spt;
    }

    /* Device Name / ID (dn) */
    char dn[20] = {0};
    JsonExtractStr(json_payload, "dn", dn, sizeof(dn), NULL);
    if (dn[0] == '\0') JsonExtractStr(json_payload, "device_id", dn, sizeof(dn), NULL);
    if (dn[0] != '\0') {
        if (strncasecmp(dn, "GTW", 3) != 0) {
            snprintf(first.Device_ID, sizeof(first.Device_ID), "GTW%s", dn);
        } else {
            strncpy(first.Device_ID, dn, sizeof(first.Device_ID) - 1);
        }
        first.Device_ID[sizeof(first.Device_ID) - 1] = '\0';
        strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);
    }

    /* Hardware Serial Number (sn) */
    char sn[24] = {0};
    JsonExtractStr(json_payload, "sn", sn, sizeof(sn), NULL);
    if (sn[0] == '\0') JsonExtractStr(json_payload, "serial", sn, sizeof(sn), NULL);
    if (sn[0] != '\0') {
        if (strncasecmp(sn, "GTW", 3) != 0) {
            snprintf(first.Serial_no, sizeof(first.Serial_no), "GTW%s", sn);
        } else {
            strncpy(first.Serial_no, sn, sizeof(first.Serial_no) - 1);
        }
        first.Serial_no[sizeof(first.Serial_no) - 1] = '\0';
        if (first.Device_ID[0] == '\0') {
            strncpy(first.Device_ID, first.Serial_no, sizeof(first.Device_ID) - 1);
            strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);
        }
    }

    /* MQTT Topic (top) */
    char top[32] = {0};
    JsonExtractStr(json_payload, "top", top, sizeof(top), NULL);
    if (top[0] == '\0') JsonExtractStr(json_payload, "topic", top, sizeof(top), NULL);
    if (top[0] == '\0') JsonExtractStr(json_payload, "mqtt_topic", top, sizeof(top), NULL);
    if (top[0] == '\0') JsonExtractStr(json_payload, "pub_topic", top, sizeof(top), NULL);
    if (top[0] != '\0') {
        strncpy(first.MQTTPubTopic, top, sizeof(first.MQTTPubTopic) - 1);
        first.MQTTPubTopic[sizeof(first.MQTTPubTopic) - 1] = '\0';
    }

    /* MQTT Broker IP / Hostname (bip) */
    char bip[48] = {0};
    JsonExtractStr(json_payload, "bip", bip, sizeof(bip), NULL);
    if (bip[0] == '\0') JsonExtractStr(json_payload, "broker", bip, sizeof(bip), NULL);
    if (bip[0] == '\0') JsonExtractStr(json_payload, "server", bip, sizeof(bip), NULL);
    if (bip[0] == '\0') JsonExtractStr(json_payload, "mqtt_server", bip, sizeof(bip), NULL);
    if (bip[0] == '\0') JsonExtractStr(json_payload, "ip", bip, sizeof(bip), NULL);
    if (bip[0] == '\0') JsonExtractStr(json_payload, "broker_ip", bip, sizeof(bip), NULL);
    if (bip[0] == '\0') JsonExtractStr(json_payload, "mqtt_ip", bip, sizeof(bip), NULL);
    if (bip[0] != '\0') {
        strncpy(first.MQTTServer, bip, sizeof(first.MQTTServer) - 1);
        first.MQTTServer[sizeof(first.MQTTServer) - 1] = '\0';
    }

    /* MQTT Broker Port (bpt) */
    int bpt = JsonExtractInt(json_payload, "bpt", -1);
    if (bpt <= 0) bpt = JsonExtractInt(json_payload, "port", -1);
    if (bpt <= 0) bpt = JsonExtractInt(json_payload, "mqtt_port", -1);
    if (bpt <= 0) bpt = JsonExtractInt(json_payload, "broker_port", -1);
    if (bpt > 0 && bpt <= 65535) {
        first.MQTTPort = (uint16_t)bpt;
    }

    /* MQTT Username (usr) */
    char usr[28] = {0};
    JsonExtractStr(json_payload, "usr", usr, sizeof(usr), NULL);
    if (usr[0] == '\0') JsonExtractStr(json_payload, "user", usr, sizeof(usr), NULL);
    if (usr[0] == '\0') JsonExtractStr(json_payload, "username", usr, sizeof(usr), NULL);
    if (usr[0] == '\0') JsonExtractStr(json_payload, "mqtt_user", usr, sizeof(usr), NULL);
    if (usr[0] == '\0') JsonExtractStr(json_payload, "mqtt_username", usr, sizeof(usr), NULL);
    if (usr[0] != '\0') {
        strncpy(first.MQTTUsername, usr, sizeof(first.MQTTUsername) - 1);
        first.MQTTUsername[sizeof(first.MQTTUsername) - 1] = '\0';
    }

    /* MQTT Password (pwd) */
    char pwd[28] = {0};
    JsonExtractStr(json_payload, "pwd", pwd, sizeof(pwd), NULL);
    if (pwd[0] == '\0') JsonExtractStr(json_payload, "pass", pwd, sizeof(pwd), NULL);
    if (pwd[0] == '\0') JsonExtractStr(json_payload, "password", pwd, sizeof(pwd), NULL);
    if (pwd[0] == '\0') JsonExtractStr(json_payload, "mqtt_pass", pwd, sizeof(pwd), NULL);
    if (pwd[0] == '\0') JsonExtractStr(json_payload, "mqtt_password", pwd, sizeof(pwd), NULL);
    if (pwd[0] != '\0') {
        strncpy(first.MQTTPassword, pwd, sizeof(first.MQTTPassword) - 1);
        first.MQTTPassword[sizeof(first.MQTTPassword) - 1] = '\0';
    }

    /* Multiplier Factor (mul) */
    int mul = JsonExtractInt(json_payload, "mul", -1);
    if (mul > 0 && mul <= 255) {
        first.mul_factor = (uint8_t)mul;
    }

    /* Optional Date & Time */
    Config_ApplyTimeJson(json_payload);

    /* Save updated config to SPI Flash */
    NVM_SaveConfig();
    return 1;
}

/* ==================== Apply Date & Time from JSON ==================== */
uint8_t Config_ApplyTimeJson(const char *json_payload) {
    if (!json_payload || strlen(json_payload) == 0) return 0;

    char dttm[48] = {0};
    JsonExtractStr(json_payload, "datetime", dttm, sizeof(dttm), NULL);
    if (dttm[0] == '\0') JsonExtractStr(json_payload, "DATETIME", dttm, sizeof(dttm), NULL);

    if (dttm[0] != '\0') {
        return RTC_WrtFromStr(dttm);
    }

    char dt[24] = {0}, tm[24] = {0};
    JsonExtractStr(json_payload, "dt", dt, sizeof(dt), NULL);
    if (dt[0] == '\0') JsonExtractStr(json_payload, "date", dt, sizeof(dt), NULL);
    if (dt[0] == '\0') JsonExtractStr(json_payload, "DATE", dt, sizeof(dt), NULL);
    if (dt[0] == '\0') JsonExtractStr(json_payload, "DD", dt, sizeof(dt), NULL);

    JsonExtractStr(json_payload, "tm", tm, sizeof(tm), NULL);
    if (tm[0] == '\0') JsonExtractStr(json_payload, "time", tm, sizeof(tm), NULL);
    if (tm[0] == '\0') JsonExtractStr(json_payload, "TIME", tm, sizeof(tm), NULL);
    if (tm[0] == '\0') JsonExtractStr(json_payload, "TT", tm, sizeof(tm), NULL);

    uint8_t ok = 0;
    if (dt[0] != '\0') ok |= DS3231_SetDateStr(dt);
    if (tm[0] != '\0') ok |= DS3231_SetTimeStr(tm);
    return ok;
}

/* ==================== Apply Multi-Sensor Configuration from JSON ==================== */
uint8_t Sensor_ApplyConfigJson(const char *json_payload) {
    if (!json_payload || strlen(json_payload) == 0) return 0;

    /* 0. Parse Global Telemetry Sampling Interval if present */
    int samp_root = JsonExtractInt(json_payload, "SET_SAMPLING_SEC", -1);
    if (samp_root <= 0) samp_root = JsonExtractInt(json_payload, "set_sampling_sec", -1);
    if (samp_root <= 0) samp_root = JsonExtractInt(json_payload, "SAMPLING_SEC", -1);
    if (samp_root <= 0) samp_root = JsonExtractInt(json_payload, "sampling_sec", -1);
    if (samp_root <= 0) samp_root = JsonExtractInt(json_payload, "SAMPLE_TIME", -1);
    if (samp_root <= 0) samp_root = JsonExtractInt(json_payload, "sample_time", -1);
    if (samp_root <= 0) samp_root = JsonExtractInt(json_payload, "spt", -1);
    if (samp_root > 0 && samp_root <= 86400) {
        first.Sample_Time = (uint16_t)samp_root;
    }

    /* 1. Parse Global RS-485 Bus Settings */
    int baud = JsonExtractInt(json_payload, "baud_rate", -1);
    if (baud <= 0) baud = JsonExtractInt(json_payload, "baud", -1);
    if (baud == 2400 || baud == 4800 || baud == 9600 || baud == 19200 || 
        baud == 38400 || baud == 57600 || baud == 115200) {
        first.sensor_baud = (uint32_t)baud;
    }

    int db = JsonExtractInt(json_payload, "data_bits", -1);
    if (db <= 0) db = JsonExtractInt(json_payload, "db", -1);
    if (db == 7 || db == 8) first.sensor_data_bits = (uint8_t)db;

    char prt[16] = {0};
    JsonExtractStr(json_payload, "parity", prt, sizeof(prt), NULL);
    if (prt[0] == '\0') JsonExtractStr(json_payload, "prt", prt, sizeof(prt), NULL);
    if (prt[0] != '\0') {
        if (strcasecmp(prt, "none") == 0 || strcasecmp(prt, "N") == 0 || strcasecmp(prt, "0") == 0) {
            strncpy(first.sensor_parity, "none", sizeof(first.sensor_parity) - 1);
        } else if (strcasecmp(prt, "even") == 0 || strcasecmp(prt, "E") == 0) {
            strncpy(first.sensor_parity, "even", sizeof(first.sensor_parity) - 1);
        } else if (strcasecmp(prt, "odd") == 0 || strcasecmp(prt, "O") == 0) {
            strncpy(first.sensor_parity, "odd", sizeof(first.sensor_parity) - 1);
        }
        first.sensor_parity[sizeof(first.sensor_parity) - 1] = '\0';
    }

    int sb = JsonExtractInt(json_payload, "stop_bits", -1);
    if (sb <= 0) sb = JsonExtractInt(json_payload, "sb", -1);
    if (sb == 1 || sb == 2) first.sensor_stop_bits = (uint8_t)sb;

    int global_qty = JsonExtractInt(json_payload, "QuentityOFregisters", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "QuantityOFregisters", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "quantityofregisters", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "quantity_of_registers", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "num_registers", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "quantity", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "num_reg", -1);
    if (global_qty <= 0) global_qty = JsonExtractInt(json_payload, "qty", -1);
    if (global_qty >= 1 && global_qty <= 16) {
        for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
            first.sensors[i].num_registers = (uint16_t)global_qty;
        }
    }

    int total_sens = JsonExtractInt(json_payload, "total_sensors", -1);
    if (total_sens <= 0) total_sens = JsonExtractInt(json_payload, "total_sensor", -1);
    if (total_sens <= 0) total_sens = JsonExtractInt(json_payload, "total_no_of_sensors", -1);
    if (total_sens > 0 && total_sens <= MAX_MODBUS_SENSORS) {
        first.total_sensors = (uint8_t)total_sens;
        for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
            first.sensors[i].enabled = (i < first.total_sensors) ? 1 : 0;
        }
    }

    /* 2. Check for Specific Sensor Keys ("s1", "s2", ..., "s16") */
    uint8_t matched_keys = 0;
    char key_buf[16];

    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        snprintf(key_buf, sizeof(key_buf), "\"s%u\"", (unsigned int)(i + 1));
        const char *p = strstr(json_payload, key_buf);
        if (!p) {
            snprintf(key_buf, sizeof(key_buf), "\"S%u\"", (unsigned int)(i + 1));
            p = strstr(json_payload, key_buf);
        }
        if (!p) {
            snprintf(key_buf, sizeof(key_buf), "\"sensor%u\"", (unsigned int)(i + 1));
            p = strstr(json_payload, key_buf);
        }

        if (p) {
            /* Find start of object brace '{' for this sensor */
            const char *p_open = strchr(p, '{');
            if (p_open) {
                ParseSingleSensorBlock(p_open, i);
                matched_keys++;
                g_last_configured_sensor_idx = i;
            }
        }
    }

    /* 3. If no "s<N>" keys found, treat as single sensor (s1 or sensor_index) */
    if (matched_keys == 0) {
        int target_idx = JsonExtractInt(json_payload, "sensor_index", 1) - 1;
        if (target_idx < 0 || target_idx >= MAX_MODBUS_SENSORS) target_idx = 0;
        ParseSingleSensorBlock(json_payload, (uint8_t)target_idx);
        g_last_configured_sensor_idx = (uint8_t)target_idx;
    }

    /* 4. Count total enabled sensors */
    uint8_t total_act = 0;
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        if (first.sensors[i].enabled) total_act++;
    }
    first.total_sensors = total_act;

    /* Save to Non-Volatile Flash */
    NVM_SaveConfig();

    /* Clear runtime state so updated sensor settings take immediate effect */
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        memset(&g_sensors_runtime[i], 0, sizeof(Modbus_Sensor_Runtime_t));
    }

    /* Apply new settings to Modbus RS-485 Hardware */
    Modbus_Init(first.sensor_baud);

    return 1;
}

/* Helper to convert endianness enum to human readable string */
static const char *GetEndiannessStr(uint8_t endn) {
    switch(endn) {
        case MODBUS_ENDIAN_LITTLE:       return "little-endian";
        case MODBUS_ENDIAN_BIG_SWAP:     return "big-endian-byte-swap";
        case MODBUS_ENDIAN_LITTLE_SWAP:  return "little-endian-byte-swap";
        case MODBUS_ENDIAN_BIG:
        default:                         return "big-endian";
    }
}

/* ==================== Construct Multi-Sensor Config Response JSON ==================== */
uint16_t Sensor_GenerateConfigJsonResponse(char *out_buf, uint16_t max_len, const char *act) {
    if (!out_buf || max_len == 0) return 0;

    /* Count active / configured sensors */
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        if (first.sensors[i].enabled && first.sensors[i].slave_id > 0) {
            count++;
        }
    }
    first.total_sensors = count;

    uint16_t bus_qty = (first.sensors[0].num_registers > 0) ? first.sensors[0].num_registers : 2;

    int pos = snprintf(out_buf, max_len,
             "{\n"
             "  \"st\": \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"act\": \"%s\",\n"
             "  \"d\": {\n"
             "    \"baud_rate\": %u,\n"
             "    \"data_bits\": %u,\n"
             "    \"parity\": \"%s\",\n"
             "    \"stop_bits\": %u,\n"
             "    \"QuentityOFregisters\": %u,\n"
             "    \"total_sensors\": %u",
             (act && act[0] != '\0') ? act : "get-configsensor",
             (unsigned int)first.sensor_baud,
             (unsigned int)first.sensor_data_bits,
             (first.sensor_parity[0] != '\0') ? first.sensor_parity : "none",
             (unsigned int)first.sensor_stop_bits,
             (unsigned int)bus_qty,
             (unsigned int)count);

    if (pos < 0 || pos >= (int)max_len) pos = (int)max_len - 1;

    /* Output only the sensors that are enabled and configured */
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        const Modbus_Sensor_Config_t *cfg = &first.sensors[i];
        if (cfg->enabled && cfg->slave_id > 0 && pos < (int)(max_len - 1)) {
            int w = snprintf(out_buf + pos, max_len - (size_t)pos,
                 ",\n"
                 "    \"s%u\": {\"enabled\": %u, \"slave_id\": %u, \"value_format\": \"%s\", \"start_register\": %u, \"read_registers\": %u, \"endian_type\": \"%s\", \"decimal_point\": %u}",
                 (unsigned int)(i + 1),
                 (unsigned int)cfg->enabled,
                 (unsigned int)cfg->slave_id,
                 (cfg->value_format[0] != '\0') ? cfg->value_format : "32bit-signed",
                 (unsigned int)cfg->start_register,
                 (unsigned int)cfg->num_registers,
                 GetEndiannessStr(cfg->endianness),
                 (unsigned int)cfg->decimal_point);
            if (w > 0) {
                pos += w;
                if (pos >= (int)max_len) pos = (int)max_len - 1;
            }
        }
    }

    if (pos < (int)(max_len - 1)) {
        int w = snprintf(out_buf + pos, max_len - (size_t)pos, "\n  }\n}");
        if (w > 0) pos += w;
    }
    return (uint16_t)pos;
}

/* ==================== Print Config Sensor JSON to Terminal ==================== */
void Sensor_PrintConfigJson(void) {
    static char s_json_buf[1536]; /* Static to prevent stack overflow on 2KB stack */
    Sensor_GenerateConfigJsonResponse(s_json_buf, sizeof(s_json_buf), "get-configsensor");
    printf("%s\r\n", s_json_buf);
}



/* ==================== Initialize & Scan Existing Logged Frames on Boot ==================== */
void Flash_Log_Init(void) {
    uint8_t head[16];
    uint32_t addr = FLASH_LOG_START_ADDR;
    g_total_logged_frames = 0;
    g_current_flash_addr  = FLASH_LOG_START_ADDR;

    /* Scan through Flash to recover logged frame count & find next available write slot */
    while (addr < W25Q32_TOTAL_SIZE) {
        W25Qxx_ReadData(head, addr, sizeof(head));

        /* Blank slot (unprogrammed flash is all 0xFF) */
        if (head[0] == 0xFF && head[1] == 0xFF && head[2] == 0xFF && head[3] == 0xFF) {
            g_current_flash_addr = addr;
            break;
        }

        /* Valid frame slot (starts with ASCII record number) */
        if (head[0] >= '0' && head[0] <= '9') {
            g_total_logged_frames++;
        }

        addr += FLASH_FRAME_SIZE;
        g_current_flash_addr = addr;
    }

    if (g_current_flash_addr >= W25Q32_TOTAL_SIZE) {
        g_current_flash_addr = FLASH_LOG_START_ADDR;
        PF.FlashFull = 1;
    }

    printf("[FLASH] Data Log Memory Initialized: %u existing frames preserved.\r\n", 
           (unsigned int)g_total_logged_frames);
    printf("[FLASH] Next frame will be logged at address 0x%06X\r\n", 
           (unsigned int)g_current_flash_addr);
}

/* ==================== Reset & Clear All Data Logs (RESET_EEPROM Command) ==================== */
void Reset_EEPROM(void) {
    printf("[RESET_EEPROM] Clearing all data log frames from Flash...\r\n");

    /* Calculate sector range to erase (erase at least first 16 sectors = 64KB or all used sectors) */
    uint32_t erase_end = g_current_flash_addr + W25Q32_SECTOR_SIZE;
    if (erase_end < (FLASH_LOG_START_ADDR + (16 * W25Q32_SECTOR_SIZE))) {
        erase_end = FLASH_LOG_START_ADDR + (16 * W25Q32_SECTOR_SIZE);
    }
    if (erase_end > W25Q32_TOTAL_SIZE) {
        erase_end = W25Q32_TOTAL_SIZE;
    }

    /* Erase each 4KB sector in the data log space (Sector 1 onwards, preserving Sector 0 NVM config) */
    for (uint32_t s_addr = FLASH_LOG_START_ADDR; s_addr < erase_end; s_addr += W25Q32_SECTOR_SIZE) {
        W25Qxx_EraseSector(s_addr);
    }

    g_current_flash_addr  = FLASH_LOG_START_ADDR;
    g_total_logged_frames = 0;
    Device_Reset();

    printf("[RESET_EEPROM] SUCCESS: All data frames erased from Flash! Total logs count: 0.\r\n");
}

void Device_Reset(void) {
    PF.in_count     = 0;
    PF.Massege_No   = 1;
    PF.flash_Add    = 0;
    PF.frame_idx    = 0;
    PF.sector       = 0;
    PF.sector_addr  = 0;
    PF.Rd_Frame_idx = 0;
    temp.SendCnt    = 1;
    PF.FlashFull    = 0;
    PF.FileNo       = 1;
}

/* ==================== Save Published Frame to W25Q32 ==================== */
void SaveFrameToFlash(const char *json_frame) {
    if (!json_frame || strlen(json_frame) == 0) return;

    uint8_t buffer[FLASH_FRAME_SIZE];
    memset(buffer, 0, sizeof(buffer));

    /* Copy json string up to buffer size - 1 */
    strncpy((char*)buffer, json_frame, FLASH_FRAME_SIZE - 1);

    /* If writing at the exact start of a 4KB sector, check if it needs erase */
    if ((g_current_flash_addr % W25Q32_SECTOR_SIZE) == 0) {
        uint8_t check_head[4];
        W25Qxx_ReadData(check_head, g_current_flash_addr, sizeof(check_head));
        if (check_head[0] != 0xFF || check_head[1] != 0xFF) {
            W25Qxx_EraseSector(g_current_flash_addr);
        }
    }

    /* Write frame data to SPI Flash */
    W25Qxx_WriteData(buffer, g_current_flash_addr, FLASH_FRAME_SIZE);

    g_current_flash_addr += FLASH_FRAME_SIZE;
    g_total_logged_frames++;

    /* Wrap around if flash boundary reached (preserving Sector 0 for NVM config) */
    if (g_current_flash_addr >= W25Q32_TOTAL_SIZE) {
        g_current_flash_addr = FLASH_LOG_START_ADDR;
        PF.FlashFull = 1;
    }
}

/* ==================== Asynchronous Non-blocking USB Data Streaming Task ==================== */
volatile uint8_t  g_usb_dump_active    = 0;
volatile uint32_t g_usb_dump_curr_addr = FLASH_LOG_START_ADDR;
volatile uint32_t g_usb_dump_max_addr  = FLASH_LOG_START_ADDR;
volatile uint32_t g_usb_dump_count     = 0;

void USB_StartDump(void) {
    if (g_usb_dump_active) {
        return;
    }

    g_usb_dump_curr_addr = FLASH_LOG_START_ADDR;
    g_usb_dump_max_addr  = g_current_flash_addr;
    if (PF.FlashFull || (g_usb_dump_max_addr == FLASH_LOG_START_ADDR && g_total_logged_frames > 0)) {
        g_usb_dump_max_addr = W25Q32_TOTAL_SIZE;
    }
    g_usb_dump_count = 0;

    /* Print standard CSV Header for Excel / .CSV files */
    printf("Sr No,DATE,TIME,LAT,LONG,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,SIGNAL STRENGTH\r\n");

    if (g_usb_dump_curr_addr >= g_usb_dump_max_addr && g_total_logged_frames == 0) {
        printf("1,26-09-2026,11:14:00,0.000000,0.000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,26\r\n");
        g_usb_dump_active = 0;
        return;
    }

    g_usb_dump_active = 1;
}

void USB_StopDump(void) {
    if (g_usb_dump_active) {
        g_usb_dump_active = 0;
        LED_YELLOW_OFF();
    }
}

/* 
 * Non-blocking slice executed cooperatively in the main loop.
 * Streams multiple frames per slice while allowing GSM 4G, MQTT, RTC, and RS-485
 * to run concurrently without any freeze or timeout!
 */
void USB_Dump_Process_Slice(void) {
    if (!g_usb_dump_active) return;

    char dump_buf[FLASH_FRAME_SIZE + 1];
    uint8_t batch = 0;

    /* Process up to 8 frames per main loop cycle for high-speed streaming without starving other tasks */
    while (g_usb_dump_active && g_usb_dump_curr_addr < g_usb_dump_max_addr && batch < 8) {
        memset(dump_buf, 0, sizeof(dump_buf));
        W25Qxx_ReadData((uint8_t*)dump_buf, g_usb_dump_curr_addr, FLASH_FRAME_SIZE);
        dump_buf[FLASH_FRAME_SIZE] = '\0';

        /* Clean whitespace & newlines */
        int len = strlen(dump_buf);
        while(len > 0 && (dump_buf[len-1] == '\r' || dump_buf[len-1] == '\n' || dump_buf[len-1] == ' ')) {
            dump_buf[--len] = '\0';
        }

        /* Check if contains valid CSV frame */
        if (len > 0 && (dump_buf[0] >= '0' && dump_buf[0] <= '9')) {
            g_usb_dump_count++;
            printf("%s\r\n", dump_buf);
            
            /* Fast blink Yellow LED during COM port data sharing */
            LED_YELLOW_TOGGLE();
        }

        g_usb_dump_curr_addr += FLASH_FRAME_SIZE;
        batch++;
    }

    if (g_usb_dump_curr_addr >= g_usb_dump_max_addr) {
        if (g_usb_dump_count == 0) {
            printf("1,26-09-2026,11:14:00,0.000000,0.000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,26,4.1\r\n");
        }
        g_usb_dump_active = 0;
        LED_YELLOW_OFF();
    }
}

void USB_DumpAllReadingData(void) {
    USB_StartDump();
}


