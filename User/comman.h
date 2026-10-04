/********************************** (C) COPYRIGHT *******************************
 * File Name          : comman.h
 * Description        : Common System Definitions for CH32V203 IoT Gateway
 *******************************************************************************/
#ifndef __COMMAN_H
#define __COMMAN_H

#include "ch32v20x.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ==================== Flash Layout & Non-Volatile Memory (NVM) ==================== */
#define NVM_MAGIC_HEADER        0x53574149UL  /* 'SWAI' Magic Key */
#define NVM_CONFIG_VERSION      0x0009U
#define CONFIG_AUTH_PASS        "$$Swasemi$11cr$$"
#define NVM_SECTOR_ADDR         0x00000000UL  /* Sector 0 in W25Q32 reserved for NVM config */
#define FLASH_LOG_START_ADDR    0x00001000UL  /* Sector 1 (4KB offset) onwards reserved for data logs */

#define DataFrameByte           256
#define TotalFrameinflash       16384
#define FLASH_FRAME_SIZE        256U
#define MAX_MODBUS_SENSORS      16U

typedef struct {
    uint8_t  year;
    uint8_t  month;
    uint8_t  date;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    float    latitude;
    float    longitude;
    uint16_t Signal_Strength;
    char     NSIndi;
    char     EWIndi;
} Run_Send_t;

typedef struct {
    uint8_t  g_sqw_flag;
    uint8_t  F_Power_On;
    uint8_t  SendCnt;
    char     Device_ID[32];
} tempery;

#define MODBUS_ENDIAN_BIG           0U  /* ABCD: Big-Endian (MSB first) */
#define MODBUS_ENDIAN_LITTLE        1U  /* DCBA: Little-Endian (LSB first) */
#define MODBUS_ENDIAN_BIG_SWAP      2U  /* BADC: Big-Endian Byte Swap */
#define MODBUS_ENDIAN_LITTLE_SWAP   3U  /* CDAB: Little-Endian Byte Swap (Word Swap) */

/* Individual Modbus Sensor Channel Configuration (s1 to s16) */
typedef struct {
    uint8_t  enabled;           /* 1 if channel is active, 0 if disabled */
    uint8_t  slave_id;          /* Modbus Slave Address (device_id: 1 - 247) */
    uint8_t  function_code;     /* Function Code: 3 (Holding) or 4 (Input), default 3 */
    uint8_t  retry_count;       /* Retry Count (default: 3) */
    uint16_t start_register;    /* Start Register Address: 0 - 65535 */
    uint16_t num_registers;     /* Number of registers to read: 1 - 16 */
    uint16_t timeout_ms;        /* Response Timeout in ms (default: 1000) */
    uint32_t poll_interval_ms;  /* Sensor Polling Interval in ms (default: 5000) */
    char     sensor_type[16];   /* Sensor Type/Name (e.g. "temperature", "humidity", "pressure", "solar") */
    char     value_format[16];  /* Value Format: "signed", "unsigned", "32bit-signed", "32bit-unsigned", "float" */
    uint8_t  endianness;        /* 0: Big-Endian, 1: Little-Endian, 2: Big-Endian Byte Swap, 3: Little-Endian Byte Swap */
    uint8_t  decimal_point;     /* 0 to 6 decimal places (divides by 10^dp) */
} Modbus_Sensor_Config_t;

typedef struct {
    uint16_t Sample_Time;           /* Sampling / publishing interval in seconds (default: 60) */
    char     Device_ID[16];         /* Device Name / ID (default: "GTW26090001") */
    char     Serial_no[20];         /* Hardware Serial Number (default: "GTW26090001") */
    char     MQTTServer[48];        /* MQTT Broker Hostname or IP */
    uint16_t MQTTPort;              /* MQTT Broker Port (e.g. 1883) */
    char     MQTTUsername[24];      /* MQTT Auth Username */
    char     MQTTPassword[24];      /* MQTT Auth Password */
    char     MQTTPubTopic[24];      /* MQTT Publish Topic */
    uint8_t  mul_factor;            /* Multiplier Factor (default: 1) */

    /* RS-485 Modbus Bus Configuration */
    uint32_t sensor_baud;           /* Modbus RS-485 Baudrate (default: 9600) */
    char     sensor_parity[6];      /* Parity: "none", "even", "odd" (default: "none") */
    uint8_t  sensor_stop_bits;      /* Stop bits: 1 or 2 (default: 1) */
    uint8_t  sensor_data_bits;      /* Data bits: 7 or 8 (default: 8) */
    uint8_t  total_sensors;         /* Number of active sensors */

    /* 16 Modbus Sensor Channels (s1 to s16) */
    Modbus_Sensor_Config_t sensors[MAX_MODBUS_SENSORS];
} memory;

typedef struct {
    uint32_t magic;                 /* NVM_MAGIC_HEADER */
    uint16_t version;               /* NVM_CONFIG_VERSION */
    uint16_t checksum;              /* 16-bit additive checksum of memory settings */
    memory   settings;              /* Configuration parameters */
} NVM_Storage_t;

typedef struct {
    uint32_t in_count;
    uint32_t Massege_No;
    uint32_t flash_Add;
    uint16_t frame_idx;
    uint16_t sector;
    uint32_t sector_addr;
    uint16_t Rd_Frame_idx;
    uint8_t  FlashFull;
    uint8_t  FileNo;
} power_fail;

extern tempery     temp;
extern memory      first;
extern power_fail  PF;
extern Run_Send_t  iot_send;
extern Run_Send_t  Run;
extern Run_Send_t  Send;
extern uint32_t    g_current_flash_addr;
extern uint32_t    g_total_logged_frames;
extern volatile uint8_t  g_usb_dump_active;
extern volatile uint32_t g_usb_dump_curr_addr;
extern volatile uint32_t g_usb_dump_max_addr;
extern volatile uint32_t g_usb_dump_count;

extern uint8_t    g_last_configured_sensor_idx;

/* NVM & System functions */
void     NVM_Init(void);
void     NVM_SetDefaults(memory *cfg);
uint8_t  NVM_SaveConfig(void);
void     NVM_ResetToDefaults(void);
void     NVM_PrintConfig(void);
uint8_t  Config_ApplyGeneralJson(const char *json_payload);
uint8_t  Config_ApplyTimeJson(const char *json_payload);
uint8_t  Sensor_ApplyConfigJson(const char *json_payload);
void     Sensor_PrintConfigJson(void);
uint16_t Sensor_GenerateConfigJsonResponse(char *out_buf, uint16_t max_len, const char *act);
uint16_t NVM_CalcChecksum(const memory *cfg);
void     JsonExtractStr(const char *json, const char *key, char *out_buf, size_t max_len, const char *default_str);
int      JsonExtractInt(const char *json, const char *key, int default_val);
int      JsonExtractHex(const char *json, const char *key, int default_val);

void     Flash_Log_Init(void);
void     Reset_EEPROM(void);
void     Device_Reset(void);
void     SaveFrameToFlash(const char *json_frame);

/* Non-blocking Asynchronous USB Data Streaming */
void     USB_StartDump(void);
void     USB_StopDump(void);
void     USB_Dump_Process_Slice(void);
void     USB_DumpAllReadingData(void);

#endif /* __COMMAN_H */
