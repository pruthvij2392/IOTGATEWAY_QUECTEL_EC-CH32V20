/********************************** (C) COPYRIGHT *******************************
 * File Name          : modbus_rtu.h
 * Description        : Modbus RTU RS-485 Master Driver for Pressure Sensor
 *                      - RS-485: USART1 PA9=TX, PA10=RX, PA8=DE (Driver Enable)
 *                      - Slave Address : 0x02
 *                      - Query Command : 02 03 00 00 00 02 C4 38
 *                      - Response Frame: 02 03 04 HH HL LH LL CRC_L CRC_H
 *******************************************************************************/
#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "ch32v20x.h"
#include <stdint.h>

#define MAX_MODBUS_SENSORS      16U
#define MODBUS_RX_BUF_SIZE      128U

typedef struct {
    uint8_t  sensor_online;
    uint8_t  retry_counter;
    uint8_t  reg_count;
    uint32_t last_poll_tick;
    uint32_t last_response_tick;
    uint32_t total_queries;
    uint32_t total_responses;
    uint32_t total_crc_errors;
    uint16_t registers[4];      /* Read registers from sensor (1 to 4 registers) */
    float    parsed_value;      /* Computed primary value */
    char     val_str[16];       /* Pure numeric string for JSON telemetry (e.g. "31.5", "65.2", "750", "0000") */
    char     display_str[24];   /* e.g. "29.56 C", "60.35 %", "1002.00 hPa" */
} Modbus_Sensor_Runtime_t;

/* Backward compatible pressure structure */
typedef struct {
    uint8_t  slave_id;
    uint32_t raw_pressure;
    float    pressure_hpa;
    float    pressure_kpa;
    float    pressure_bar;
    uint8_t  sensor_online;
} Modbus_Pressure_t;

extern Modbus_Sensor_Runtime_t g_sensors_runtime[MAX_MODBUS_SENSORS];
extern Modbus_Pressure_t       g_modbus_pressure;
extern uint8_t                 g_current_sensor_idx;

/* Function Prototypes */
void     Modbus_Init(uint32_t baudrate);
void     Modbus_SendQuery(void);
void     Modbus_SendQueryForIndex(uint8_t sensor_idx);
void     Modbus_Process_Task(uint32_t current_tick);
uint16_t Modbus_CRC16(const uint8_t *buffer, uint16_t len);
uint8_t  Modbus_GetPressure(float *out_hpa, uint32_t *out_raw);

#endif /* __MODBUS_RTU_H */
