/********************************** (C) COPYRIGHT *******************************
 * File Name          : gsm_quectel.h
 * Description        : Quectel 4G Module (EG800G/EC200U) Driver with MQTT for CH32V203
 *                      - Pins: iot en = PB4, 4g pwr = PB3, pcu_tx = PA2, pcu_rx = PA3
 *                      - MQTT Broker: mqtt.swasemi.in:1883
 *                      - Credentials: user mqtt_swasemi, pass MQTT@143
 *                      - Topic: IOTGATEWAY
 *******************************************************************************/
#ifndef __GSM_QUECTEL_H
#define __GSM_QUECTEL_H

#include "ch32v20x.h"
#include <stdint.h>

/* ==================== IoT Module Pin Definitions ==================== */
/* iot en = pb4, 4g pwr = pb3, pcu_tx = pa2, pcu_rx = pa3 */
#define IOT_EN_PORT         GPIOB
#define IOT_EN_PIN          GPIO_Pin_4    /* PB4 - IoT Module Power Enable */

#define GSM_PWR_PORT        GPIOB
#define GSM_PWR_PIN         GPIO_Pin_3    /* PB3 - 4G Power / PWRKEY */

#define GSM_UART            USART2
#define GSM_TX_PORT         GPIOA
#define GSM_TX_PIN          GPIO_Pin_2    /* PA2 - MCU TX -> GSM RX (pcu_tx) */
#define GSM_RX_PORT         GPIOA
#define GSM_RX_PIN          GPIO_Pin_3    /* PA3 - MCU RX <- GSM TX (pcu_rx) */

#define GSM_RX_BUFFER_SIZE  1024

/* ==================== MQTT Configuration ==================== */
#define MQTT_BROKER         "mqtt.swasemi.in"
#define MQTT_PORT           1883
#define MQTT_CLIENT_ID      "CH32V203_GATEWAY"
#define MQTT_PUB_TOPIC      "IOTGATEWAY"
#define MQTT_USERNAME       "mqtt_swasemi"
#define MQTT_PASSWORD       "mqtt@Swasemi@11cr"

/* ==================== Sensor Data Structure ==================== */
typedef struct {
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
    uint16_t particle_count;
} SensorData_t;

/* ==================== GSM Network Time Structure ==================== */
typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} GSM_Time_t;

/* ==================== GSM State Machine States ==================== */
typedef enum {
    GSM_STATE_INIT,
    GSM_STATE_ATE0,
    GSM_STATE_SIM_CHECK,
    GSM_STATE_NETWORK_REG,
    GSM_STATE_GPS_INIT,       /* Turn ON GPS engine */
    GSM_STATE_CTZU,           /* Auto time zone update */
    GSM_STATE_CCLK_QUERY,     /* Query network time (AT+QLTS=2 / AT+CCLK?) */
    GSM_STATE_PARSE_TIME,     /* Parse time string */
    GSM_STATE_QMTOPEN,
    GSM_STATE_QMTCONN,
    GSM_STATE_QMTSUB,         /* Subscribe to SWA/<Device_ID>/CONFIG/REQ */
    GSM_STATE_PUBLISH,
    GSM_STATE_PUBLISH_WAIT,
    GSM_STATE_DISCONNECT,
    GSM_STATE_STANDBY,
    GSM_STATE_ERROR
} GSM_State_t;

/* ==================== Function Prototypes ==================== */
void GSM_Pins_Init(void);
void GSM_PowerOn(void);
void GSM_Init(void);
void GSM_ProcessState(void);
void GSM_SendCommand(const char *cmd);
void GSM_SendData(const uint8_t *data, uint16_t len);
uint8_t GSM_CheckResponse(const char *expected);
void GSM_WaitForResponse(const char *expected, uint16_t timeout_ms);
void GSM_PublishSensorData(void);
void GSM_PublishConfigRequest(void);
void GSM_PublishConfigResponse(void);
void GSM_HandleSetConfig(const char *json_payload, const char *target_id);
void GSM_HandleSetTime(const char *json_payload, const char *target_id);
void GSM_HandleRemoteResetEEPROM(const char *target_id);
void GSM_HandleRemoteResetConfig(const char *target_id);
void GSM_HandleRemoteReset(const char *target_id);
void GSM_CheckIncomingMessages(void);
uint8_t GSM_IsConnected(void);
uint8_t GSM_IsNetworkRegistered(void);
uint8_t GSM_IsGPSFixed(void);
void GSM_TriggerPublish(void);
uint8_t GSM_SyncNetworkTime(void);
uint8_t ParseCCLKTime(char *response, GSM_Time_t *time);
void ParseCSQ(char *response);
void ProcessGPSFrame(const char *response);
void GPS_DATAConversion(char *GpsStr, uint8_t CalLen, uint8_t CalOpr);
void READING_FRAME(char *out_frame, uint16_t max_len);
void GENERATE_CSV_FRAME(char *out_csv, uint16_t max_len, uint32_t record_num);

/* ==================== External Variables ==================== */
extern volatile GSM_State_t g_gsm_state;
extern volatile uint8_t g_gsm_response_ready;
extern uint8_t g_gsm_rx_buffer[GSM_RX_BUFFER_SIZE];
extern uint16_t g_gsm_rx_len;
extern SensorData_t g_sensor_data;
extern GSM_Time_t g_gsm_time;
extern uint8_t g_csq_val;

#endif /* __GSM_QUECTEL_H */