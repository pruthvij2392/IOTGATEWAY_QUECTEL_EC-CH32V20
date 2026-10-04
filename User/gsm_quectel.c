/********************************** (C) COPYRIGHT *******************************
 * File Name          : gsm_quectel.c
 * Description        : Quectel 4G Module (EG800G/EC200U) Driver with MQTT for CH32V203
 *                      - Pins: IOT_EN = PB4, 4G_PWR = PB3, TX = PA2, RX = PA3
 *                      - UART: USART2 @ 115200 baud
 *                      - MQTT Broker: mqtt.swasemi.in:1883
 *                      - Topic: IOTGATEWAY
 *******************************************************************************/
#include "gsm_quectel.h"
#include "main.h"
#include "comman.h"
#include "debug.h"
#include "ds3231.h"
#include "modbus_rtu.h"
#include <string.h>
#include <stdio.h>

#define GSM_DEBUG_PRINT 0
#if GSM_DEBUG_PRINT
  #define printf_gsm(...) printf(__VA_ARGS__)
#else
  #define printf_gsm(...) ((void)0)
#endif

/* ==================== Global & State Variables ==================== */
volatile GSM_State_t g_gsm_state          = GSM_STATE_INIT;
volatile uint8_t     g_gsm_response_ready = 0;
uint8_t              g_gsm_rx_buffer[GSM_RX_BUFFER_SIZE];
uint16_t             g_gsm_rx_len         = 0;

SensorData_t g_sensor_data;
GSM_Time_t   g_gsm_time;
uint8_t      g_csq_val           = 24;
static uint8_t g_trigger_publish = 0;

/* ==================== Helper Functions ==================== */
static void ClearRXBuffer(void) {
    memset((void*)g_gsm_rx_buffer, 0, sizeof(g_gsm_rx_buffer));
    g_gsm_rx_len         = 0;
    g_gsm_response_ready = 0;
}

static uint8_t CheckResponse(const char *expected) {
    if(strstr((const char*)g_gsm_rx_buffer, expected) != NULL) {
        return 1;
    }
    return 0;
}

/* ==================== USART2 Interrupt Handler ==================== */
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) {
    if(USART_GetITStatus(GSM_UART, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART_ReceiveData(GSM_UART);
        if(g_gsm_rx_len < GSM_RX_BUFFER_SIZE - 1) {
            g_gsm_rx_buffer[g_gsm_rx_len++] = (char)c;
            g_gsm_rx_buffer[g_gsm_rx_len]   = '\0';
        } else if(g_gsm_state == GSM_STATE_STANDBY) {
            /* If in standby and buffer is full, slide left to preserve incoming URCs */
            memmove((void*)&g_gsm_rx_buffer[0], (void*)&g_gsm_rx_buffer[128], (GSM_RX_BUFFER_SIZE - 128));
            g_gsm_rx_len -= 128;
            g_gsm_rx_buffer[g_gsm_rx_len++] = (char)c;
            g_gsm_rx_buffer[g_gsm_rx_len]   = '\0';
        }
        USART_ClearITPendingBit(GSM_UART, USART_IT_RXNE);
    }
}

/* ==================== Hardware Initialization ==================== */
void GSM_Pins_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* Enable GPIOB and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* CRITICAL FOR CH32V203: PB3 is JTDO, PB4 is NJTRST (JTAG pins by default).
     * Must disable SWJ/JTAG so PB3 (4G_PWR) and PB4 (IOT_EN) work as normal GPIOs! */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    /* PB4 = IOT_EN (Power Enable), PB3 = 4G_PWR (PWRKEY) */
    GPIO_InitStructure.GPIO_Pin   = IOT_EN_PIN | GSM_PWR_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(IOT_EN_PORT, IOT_EN_PIN);
    GPIO_ResetBits(GSM_PWR_PORT, GSM_PWR_PIN);
}

void GSM_PowerOn(void) {
    printf_gsm("[QUECTEL] Hardware power on: Setting IOT_EN (PB4) HIGH...\r\n");
    /* 1. Turn on 4G Module Power Supply (PB4 = HIGH) */
    GPIO_SetBits(IOT_EN_PORT, IOT_EN_PIN);
    Delay_Ms(500);

    /* 2. Pulse PWRKEY (PB3 = HIGH for 1.5s, then LOW) to turn ON baseband */
    printf_gsm("[QUECTEL] Pulsing PWRKEY (PB3) for 1.5s...\r\n");
    GPIO_SetBits(GSM_PWR_PORT, GSM_PWR_PIN);
    Delay_Ms(1500);
    GPIO_ResetBits(GSM_PWR_PORT, GSM_PWR_PIN);

    /* 3. Wait 4 seconds for Quectel module firmware to boot up */
    printf_gsm("[QUECTEL] Waiting 4s for Quectel baseband boot...\r\n");
    Delay_Ms(4000);
}

void GSM_Init(void) {
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure  = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA2 - USART2_TX (AF_PP) */
    GPIO_InitStructure.GPIO_Pin   = GSM_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GSM_TX_PORT, &GPIO_InitStructure);

    /* PA3 - USART2_RX (Floating Input) */
    GPIO_InitStructure.GPIO_Pin  = GSM_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GSM_RX_PORT, &GPIO_InitStructure);

    /* USART2 Configuration: 115200 8N1 */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(GSM_UART, &USART_InitStructure);

    /* Enable RX Interrupt */
    USART_ITConfig(GSM_UART, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(GSM_UART, ENABLE);

    ClearRXBuffer();
}

/* ==================== AT Command Communication ==================== */
void GSM_SendCommand(const char *cmd) {
    ClearRXBuffer();

    /* Send command string */
    while(*cmd) {
        USART_SendData(GSM_UART, *cmd++);
        while(USART_GetFlagStatus(GSM_UART, USART_FLAG_TXE) == RESET);
    }

    /* Send CRLF (\r\n) terminator */
    USART_SendData(GSM_UART, '\r');
    while(USART_GetFlagStatus(GSM_UART, USART_FLAG_TXE) == RESET);
    USART_SendData(GSM_UART, '\n');
    while(USART_GetFlagStatus(GSM_UART, USART_FLAG_TXE) == RESET);
}

void GSM_SendData(const uint8_t *data, uint16_t len) {
    for(uint16_t i = 0; i < len; i++) {
        USART_SendData(GSM_UART, data[i]);
        while(USART_GetFlagStatus(GSM_UART, USART_FLAG_TXE) == RESET);
    }
}

void GSM_WaitForResponse(const char *expected, uint16_t timeout_ms) {
    uint32_t start = GetTick();
    while((GetTick() - start) < timeout_ms) {
        if(CheckResponse(expected) || CheckResponse("ERROR")) {
            break;
        }
        Delay_Ms(10);
    }
}

uint8_t GSM_CheckResponse(const char *expected) {
    return CheckResponse(expected);
}

/* ==================== Sync RTC with Network Time ==================== */
uint8_t GSM_SyncNetworkTime(void) {
    /* 1. Try AT+QLTS=2 */
    GSM_SendCommand("AT+QLTS=2");
    GSM_WaitForResponse("+QLTS:", 1200);
    if(CheckResponse("+QLTS:")) {
        if(RTC_WrtFromStr((char*)g_gsm_rx_buffer)) {
            return 1;
        }
    }

    /* 2. Fallback to AT+CCLK? */
    GSM_SendCommand("AT+CCLK?");
    GSM_WaitForResponse("+CCLK:", 1200);
    if(CheckResponse("+CCLK:")) {
        if(RTC_WrtFromStr((char*)g_gsm_rx_buffer)) {
            return 1;
        }
    }

    return 0;
}

/* ==================== Parse CSQ (Signal Strength) ==================== */
void ParseCSQ(char *response) {
    char *p = strstr(response, "+CSQ: ");
    if(p) {
        p += 6;
        while(*p == ' ') p++;
        int csq = atoi(p);
        if(csq > 0 && csq != 99) {
            g_csq_val = (uint8_t)csq;
            iot_send.Signal_Strength = g_csq_val;
        }
    }
}

/* ==================== GPS NMEA Data Conversion & Parser ==================== */
void GPS_DATAConversion(char *GpsStr, uint8_t CalLen, uint8_t CalOpr) {
    uint8_t dot = CalLen;
    uint32_t pre = 0;
    uint32_t frac = 0;
    uint32_t dd = 0, mm = 0;
    uint32_t scale = 1;

    for (uint8_t i = 0; i < CalLen; i++) {
        char c = GpsStr[i];
        if (c == '.') {
            dot = i;
            break;
        }
        pre = pre * 10u + (uint32_t)(c - '0');
    }

    if (dot < CalLen) {
        for (uint8_t i = (dot + 1); i < CalLen; i++) {
            char c = GpsStr[i];
            if (c < '0' || c > '9') break;
            frac = frac * 10u + (uint32_t)(c - '0');
            scale *= 10u;
        }
    }

    switch (CalOpr) {
        case 1: /* Latitude */
            dd = pre / 100;
            mm = pre % 100;
            iot_send.latitude = (float)((float)frac / (float)scale);
            iot_send.latitude += (float)mm;
            iot_send.latitude /= (float)60.0;
            iot_send.latitude += (float)dd;
            break;

        case 2: /* Longitude */
            dd = pre / 100;
            mm = pre % 100;
            iot_send.longitude = (float)((float)frac / (float)scale);
            iot_send.longitude += (float)mm;
            iot_send.longitude /= (float)60.0;
            iot_send.longitude += (float)dd;
            break;
    }
}

void ProcessGPSFrame(const char *response) {
    const char *rmc = strstr(response, "$GNRMC");
    if(!rmc) rmc = strstr(response, "$GPRMC");
    if(!rmc) rmc = strstr(response, "$GQRMC");
    if(!rmc) rmc = strstr(response, "$GBRMC");

    if(!rmc) {
        iot_send.latitude = 0.0f;
        iot_send.longitude = 0.0f;
        return;
    }

    char hold[128];
    strncpy(hold, rmc, sizeof(hold) - 1);
    hold[sizeof(hold) - 1] = '\0';

    char *token = strtok(hold, ",");
    int comma_idx = 0;
    char status = 'V';
    char lat_str[32] = {0};
    char ns_ind = 'N';
    char lon_str[32] = {0};
    char ew_ind = 'E';

    while(token != NULL && comma_idx <= 6) {
        if(comma_idx == 2 && strlen(token) > 0) {
            status = token[0];
        } else if(comma_idx == 3) {
            strncpy(lat_str, token, sizeof(lat_str) - 1);
        } else if(comma_idx == 4 && strlen(token) > 0) {
            ns_ind = token[0];
        } else if(comma_idx == 5) {
            strncpy(lon_str, token, sizeof(lon_str) - 1);
        } else if(comma_idx == 6 && strlen(token) > 0) {
            ew_ind = token[0];
        }
        token = strtok(NULL, ",");
        comma_idx++;
    }

    if(status == 'A' && strlen(lat_str) >= 4 && strlen(lon_str) >= 5) {
        GPS_DATAConversion(lat_str, (uint8_t)strlen(lat_str), 1);
        if(ns_ind == 'S') iot_send.latitude = -iot_send.latitude;

        GPS_DATAConversion(lon_str, (uint8_t)strlen(lon_str), 2);
        if(ew_ind == 'W') iot_send.longitude = -iot_send.longitude;

        printf_gsm("[GPS] Fix acquired: Lat=%.6f, Lon=%.6f\r\n", iot_send.latitude, iot_send.longitude);
    } else {
        iot_send.latitude = 0.0f;
        iot_send.longitude = 0.0f;
        printf_gsm("[GPS] No Fix (status='%c') -> coordinates 0.00\r\n", status);
    }
}

/* ==================== Parse Network Time (+CCLK / +QLTS) ==================== */
uint8_t ParseCCLKTime(char *response, GSM_Time_t *time) {
    return RTC_WrtFromStr(response);
}

/* ==================== JSON Reading Frame Formatter ==================== */
void READING_FRAME(char *out_frame, uint16_t max_len) {
    RTC_TimeTypeDef rtc;

    /* Get live RTC time from DS3231 */
    if (!DS3231_GetTime(&rtc) || rtc.date < 1 || rtc.date > 31 || rtc.month < 1 || rtc.month > 12) {
        rtc.date   = (g_gsm_time.date   > 0) ? g_gsm_time.date   : 24;
        rtc.month  = (g_gsm_time.month  > 0) ? g_gsm_time.month  : 9;
        rtc.year   = (g_gsm_time.year   > 0) ? g_gsm_time.year   : 26;
        rtc.hour   = g_gsm_time.hour;
        rtc.minute = g_gsm_time.minute;
        rtc.second = g_gsm_time.second;
    }

    /* Format Latitude & Longitude (0 if not available) */
    char lat_buf[24] = "0";
    char lon_buf[24] = "0";

    if (iot_send.latitude != 0.0f) {
        int lat_w = (int)iot_send.latitude;
        int lat_f = (int)((iot_send.latitude - (float)lat_w) * 1000000.0f);
        if (lat_f < 0) lat_f = -lat_f;
        sprintf(lat_buf, "%d.%06d", lat_w, lat_f);
    }
    if (iot_send.longitude != 0.0f) {
        int lon_w = (int)iot_send.longitude;
        int lon_f = (int)((iot_send.longitude - (float)lon_w) * 1000000.0f);
        if (lon_f < 0) lon_f = -lon_f;
        sprintf(lon_buf, "%d.%06d", lon_w, lon_f);
    }

    /* Signal percentage: 0 to 100% */
    uint16_t csq_pct = (g_csq_val <= 31) ? (((uint32_t)g_csq_val * 100UL) / 31UL) : 0;

    /* Build JSON Telemetry Frame: ts, la, lo, xg, xv, d: { S1..S16 } */
    int pos = snprintf(out_frame, max_len,
             "{\n"
             "  \"ts\": \"20%02d-%02d-%02dT%02d:%02d:%02d\",\n"
             "  \"la\": %s,\n"
             "  \"lo\": %s,\n"
             "  \"xg\": %u,\n"
             "  \"xv\": 12.0,\n"
             "  \"d\": {\n",
             rtc.year, rtc.month, rtc.date,
             rtc.hour, rtc.minute, rtc.second,
             lat_buf,
             lon_buf,
             (unsigned int)csq_pct);

    if (pos < 0 || pos >= (int)max_len) pos = (int)max_len - 1;

    /* Output all 16 channels: S1 to S16 */
    uint32_t now_tick = GetTick();
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        char val_buf[16] = "0";
        const Modbus_Sensor_Runtime_t *rt = &g_sensors_runtime[i];

        /* If sensor is enabled, online, and received response recently */
        if (first.sensors[i].enabled && rt->sensor_online && 
            (rt->last_response_tick > 0 && (now_tick - rt->last_response_tick) < 15000) &&
            rt->val_str[0] != '\0' && strcmp(rt->val_str, "0") != 0 && strcmp(rt->val_str, "0.00") != 0) {
            strncpy(val_buf, rt->val_str, sizeof(val_buf) - 1);
        } else {
            strcpy(val_buf, "0");
        }

        val_buf[sizeof(val_buf) - 1] = '\0';
        if (pos < (int)(max_len - 1)) {
            int w = snprintf(out_frame + pos, max_len - (size_t)pos,
                            "    \"S%u\": %s%s\n",
                            (unsigned int)(i + 1),
                            val_buf,
                            (i < MAX_MODBUS_SENSORS - 1) ? "," : "");
            if (w > 0) {
                pos += w;
                if (pos >= (int)max_len) pos = (int)max_len - 1;
            }
        }
    }

    if (pos < (int)(max_len - 1)) {
        int w = snprintf(out_frame + pos, max_len - (size_t)pos, "  }\n}");
        if (w > 0) pos += w;
    }
}

/* ==================== CSV Reading Frame Formatter (For .CSV / Excel Logging) ==================== */
void GENERATE_CSV_FRAME(char *out_csv, uint16_t max_len, uint32_t record_num) {
    RTC_TimeTypeDef rtc;

    if (!DS3231_GetTime(&rtc) || rtc.date < 1 || rtc.date > 31 || rtc.month < 1 || rtc.month > 12) {
        rtc.date   = (g_gsm_time.date   > 0) ? g_gsm_time.date   : 26;
        rtc.month  = (g_gsm_time.month  > 0) ? g_gsm_time.month  : 9;
        rtc.year   = (g_gsm_time.year   > 0) ? g_gsm_time.year   : 26;
        rtc.hour   = g_gsm_time.hour;
        rtc.minute = g_gsm_time.minute;
        rtc.second = g_gsm_time.second;
    }

    char lat_buf[24] = "0.000000";
    char lon_buf[24] = "0.000000";

    if (iot_send.latitude != 0.0f) {
        int lat_w = (int)iot_send.latitude;
        int lat_f = (int)((iot_send.latitude - (float)lat_w) * 1000000.0f);
        if (lat_f < 0) lat_f = -lat_f;
        sprintf(lat_buf, "%d.%06d", lat_w, lat_f);
    }
    if (iot_send.longitude != 0.0f) {
        int lon_w = (int)iot_send.longitude;
        int lon_f = (int)((iot_send.longitude - (float)lon_w) * 1000000.0f);
        if (lon_f < 0) lon_f = -lon_f;
        sprintf(lon_buf, "%d.%06d", lon_w, lon_f);
    }

    /* Single-line CSV format: Sr No,DATE,TIME,LAT,LONG,s1,s2,...s16,SIGNAL STRENGTH */
    int pos = snprintf(out_csv, max_len,
                       "%u,%02d-%02d-20%02d,%02d:%02d:%02d,%s,%s",
                       (unsigned int)record_num,
                       rtc.date, rtc.month, rtc.year,
                       rtc.hour, rtc.minute, rtc.second,
                       lat_buf,
                       lon_buf);

    if (pos < 0 || pos >= (int)max_len) pos = (int)max_len - 1;

    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        const Modbus_Sensor_Runtime_t *rt = &g_sensors_runtime[i];
        const char *val = (rt->sensor_online && rt->val_str[0] != '\0' && strcmp(rt->val_str, "0.00") != 0 && strcmp(rt->val_str, "0") != 0) ? rt->val_str : "0";
        if (pos < (int)(max_len - 1)) {
            int w = snprintf(out_csv + pos, max_len - (size_t)pos, ",%s", val);
            if (w > 0) {
                pos += w;
                if (pos >= (int)max_len) pos = (int)max_len - 1;
            }
        }
    }

    if (pos < (int)(max_len - 1)) {
        snprintf(out_csv + pos, max_len - (size_t)pos, ",%u", (unsigned int)g_csq_val);
    }
}

/* ==================== MQTT Publish Helper ==================== */
static uint8_t GSM_PublishMqttTopicPayload(const char *topic, const char *payload) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,0,0,0,\"%s\"", topic);
    GSM_SendCommand(cmd);

    uint32_t start = GetTick();
    uint8_t got_prompt = 0;
    while((GetTick() - start) < 2000) {
        IWDG_ReloadCounter();
        if(strstr((const char*)g_gsm_rx_buffer, ">") != NULL) {
            got_prompt = 1;
            break;
        }
        Delay_Ms(5);
    }

    if(got_prompt) {
        GSM_SendData((const uint8_t*)payload, strlen(payload));
        USART_SendData(GSM_UART, 0x1A);
        while(USART_GetFlagStatus(GSM_UART, USART_FLAG_TXE) == RESET);
        GSM_WaitForResponse("+QMTPUB: 0,0,0", 3000);
        ClearRXBuffer();
        return 1;
    }
    printf("[MQTT] ERR: No prompt for '%s'\r\n", topic);
    return 0;
}

/* ==================== MQTT Publish Sensor Data ==================== */
void GSM_PublishSensorData(void) {
    static char payload[768];
    static char csv_record[256];
    static char pub_topic[96];

    /* 1. Generate clean valid JSON Telemetry Frame for MQTT Broker */
    READING_FRAME(payload, sizeof(payload));

    /* 2. Generate Single-Line CSV Record and save to Flash */
    GENERATE_CSV_FRAME(csv_record, sizeof(csv_record), g_total_logged_frames + 1);
    SaveFrameToFlash(csv_record);

    const char *dev_name = (first.Device_ID[0] != '\0') ? first.Device_ID : 
                           ((first.Serial_no[0] != '\0') ? first.Serial_no : "DLH26080011");
    snprintf(pub_topic, sizeof(pub_topic), "SWA/%s/READING", dev_name);

    printf_gsm("[QUECTEL] Publishing Payload to '%s':\r\n%s\r\n", pub_topic, payload);
    GSM_PublishMqttTopicPayload(pub_topic, payload);
}

/* ==================== MQTT Publish Config Request ==================== */
void GSM_PublishConfigRequest(void) {
    static char req_topic[96];
    static char payload[128];
    const char *dev_id = (first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001";

    snprintf(req_topic, sizeof(req_topic), "SWA/%s/CONFIG/REQ", dev_id);
    snprintf(payload, sizeof(payload), "{\n  \"ver\": \"2.0.0\",\n  \"act\": \"get-config\"\n}");

    printf_gsm("[QUECTEL] Publishing Config Request to %s\r\n", req_topic);
    GSM_PublishMqttTopicPayload(req_topic, payload);
}

/* ==================== MQTT Publish Config Response ==================== */
void GSM_PublishConfigResponseForId(const char *target_id) {
    static char res_topic[96];
    static char payload[384];
    const char *target_sn = (first.Serial_no[0] != '\0') ? first.Serial_no : 
                            ((target_id && target_id[0] != '\0') ? target_id : "GTW26090001");
    uint16_t sample_sec   = (first.Sample_Time > 0) ? first.Sample_Time : 60;

    snprintf(res_topic, sizeof(res_topic), "SWA/%s/CONFIG/RES", target_sn);

    snprintf(payload, sizeof(payload),
             "{\n"
             "  \"st\":  \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"act\": \"set-config\",\n"
             "  \"d\": {\n"
             "    \"sn\": \"%s\",\n"
             "    \"dn\": \"%s\",\n"
             "    \"spt\": %u,\n"
             "    \"bip\": \"%s\",\n"
             "    \"bpt\": %u,\n"
             "    \"usr\": \"%s\",\n"
             "    \"mul\": %u\n"
             "  }\n"
             "}",
             target_sn,
             (first.Device_ID[0] != '\0') ? first.Device_ID : target_sn,
             (unsigned int)sample_sec,
             (first.MQTTServer[0] != '\0') ? first.MQTTServer : "mqtt.swasemi.in",
             (unsigned int)((first.MQTTPort > 0) ? first.MQTTPort : 1883),
             (first.MQTTUsername[0] != '\0') ? first.MQTTUsername : "mqtt_swasemi",
             (unsigned int)first.mul_factor);

    printf("[MQTT] Publishing Config Response to '%s'...\r\n", res_topic);
    GSM_PublishMqttTopicPayload(res_topic, payload);
}

void GSM_PublishConfigResponse(void) {
    GSM_PublishConfigResponseForId(NULL);
}

/* ==================== MQTT Handle Set General Config ==================== */
void GSM_HandleSetConfig(const char *json_payload, const char *target_id) {
    printf("\r\n[MQTT] Parsing 'set-config'...\r\n");
    Config_ApplyGeneralJson(json_payload);
    const char *target_sn = (first.Serial_no[0] != '\0') ? first.Serial_no : 
                            ((target_id && target_id[0] != '\0') ? target_id : "GTW26090001");
    GSM_PublishConfigResponseForId(target_sn);
}

/* ==================== MQTT Handle Remote RESET_EEPROM (Clear Data Logs) ==================== */
void GSM_HandleRemoteResetEEPROM(const char *target_id) {
    static char res_topic[96];
    static char payload[192];
    const char *target_sn = (target_id && target_id[0] != '\0') ? target_id : 
                            ((first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001");

    printf("\r\n[MQTT] <<< Executing REMOTE RESET_EEPROM (Clear Flash Data Logs) for Device: %s >>>\r\n", target_sn);

    /* 1. Clear & Erase all logged data frames from SPI Flash */
    Reset_EEPROM();

    /* 2. Build Success Response */
    snprintf(res_topic, sizeof(res_topic), "SWA/%s/CONFIG/RES", target_sn);
    snprintf(payload, sizeof(payload),
             "{\n"
             "  \"st\": \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"sn\": \"%s\",\n"
             "  \"act\": \"reset-eeprom\"\n"
             "}",
             target_sn);

    /* 3. Publish Success Response to MQTT Broker */
    printf("[MQTT] Publishing Reset-EEPROM Success Ack to '%s'...\r\n", res_topic);
    GSM_PublishMqttTopicPayload(res_topic, payload);
}

/* ==================== MQTT Handle Remote RESET_CONFIG (Factory Reset Settings) ==================== */
void GSM_HandleRemoteResetConfig(const char *target_id) {
    static char res_topic[96];
    static char payload[192];
    const char *target_sn = (target_id && target_id[0] != '\0') ? target_id : 
                            ((first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001");

    printf("\r\n[MQTT] <<< Executing REMOTE RESET_CONFIG (Factory Defaults) for Device: %s >>>\r\n", target_sn);

    /* 1. Reset all configuration settings to factory default and save to NVM */
    NVM_ResetToDefaults();

    /* 2. Build Success Response */
    snprintf(res_topic, sizeof(res_topic), "SWA/%s/CONFIG/RES", target_sn);
    snprintf(payload, sizeof(payload),
             "{\n"
             "  \"st\": \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"sn\": \"%s\",\n"
             "  \"act\": \"reset-config\"\n"
             "}",
             target_sn);

    /* 3. Publish Success Response to MQTT Broker */
    printf("[MQTT] Publishing Reset-Config Success Ack to '%s'...\r\n", res_topic);
    GSM_PublishMqttTopicPayload(res_topic, payload);
}

/* ==================== MQTT Handle Remote Device Reboot ==================== */
void GSM_HandleRemoteReset(const char *target_id) {
    static char res_topic[96];
    static char payload[192];
    const char *target_sn = (target_id && target_id[0] != '\0') ? target_id : 
                            ((first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001");

    printf("\r\n[MQTT] <<< Executing REMOTE SYSTEM REBOOT for Device: %s >>>\r\n", target_sn);

    /* 1. Build Success Response */
    snprintf(res_topic, sizeof(res_topic), "SWA/%s/CONFIG/RES", target_sn);
    snprintf(payload, sizeof(payload),
             "{\n"
             "  \"st\": \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"sn\": \"%s\",\n"
             "  \"act\": \"reboot\"\n"
             "}",
             target_sn);

    /* 2. Publish Success Response to MQTT Broker */
    printf("[MQTT] Publishing Reboot Success Ack to '%s'...\r\n", res_topic);
    GSM_PublishMqttTopicPayload(res_topic, payload);

    /* 3. Small delay to flush UART buffers and trigger system reboot */
    printf("[SYSTEM] Rebooting MCU...\r\n");
    Delay_Ms(500);
    NVIC_SystemReset();
}

/* ==================== MQTT Handle Set Date / Time ==================== */
void GSM_HandleSetTime(const char *json_payload, const char *target_id) {
    static char res_topic[96];
    static char payload[256];
    const char *target_sn = (target_id && target_id[0] != '\0') ? target_id : 
                            ((first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001");

    printf("\r\n[MQTT] <<< Processing Date/Time Config for Device: %s >>>\r\n", target_sn);

    /* Parse and apply date/time to DS3231 RTC */
    Config_ApplyTimeJson(json_payload);

    /* Read updated live RTC time */
    RTC_TimeTypeDef rtc;
    if (!DS3231_GetTime(&rtc) || rtc.date < 1 || rtc.month < 1 || rtc.year < 20) {
        rtc.year = 26; rtc.month = 9; rtc.date = 29; rtc.hour = 12; rtc.minute = 0; rtc.second = 0;
    }

    /* Build Response */
    snprintf(res_topic, sizeof(res_topic), "SWA/%s/CONFIG/RES", target_sn);
    snprintf(payload, sizeof(payload),
             "{\n"
             "  \"st\": \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"sn\": \"%s\",\n"
             "  \"act\": \"set-time\",\n"
             "  \"d\": {\n"
             "    \"dt\": \"%02d/%02d/%02d\",\n"
             "    \"tm\": \"%02d:%02d:%02d\"\n"
             "  }\n"
             "}",
             target_sn,
             rtc.date, rtc.month, rtc.year,
             rtc.hour, rtc.minute, rtc.second);

    printf("[MQTT] Publishing Time Config Response to '%s'...\r\n", res_topic);
    GSM_PublishMqttTopicPayload(res_topic, payload);
}

/* ==================== MQTT Publish Get Time Response ==================== */
void GSM_PublishGetTimeResponse(const char *target_id) {
    static char res_topic[96];
    static char payload[256];
    const char *target_sn = (target_id && target_id[0] != '\0') ? target_id : 
                            ((first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001");

    RTC_TimeTypeDef rtc;
    if (!DS3231_GetTime(&rtc) || rtc.date < 1 || rtc.month < 1 || rtc.year < 20) {
        rtc.year = 26; rtc.month = 9; rtc.date = 29; rtc.hour = 12; rtc.minute = 0; rtc.second = 0;
    }

    snprintf(res_topic, sizeof(res_topic), "SWA/%s/CONFIG/RES", target_sn);
    snprintf(payload, sizeof(payload),
             "{\n"
             "  \"st\": \"success\",\n"
             "  \"ver\": \"2.0.0\",\n"
             "  \"sn\": \"%s\",\n"
             "  \"act\": \"get-time\",\n"
             "  \"d\": {\n"
             "    \"dt\": \"%02d/%02d/%02d\",\n"
             "    \"tm\": \"%02d:%02d:%02d\"\n"
             "  }\n"
             "}",
             target_sn,
             rtc.date, rtc.month, rtc.year,
             rtc.hour, rtc.minute, rtc.second);

    printf("[MQTT] Publishing Get Time Response to '%s'...\r\n", res_topic);
    GSM_PublishMqttTopicPayload(res_topic, payload);
}

/* ==================== Check for Incoming MQTT Messages ==================== */
void GSM_CheckIncomingMessages(void) {
    static char rx_copy[GSM_RX_BUFFER_SIZE];

    if(g_gsm_rx_len > 0) {
        char *p_set_config   = strstr((char*)g_gsm_rx_buffer, "set-config");
        char *p_set_time     = strstr((char*)g_gsm_rx_buffer, "set-time");
        if(!p_set_time) p_set_time = strstr((char*)g_gsm_rx_buffer, "set-datetime");
        if(!p_set_time) p_set_time = strstr((char*)g_gsm_rx_buffer, "set-date");
        char *p_get_time     = strstr((char*)g_gsm_rx_buffer, "get-time");
        if(!p_get_time) p_get_time = strstr((char*)g_gsm_rx_buffer, "get-datetime");

        char *p_reset_eeprom = strstr((char*)g_gsm_rx_buffer, "reset-eeprom");
        if(!p_reset_eeprom) p_reset_eeprom = strstr((char*)g_gsm_rx_buffer, "reset_eeprom");
        if(!p_reset_eeprom) p_reset_eeprom = strstr((char*)g_gsm_rx_buffer, "reset-data");
        if(!p_reset_eeprom) p_reset_eeprom = strstr((char*)g_gsm_rx_buffer, "clear-logs");

        char *p_reset_config = strstr((char*)g_gsm_rx_buffer, "reset-config");
        if(!p_reset_config) p_reset_config = strstr((char*)g_gsm_rx_buffer, "reset_config");
        if(!p_reset_config) p_reset_config = strstr((char*)g_gsm_rx_buffer, "factory-reset");

        char *p_reset        = NULL;
        if(!p_reset_eeprom && !p_reset_config) {
            p_reset = strstr((char*)g_gsm_rx_buffer, "\"reset\"");
            if(!p_reset) p_reset = strstr((char*)g_gsm_rx_buffer, "\"reboot\"");
        }

        char *p_req          = strstr((char*)g_gsm_rx_buffer, "get-config");
        char *p_urc          = strstr((char*)g_gsm_rx_buffer, "+QMTRECV:");
        
        if(p_reset_eeprom != NULL || p_reset_config != NULL || p_reset != NULL || p_set_time != NULL || p_get_time != NULL || p_set_config != NULL || p_req != NULL || 
           (p_urc != NULL && strstr((char*)g_gsm_rx_buffer, "CONFIG/REQ") != NULL)) {
            char target_id[48] = {0};
            
            /* Extract device ID / Serial from topic: SWA/<target_id>/CONFIG/REQ */
            char *p_swa = strstr((char*)g_gsm_rx_buffer, "SWA/");
            if(p_swa != NULL) {
                p_swa += 4; /* skip "SWA/" */
                char *p_slash = strchr(p_swa, '/');
                if(p_slash != NULL && (p_slash - p_swa) < (int)sizeof(target_id)) {
                    int id_len = (int)(p_slash - p_swa);
                    strncpy(target_id, p_swa, id_len);
                    target_id[id_len] = '\0';
                }
            }
            if (target_id[0] == '\0') {
                strncpy(target_id, (first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001", sizeof(target_id) - 1);
            }

            /* Accept for this gateway (matches Serial, Device ID, GTW prefix or wildcard) */
            uint8_t is_for_me = 0;
            if (strcasecmp(target_id, first.Serial_no) == 0 ||
                strcasecmp(target_id, first.Device_ID) == 0 ||
                strcasecmp(target_id, "GTW26090001") == 0 ||
                strcasecmp(target_id, "ALL") == 0 ||
                target_id[0] == '+') {
                is_for_me = 1;
            } else {
                /* Check if JSON body specifies "sn" matching our serial */
                const char *p_sn = strstr((char*)g_gsm_rx_buffer, "\"sn\"");
                if (p_sn) {
                    p_sn = strchr(p_sn, ':');
                    if (p_sn) {
                        while (*p_sn == ' ' || *p_sn == '\"') p_sn++;
                        if (strncasecmp(p_sn, first.Serial_no, strlen(first.Serial_no)) == 0 ||
                            strncasecmp(p_sn, "GTW26090001", 11) == 0) {
                            is_for_me = 1;
                        }
                    }
                }
            }

            if (!is_for_me) {
                printf("[MQTT] Ignored command for other device '%s' (My SN: %s)\r\n", target_id, first.Serial_no);
                ClearRXBuffer();
                return;
            }

            if(p_reset_eeprom != NULL) {
                printf("\r\n[MQTT] <<< Processing 'reset-eeprom' for Device: %s >>>\r\n", target_id);
                ClearRXBuffer();
                GSM_HandleRemoteResetEEPROM(target_id);
            }
            else if(p_reset_config != NULL) {
                printf("\r\n[MQTT] <<< Processing 'reset-config' for Device: %s >>>\r\n", target_id);
                ClearRXBuffer();
                GSM_HandleRemoteResetConfig(target_id);
            }
            else if(p_reset != NULL) {
                printf("\r\n[MQTT] <<< Processing 'reboot' for Device: %s >>>\r\n", target_id);
                ClearRXBuffer();
                GSM_HandleRemoteReset(target_id);
            }
            else if(p_set_time != NULL) {
                printf("\r\n[MQTT] <<< Processing 'set-time' for Device: %s >>>\r\n", target_id);
                strncpy(rx_copy, (char*)g_gsm_rx_buffer, sizeof(rx_copy) - 1);
                rx_copy[sizeof(rx_copy) - 1] = '\0';
                ClearRXBuffer();
                GSM_HandleSetTime(rx_copy, target_id);
            }
            else if(p_get_time != NULL) {
                printf("\r\n[MQTT] <<< Processing 'get-time' for Device: %s >>>\r\n", target_id);
                ClearRXBuffer();
                GSM_PublishGetTimeResponse(target_id);
            }
            else if(p_set_config != NULL) {
                printf("\r\n[MQTT] <<< Processing 'set-config' for Device: %s >>>\r\n", target_id);
                strncpy(rx_copy, (char*)g_gsm_rx_buffer, sizeof(rx_copy) - 1);
                rx_copy[sizeof(rx_copy) - 1] = '\0';
                ClearRXBuffer();
                GSM_HandleSetConfig(rx_copy, target_id);
            }
            else if(p_req != NULL) {
                printf("\r\n[MQTT] <<< Processing 'get-config' for Device: %s >>>\r\n", target_id);
                ClearRXBuffer();
                GSM_PublishConfigResponseForId(target_id);
            }
        }
    }
}

uint8_t GSM_IsConnected(void) {
    return (g_gsm_state == GSM_STATE_QMTSUB || g_gsm_state == GSM_STATE_STANDBY || g_gsm_state == GSM_STATE_PUBLISH || g_gsm_state == GSM_STATE_PUBLISH_WAIT);
}

uint8_t GSM_IsNetworkRegistered(void) {
    return (g_gsm_state >= GSM_STATE_GPS_INIT && g_gsm_state != GSM_STATE_ERROR && g_csq_val > 0 && g_csq_val != 99);
}

uint8_t GSM_IsGPSFixed(void) {
    return (iot_send.latitude != 0.0f || iot_send.longitude != 0.0f);
}

void GSM_TriggerPublish(void) {
    g_trigger_publish = 1;
}

/* ==================== GSM State Machine (Non-Blocking & Watchdog Protected) ==================== */
void GSM_ProcessState(void) {
    static char cmd_buffer[128];
    static uint8_t retry_count = 0;
    static uint8_t connect_retry = 0;
    static uint8_t s_sub_state = 0;
    static uint32_t s_state_tick = 0;
    static uint8_t s_consec_errors = 0;

    IWDG_ReloadCounter(); /* Always keep hardware watchdog fed */

    switch(g_gsm_state) {

        case GSM_STATE_INIT:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: INIT - Sending AT...\r\n");
                GSM_SendCommand("AT");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("OK")) {
                    printf_gsm("[QUECTEL] AT OK -> moving to ATE0\r\n");
                    g_gsm_state = GSM_STATE_ATE0;
                    s_sub_state = 0;
                    retry_count = 0;
                } else if((GetTick() - s_state_tick) >= 1500) {
                    if(++retry_count >= 5) {
                        printf_gsm("[QUECTEL] INIT FAILED after 5 retries -> ERROR\r\n");
                        g_gsm_state = GSM_STATE_ERROR;
                        s_sub_state = 0;
                    } else {
                        s_sub_state = 0; /* Retry */
                    }
                }
            }
            break;

        case GSM_STATE_ATE0:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: ATE0 - Disabling echo...\r\n");
                GSM_SendCommand("ATE0");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("OK") || (GetTick() - s_state_tick) >= 800) {
                    printf_gsm("[QUECTEL] ATE0 OK -> SIM_CHECK\r\n");
                    g_gsm_state = GSM_STATE_SIM_CHECK;
                    s_sub_state = 0;
                    retry_count = 0;
                }
            }
            break;

        case GSM_STATE_SIM_CHECK:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: SIM_CHECK - AT+CPIN?...\r\n");
                GSM_SendCommand("AT+CPIN?");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("READY")) {
                    printf_gsm("[QUECTEL] SIM READY -> NETWORK_REG\r\n");
                    g_gsm_state = GSM_STATE_NETWORK_REG;
                    s_sub_state = 0;
                    retry_count = 0;
                } else if((GetTick() - s_state_tick) >= 2000) {
                    if(++retry_count >= 5) {
                        printf_gsm("[QUECTEL] SIM NOT READY -> ERROR\r\n");
                        g_gsm_state = GSM_STATE_ERROR;
                        s_sub_state = 0;
                    } else {
                        s_sub_state = 0; /* Retry */
                    }
                }
            }
            break;

        case GSM_STATE_NETWORK_REG:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: NETWORK_REG - Checking AT+CEREG?...\r\n");
                GSM_SendCommand("AT+CEREG?");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("+CEREG: 0,1") || CheckResponse("+CEREG: 0,5") ||
                   CheckResponse("+CEREG: 1,1") || CheckResponse("+CEREG: 1,5") ||
                   CheckResponse("+CREG: 0,1")  || CheckResponse("+CREG: 0,5")  ||
                   CheckResponse("+CREG: 1,1")  || CheckResponse("+CREG: 1,5")) {
                    printf_gsm("[QUECTEL] Network REGISTERED -> GPS_INIT\r\n");
                    g_gsm_state = GSM_STATE_GPS_INIT;
                    s_sub_state = 0;
                    retry_count = 0;
                } else if((GetTick() - s_state_tick) >= 2000) {
                    if(++retry_count >= 15) {
                        printf_gsm("[QUECTEL] Network registration moving to GPS_INIT...\r\n");
                        g_gsm_state = GSM_STATE_GPS_INIT;
                        s_sub_state = 0;
                        retry_count = 0;
                    } else {
                        s_sub_state = 0; /* Re-check */
                    }
                }
            }
            break;

        case GSM_STATE_GPS_INIT:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: GPS_INIT - Powering ON GPS Engine (AT+QGPS=1)...\r\n");
                GSM_SendCommand("AT+QGPS=1");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("OK") || CheckResponse("ERROR") || (GetTick() - s_state_tick) >= 800) {
                    g_gsm_state = GSM_STATE_CTZU;
                    s_sub_state = 0;
                    retry_count = 0;
                }
            }
            break;

        case GSM_STATE_CTZU:
            if(s_sub_state == 0) {
                GSM_SendCommand("AT+CTZU=1");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("OK") || (GetTick() - s_state_tick) >= 500) {
                    g_gsm_state = GSM_STATE_CCLK_QUERY;
                    s_sub_state = 0;
                    retry_count = 0;
                }
            }
            break;

        case GSM_STATE_CCLK_QUERY:
        case GSM_STATE_PARSE_TIME:
            if(s_sub_state == 0) {
                GSM_SendCommand("AT+QLTS=2");
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("+QLTS:")) {
                    RTC_WrtFromStr((char*)g_gsm_rx_buffer);
                    g_gsm_state = GSM_STATE_QMTOPEN;
                    s_sub_state = 0;
                    connect_retry = 0;
                } else if((GetTick() - s_state_tick) >= 800) {
                    g_gsm_state = GSM_STATE_QMTOPEN;
                    s_sub_state = 0;
                    connect_retry = 0;
                }
            }
            break;

        case GSM_STATE_QMTOPEN:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: QMTOPEN - Connecting to %s:%d ...\r\n", first.MQTTServer, (int)first.MQTTPort);
                snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+QMTOPEN=0,\"%s\",%d", first.MQTTServer, (int)first.MQTTPort);
                GSM_SendCommand(cmd_buffer);
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("+QMTOPEN: 0,0") || CheckResponse("+QMTOPEN: 0,2")) {
                    printf_gsm("[QUECTEL] MQTT broker OPEN OK -> QMTCONN\r\n");
                    g_gsm_state = GSM_STATE_QMTCONN;
                    s_sub_state = 0;
                    connect_retry = 0;
                } else if(CheckResponse("ERROR") || (GetTick() - s_state_tick) >= 6000) {
                    if(++connect_retry >= 3) {
                        printf_gsm("[QUECTEL] QMTOPEN FAILED (retry=%d) -> ERROR\r\n", connect_retry);
                        g_gsm_state = GSM_STATE_ERROR;
                        s_sub_state = 0;
                    } else {
                        s_sub_state = 0; /* Retry */
                    }
                }
            }
            break;

        case GSM_STATE_QMTCONN:
            if(s_sub_state == 0) {
                printf_gsm("[QUECTEL] STATE: QMTCONN - client=%s\r\n", first.Device_ID);
                snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                        first.Device_ID, first.MQTTUsername, first.MQTTPassword);
                GSM_SendCommand(cmd_buffer);
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("+QMTCONN: 0,0,0")) {
                    printf_gsm("[QUECTEL] MQTT CONNECTED! -> QMTSUB\r\n");
                    g_gsm_state = GSM_STATE_QMTSUB;
                    s_sub_state = 0;
                    connect_retry = 0;
                    s_consec_errors = 0;
                } else if(CheckResponse("ERROR") || (GetTick() - s_state_tick) >= 6000) {
                    if(++connect_retry >= 3) {
                        printf_gsm("[QUECTEL] QMTCONN FAILED (retry=%d) -> ERROR\r\n", connect_retry);
                        g_gsm_state = GSM_STATE_ERROR;
                        s_sub_state = 0;
                    } else {
                        s_sub_state = 0; /* Retry */
                    }
                }
            }
            break;

        case GSM_STATE_QMTSUB:
            if(s_sub_state == 0) {
                const char *sn = (first.Serial_no[0] != '\0') ? first.Serial_no : "GTW26090001";
                snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+QMTSUB=0,1,\"SWA/%s/CONFIG/REQ\",0", sn);
                GSM_SendCommand(cmd_buffer);
                s_state_tick = GetTick();
                s_sub_state = 1;
            } else if(s_sub_state == 1) {
                if(CheckResponse("+QMTSUB:") || (GetTick() - s_state_tick) >= 2000) {
                    GSM_SendCommand("AT+QMTSUB=0,2,\"SWA/+/CONFIG/REQ\",0");
                    s_state_tick = GetTick();
                    s_sub_state = 2;
                }
            } else if(s_sub_state == 2) {
                if(CheckResponse("+QMTSUB:") || (GetTick() - s_state_tick) >= 2000) {
                    printf("[MQTT] Subscribed to Config Topics OK!\r\n");
                    g_gsm_state = GSM_STATE_STANDBY;
                    s_sub_state = 0;
                    ClearRXBuffer();
                    GSM_SyncNetworkTime();
                }
            }
            break;

        case GSM_STATE_PUBLISH:
            GSM_PublishSensorData();
            ClearRXBuffer();
            g_gsm_state = GSM_STATE_STANDBY;
            s_sub_state = 0;
            break;

        case GSM_STATE_PUBLISH_WAIT:
            g_gsm_state = GSM_STATE_STANDBY;
            s_sub_state = 0;
            break;

        case GSM_STATE_DISCONNECT:
            GSM_SendCommand("AT+QMTDISC=0");
            g_gsm_state = GSM_STATE_INIT;
            s_sub_state = 0;
            break;

        case GSM_STATE_STANDBY:
            /* Check for incoming config request messages */
            GSM_CheckIncomingMessages();

            if(g_trigger_publish) {
                if(s_sub_state == 0) {
                    ClearRXBuffer();
                    GSM_SendCommand("AT+CSQ");
                    s_state_tick = GetTick();
                    s_sub_state = 1;
                } else if(s_sub_state == 1) {
                    if(CheckResponse("+CSQ:") || (GetTick() - s_state_tick) >= 500) {
                        ParseCSQ((char*)g_gsm_rx_buffer);
                        ClearRXBuffer();
                        GSM_SendCommand("AT+QGPSGNMEA=\"RMC\"");
                        s_state_tick = GetTick();
                        s_sub_state = 2;
                    }
                } else if(s_sub_state == 2) {
                    if(CheckResponse("+QGPSGNMEA:") || (GetTick() - s_state_tick) >= 500) {
                        ProcessGPSFrame((char*)g_gsm_rx_buffer);
                        ClearRXBuffer();
                        g_trigger_publish = 0;
                        g_gsm_state = GSM_STATE_PUBLISH;
                        s_sub_state = 0;
                    }
                }
            }
            break;

        case GSM_STATE_ERROR:
            if(s_sub_state == 0) {
                s_consec_errors++;
                if(s_consec_errors >= 3) {
                    printf("[QUECTEL] Hardware power-cycling Quectel 4G module for auto-recovery...\r\n");
                    GPIO_ResetBits(IOT_EN_PORT, IOT_EN_PIN);
                    s_state_tick = GetTick();
                    s_sub_state = 1;
                } else {
                    GSM_SendCommand("AT+QMTDISC=0");
                    s_state_tick = GetTick();
                    s_sub_state = 2;
                }
            } else if(s_sub_state == 1) {
                if((GetTick() - s_state_tick) >= 1000) {
                    GSM_PowerOn();
                    s_consec_errors = 0;
                    g_gsm_state = GSM_STATE_INIT;
                    s_sub_state = 0;
                }
            } else if(s_sub_state == 2) {
                if((GetTick() - s_state_tick) >= 1000) {
                    g_gsm_state = GSM_STATE_INIT;
                    s_sub_state = 0;
                }
            }
            break;
    }
} 