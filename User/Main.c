/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : PRUTHVI JYOTI (SWASEMI PVT LTD)
 * Version            : V2.1.0
 * Date               : 2026/09/22
 * Description        : CH32V203K SWAIOT Gateway
 *                      - USB CDC : Primary Serial Monitor (Docklight & Command CLI)
 *                      - RS-485  : USART1 (PA9/PA10, DE=PA8) - Developer banner
 *                      - RTC     : DS3231 on I2C1 (PB6=SCL, PB7=SDA)
 *                      - Flash   : W25Q32 SPI Flash (PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI)
 *                      - 4G IoT  : Quectel Module on USART2 (PA2=TX, PA3=RX, PB4=EN, PB3=PWR)
 *                      - LEDs    : PB1 RED, PA1 BLUE, PA15 WHITE, PB0 YELLOW
 *******************************************************************************/
#include "ch32v20x.h"
#include "debug.h"
#include "main.h"
#include "comman.h"
#include "ds3231.h"
#include "gsm_quectel.h"
#include "modbus_rtu.h"
#include "w25qxx.h"
#include "usb_lib.h"
#include "UART.h"
#include "ch32v20x_iwdg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ==================== Global Variables ==================== */
volatile uint32_t system_ms_counter = 0;
uint8_t           sensor_rx[RX_BUFFER_SIZE];
uint16_t          rx_len = 0;
RTC_TimeTypeDef   current_time;

/* ==================== System Tick ==================== */
uint32_t GetTick(void) {
    return system_ms_counter;
}

/* ==================== LED Initialization ==================== */
void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* PA1 (BLUE) & PA15 (WHITE) */
    GPIO_InitStructure.GPIO_Pin   = LED_BLUE_PIN | LED_WHITE_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PB0 (YELLOW) & PB1 (RED) */
    GPIO_InitStructure.GPIO_Pin   = LED_YELLOW_PIN | LED_RED_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    LED_RED_OFF();
    LED_BLUE_OFF();
    LED_WHITE_OFF();
    LED_YELLOW_OFF();
}

/* ==================== Helper: Extract Value after '=' or ':' or ' ' ==================== */
static char* ExtractParamValue(char *cmd) {
    char *p = strchr(cmd, '=');
    if(!p) p = strchr(cmd, ':');
    if(!p) p = strchr(cmd, ' ');
    if(p) {
        p++;
        while(*p == ' ' || *p == '\t' || *p == '"' || *p == '\'') p++;
        int l = strlen(p);
        while(l > 0 && (p[l-1] == ' ' || p[l-1] == '\t' || p[l-1] == '"' || p[l-1] == '\'' || p[l-1] == '\r' || p[l-1] == '\n')) {
            p[--l] = '\0';
        }
        return p;
    }
    return NULL;
}

/* ==================== USB CDC Serial Monitor Command Processing ==================== */
static void Process_USB_Command(char *cmd) {
    // Remove leading/trailing spaces, quotes, carriage returns & newlines
    while(*cmd == ' ' || *cmd == '\t' || *cmd == '"' || *cmd == '\'') cmd++;
    int len = strlen(cmd);
    if(len == 0) return;

    /* 0. Full JSON Processing for USB Application */
    if(cmd[0] == '{' || strncasecmp(cmd, "SET_CONFIGSENSOR", 16) == 0 || strncasecmp(cmd, "SET_SENSOR_JSON", 15) == 0) {
        char *p_json = (cmd[0] == '{') ? cmd : ExtractParamValue(cmd);
        if(p_json && p_json[0] == '{') {
            char act[32] = {0};
            JsonExtractStr(p_json, "act", act, sizeof(act), "");

            /* Action: set-config (General Network / Broker Settings) */
            if(strcasecmp(act, "set-config") == 0) {
                /* If changing serial number, check password */
                char sn_req[24] = {0};
                JsonExtractStr(p_json, "sn", sn_req, sizeof(sn_req), "");
                if(sn_req[0] != '\0' && strcasecmp(sn_req, first.Serial_no) != 0) {
                    char pass_req[32] = {0};
                    JsonExtractStr(p_json, "pwd", pass_req, sizeof(pass_req), "");
                    if(pass_req[0] == '\0') JsonExtractStr(p_json, "pass", pass_req, sizeof(pass_req), "");
                    if(pass_req[0] == '\0') JsonExtractStr(p_json, "password", pass_req, sizeof(pass_req), "");
                    if(strcmp(pass_req, CONFIG_AUTH_PASS) != 0) {
                        printf("{\r\n  \"status\": \"error\"\r\n}\r\n");
                        return;
                    }
                }
                if(Config_ApplyGeneralJson(p_json)) {
                    printf("{\r\n  \"status\": \"success\"\r\n}\r\n");
                } else {
                    printf("{\r\n  \"status\": \"error\"\r\n}\r\n");
                }
                return;
            }
            /* Action: get-config (Read General Settings) */
            else if(strcasecmp(act, "get-config") == 0) {
                NVM_PrintConfig();
                return;
            }
            /* Action: set-time (Configure RTC Clock) */
            else if(strcasecmp(act, "set-time") == 0) {
                if(Config_ApplyTimeJson(p_json)) {
                    printf("{\r\n  \"status\": \"success\"\r\n}\r\n");
                } else {
                    printf("{\r\n  \"status\": \"error\"\r\n}\r\n");
                }
                return;
            }
            /* Action: get-time (Read RTC Clock) */
            else if(strcasecmp(act, "get-time") == 0) {
                RTC_TimeTypeDef t;
                DS3231_GetTime(&t);
                printf("{\n  \"st\": \"success\",\n  \"ver\": \"2.0.0\",\n  \"act\": \"get-time\",\n  \"d\": {\n    \"dt\": \"%02d/%02d/%02d\",\n    \"tm\": \"%02d:%02d:%02d\"\n  }\n}\r\n",
                       t.date, t.month, t.year, t.hour, t.minute, t.second);
                return;
            }
            /* Action: get-sensor-data / get-data (Live Telemetry Frame) */
            else if(strcasecmp(act, "get-sensor-data") == 0 || strcasecmp(act, "get-data") == 0 || strcasecmp(act, "get_sensor_data") == 0) {
                static char sensor_frame[512];
                READING_FRAME(sensor_frame, sizeof(sensor_frame));
                printf("%s\r\n", sensor_frame);
                return;
            }
            /* Action: get-configsensor / get-sensor (Read or Set Sensor Settings) */
            else if(strcasecmp(act, "get-configsensor") == 0 || strcasecmp(act, "get-sensor") == 0) {
                if (strstr(p_json, "\"s1\"") || strstr(p_json, "\"s2\"") || strstr(p_json, "\"s3\"") ||
                    strstr(p_json, "\"device_id\"") || strstr(p_json, "\"baud_rate\"")) {
                    if(Sensor_ApplyConfigJson(p_json)) {
                        Sensor_PrintConfigJson();
                    } else {
                        printf("{\r\n  \"status\": \"error\"\r\n}\r\n");
                    }
                } else {
                    Sensor_PrintConfigJson();
                }
                return;
            }
            /* Action: reset / reboot (Software Reset) */
            else if(strcasecmp(act, "reset") == 0 || strcasecmp(act, "reboot") == 0) {
                printf("{\r\n  \"status\": \"success\"\r\n}\r\n");
                Delay_Ms(200);
                NVIC_SystemReset();
                return;
            }
            /* Default: Sensor / Device Setup JSON (set-configsensor / s1..s16 / baud / slave_id / etc.) */
            else {
                if(Sensor_ApplyConfigJson(p_json)) {
                    printf("{\r\n  \"status\": \"success\"\r\n}\r\n");
                } else {
                    printf("{\r\n  \"status\": \"error\"\r\n}\r\n");
                }
                return;
            }
        }
    }

    /* 1. Command: DOWNLOAD / GET_READING_DATA -> Dump all flash stored frames in CSV */
    if(strcasecmp(cmd, "DOWNLOAD") == 0 ||
       strcasecmp(cmd, "GET_READING_DATA") == 0 ||
       strstr(cmd, "DOWNLOAD") != NULL ||
       strstr(cmd, "download") != NULL ||
       strstr(cmd, "GET_READING_DATA") != NULL ||
       strstr(cmd, "get_reading_data") != NULL) {
        USB_DumpAllReadingData();
    }
    /* 2. Command: GET_SENSOR_DATA -> Output current live sensor frame */
    else if(strcasecmp(cmd, "GET_SENSOR_DATA") == 0 ||
            strstr(cmd, "GET_SENSOR_DATA") != NULL ||
            strstr(cmd, "get_sensor_data") != NULL) {
        static char sensor_frame_cmd[512];
        READING_FRAME(sensor_frame_cmd, sizeof(sensor_frame_cmd));
        printf("%s\r\n", sensor_frame_cmd);
    }
    /* 3. Command: CONFIG / GET_CONFIG -> Display All Non-Volatile Memory Settings */
    else if(strcasecmp(cmd, "CONFIG") == 0 || strcasecmp(cmd, "GET_CONFIG") == 0) {
        NVM_PrintConfig();
    }
    /* 4. Command: SET_DEVICE_ID=<id> / SET_DEVICE_NAME=<name> (e.g. GTW26090001 or 26090001) */
    else if(strncasecmp(cmd, "SET_DEVICE_ID", 13) == 0 || strncasecmp(cmd, "SET_DEVICEID", 12) == 0 ||
            strncasecmp(cmd, "SET_DEVICE_NAME", 15) == 0 || strncasecmp(cmd, "SET_NAME", 8) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            if (strncasecmp(val, "GTW", 3) != 0) {
                snprintf(first.Device_ID, sizeof(first.Device_ID), "GTW%s", val);
            } else {
                strncpy(first.Device_ID, val, sizeof(first.Device_ID) - 1);
            }
            first.Device_ID[sizeof(first.Device_ID) - 1] = '\0';
            strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);
            NVM_SaveConfig();
            printf("[CONFIG] Device Name / ID set to: %s (Saved to NVM)\r\n", first.Device_ID);
        } else {
            printf("[CONFIG] Device Name / ID: %s\r\n", first.Device_ID);
        }
    }
    /* 5. Command: SET_SERIAL_NO=<PASS>=<SN> (Requires Exact Password: $$Swasemi$11cr$$) */
    else if(strncasecmp(cmd, "SET_SERIAL_NO", 13) == 0 || strncasecmp(cmd, "SET_SERIAL", 10) == 0 || strncasecmp(cmd, "SET_SN", 6) == 0) {
        char *p_delim = strchr(cmd, '=');
        if(!p_delim) p_delim = strchr(cmd, ':');
        if(!p_delim) {
            printf("[CONFIG] Serial No: %s\r\n", first.Serial_no);
            return;
        }

        p_delim++;
        while(*p_delim == ' ' || *p_delim == '\t' || *p_delim == '"' || *p_delim == '\'') p_delim++;

        /* Find 2nd separator between <PASSWORD> and <SERIAL_NO> */
        char *p_sep2 = strchr(p_delim, '=');
        if(!p_sep2) p_sep2 = strchr(p_delim, ',');
        if(!p_sep2) p_sep2 = strchr(p_delim, ':');

        if(!p_sep2) {
            printf("[CONFIG] ERROR: Password Required to change Serial No!\r\n"
                   "[CONFIG] Format: SET_SERIAL_NO=%s=GTW26090001\r\n", CONFIG_AUTH_PASS);
            return;
        }

        /* Extract Password */
        int pass_len = (int)(p_sep2 - p_delim);
        char pass_buf[32] = {0};
        if(pass_len > 0 && pass_len < (int)sizeof(pass_buf)) {
            strncpy(pass_buf, p_delim, pass_len);
            pass_buf[pass_len] = '\0';
        }
        /* Trim trailing whitespace/quotes from password */
        int pl = strlen(pass_buf);
        while(pl > 0 && (pass_buf[pl-1] == ' ' || pass_buf[pl-1] == '\t' || pass_buf[pl-1] == '"' || pass_buf[pl-1] == '\'')) {
            pass_buf[--pl] = '\0';
        }

        /* Strict Password Verification */
        if(strcmp(pass_buf, CONFIG_AUTH_PASS) != 0) {
            printf("[CONFIG] ERROR: Invalid Password! Serial No not changed.\r\n"
                   "[CONFIG] Format: SET_SERIAL_NO=%s=GTW26090001\r\n", CONFIG_AUTH_PASS);
            return;
        }

        /* Extract New Serial Number */
        char *p_sn = p_sep2 + 1;
        while(*p_sn == ' ' || *p_sn == '\t' || *p_sn == '"' || *p_sn == '\'') p_sn++;
        int sn_len = strlen(p_sn);
        while(sn_len > 0 && (p_sn[sn_len-1] == ' ' || p_sn[sn_len-1] == '\t' || p_sn[sn_len-1] == '"' || p_sn[sn_len-1] == '\'' || p_sn[sn_len-1] == '\r' || p_sn[sn_len-1] == '\n')) {
            p_sn[--sn_len] = '\0';
        }

        if(sn_len == 0) {
            printf("[CONFIG] ERROR: Serial No cannot be empty!\r\n");
            return;
        }

        if (strncasecmp(p_sn, "GTW", 3) != 0) {
            snprintf(first.Serial_no, sizeof(first.Serial_no), "GTW%s", p_sn);
        } else {
            strncpy(first.Serial_no, p_sn, sizeof(first.Serial_no) - 1);
        }
        first.Serial_no[sizeof(first.Serial_no) - 1] = '\0';
        strncpy(first.Device_ID, first.Serial_no, sizeof(first.Device_ID) - 1);
        strncpy(temp.Device_ID, first.Device_ID, sizeof(temp.Device_ID) - 1);
        NVM_SaveConfig();
        printf("[CONFIG] Password Verified OK! Serial No set to: %s (Saved to NVM)\r\n", first.Serial_no);
    }
    /* 6. Command: SET_BROKER=<host> / SET_MQTT_SERVER=<host> */
    else if(strncasecmp(cmd, "SET_BROKER", 10) == 0 || strncasecmp(cmd, "SET_MQTT_SERVER", 15) == 0 || strncasecmp(cmd, "SET_SERVER", 10) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            strncpy(first.MQTTServer, val, sizeof(first.MQTTServer) - 1);
            first.MQTTServer[sizeof(first.MQTTServer) - 1] = '\0';
            NVM_SaveConfig();
            printf("[CONFIG] MQTT Broker set to: %s (Saved to NVM)\r\n", first.MQTTServer);
        } else {
            printf("[CONFIG] MQTT Broker: %s\r\n", first.MQTTServer);
        }
    }
    /* 7. Command: SET_PORT=<port> */
    else if(strncasecmp(cmd, "SET_PORT", 8) == 0 || strncasecmp(cmd, "SET_MQTT_PORT", 13) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            int port = atoi(val);
            if(port > 0 && port <= 65535) {
                first.MQTTPort = (uint16_t)port;
                NVM_SaveConfig();
                printf("[CONFIG] MQTT Port set to: %u (Saved to NVM)\r\n", (unsigned int)first.MQTTPort);
            } else {
                printf("[CONFIG] Invalid Port number (1 - 65535)\r\n");
            }
        } else {
            printf("[CONFIG] MQTT Port: %u\r\n", (unsigned int)first.MQTTPort);
        }
    }
    /* 8. Command: SET_USER=<user> / SET_MQTT_USER=<user> */
    else if(strncasecmp(cmd, "SET_USER", 8) == 0 || strncasecmp(cmd, "SET_MQTT_USER", 13) == 0 || strncasecmp(cmd, "SET_USERNAME", 12) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            strncpy(first.MQTTUsername, val, sizeof(first.MQTTUsername) - 1);
            first.MQTTUsername[sizeof(first.MQTTUsername) - 1] = '\0';
            NVM_SaveConfig();
            printf("[CONFIG] MQTT Username set to: %s (Saved to NVM)\r\n", first.MQTTUsername);
        } else {
            printf("[CONFIG] MQTT Username: %s\r\n", first.MQTTUsername);
        }
    }
    /* 9. Command: SET_PASS=<pass> / SET_PASSWORD=<pass> */
    else if(strncasecmp(cmd, "SET_PASS", 8) == 0 || strncasecmp(cmd, "SET_PASSWORD", 12) == 0 || strncasecmp(cmd, "SET_MQTT_PASS", 13) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            strncpy(first.MQTTPassword, val, sizeof(first.MQTTPassword) - 1);
            first.MQTTPassword[sizeof(first.MQTTPassword) - 1] = '\0';
            NVM_SaveConfig();
            printf("[CONFIG] MQTT Password updated (Saved to NVM)\r\n");
        } else {
            printf("[CONFIG] MQTT Password: %s\r\n", first.MQTTPassword);
        }
    }
    /* 10. Command: SET_TOPIC=<topic> / SET_MQTT_TOPIC=<topic> */
    else if(strncasecmp(cmd, "SET_TOPIC", 9) == 0 || strncasecmp(cmd, "SET_MQTT_TOPIC", 14) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            strncpy(first.MQTTPubTopic, val, sizeof(first.MQTTPubTopic) - 1);
            first.MQTTPubTopic[sizeof(first.MQTTPubTopic) - 1] = '\0';
            NVM_SaveConfig();
            printf("[CONFIG] MQTT Topic set to: %s (Saved to NVM)\r\n", first.MQTTPubTopic);
        } else {
            printf("[CONFIG] MQTT Topic: %s\r\n", first.MQTTPubTopic);
        }
    }
    /* 11. Command: SET_SAMPLING_SEC=<sec> / SET_SAMPLE_TIME=<sec> */
    else if(strncasecmp(cmd, "SET_SAMPLING_SEC", 16) == 0 || strncasecmp(cmd, "SET_SAMPLE_TIME", 15) == 0 || strncasecmp(cmd, "SET_SAMPLING", 12) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && strlen(val) > 0) {
            int sec = atoi(val);
            if(sec >= 1) {
                first.Sample_Time = (uint16_t)sec;
                NVM_SaveConfig();
                printf("[CONFIG] Sampling Interval Set: %u sec (Saved to NVM)\r\n", (unsigned int)first.Sample_Time);
            } else {
                printf("[CONFIG] Invalid Sampling Interval (Minimum 1 sec)\r\n");
            }
        } else {
            printf("[CONFIG] Current Sampling Interval: %u sec\r\n", (unsigned int)first.Sample_Time);
        }
    }
    else if(strcasecmp(cmd, "GET_SAMPLING_SEC") == 0) {
        printf("[CONFIG] Current Sampling Interval: %u sec\r\n", (unsigned int)first.Sample_Time);
    }
    /* 12. Command: SAVE_CONFIG / LOAD_CONFIG / RESET_CONFIG */
    else if(strcasecmp(cmd, "SAVE_CONFIG") == 0) {
        NVM_SaveConfig();
        printf("[NVM] Settings saved to Flash Sector 0 successfully.\r\n");
    }
    else if(strcasecmp(cmd, "LOAD_CONFIG") == 0) {
        NVM_Init();
        printf("[NVM] Settings reloaded from Flash Sector 0.\r\n");
    }
    else if(strcasecmp(cmd, "RESET_CONFIG") == 0 || strcasecmp(cmd, "FACTORY_RESET") == 0 || strcasecmp(cmd, "DEFAULT_CONFIG") == 0) {
        printf("[RESET_CONFIG] Resetting all gateway settings to factory defaults...\r\n");
        NVM_ResetToDefaults();
        printf("[RESET_CONFIG] SUCCESS: All parameters reset to factory defaults and saved to NVM. (Data logs preserved)\r\n");
        NVM_PrintConfig();
    }
    /* 13. Command: RESET_EEPROM / RESET_FLASH / CLEAR_LOGS -> Erase all logged data frames */
    else if(strcasecmp(cmd, "RESET_EEPROM") == 0 ||
            strcasecmp(cmd, "RESET_DATA") == 0 ||
            strcasecmp(cmd, "RESET_FLASH") == 0 ||
            strcasecmp(cmd, "CLEAR_LOGS") == 0)  {
        printf("[RESET_EEPROM] Starting full erase of data logging frames...\r\n");
        Reset_EEPROM();
    }
    /* 14. Command: STOP_DOWNLOAD / CANCEL_DOWNLOAD */
    else if(strcasecmp(cmd, "STOP_DOWNLOAD") == 0 ||
            strcasecmp(cmd, "STOP") == 0 ||
            strcasecmp(cmd, "CANCEL_DOWNLOAD") == 0) {
        USB_StopDump();
    }
    /* 15. Command: STATUS */
    else if(strcasecmp(cmd, "STATUS") == 0 || strcasecmp(cmd, "SYSTEM_STATUS") == 0) {
        int csq_dbm = (g_csq_val <= 31) ? (-113 + (int)g_csq_val * 2) : 0;
        printf("---------- SYSTEM STATUS ----------\r\n"
               "Serial No    : %s\r\n"
               "Device Name  : %s\r\n"
               "GSM State    : %d\r\n"
               "MQTT Status  : %s\r\n"
               "Signal (CSQ) : %d dBm\r\n"
               "Sampling Sec : %u sec\r\n"
               "Broker IP    : %s\r\n"
               "Broker Port  : %u\r\n"
               "MQTT User    : %s\r\n"
               "MQTT Pass    : %s\r\n"
               "Flash Frames : %u\r\n"
               "Download     : %s\r\n",
               first.Serial_no,
               (first.Device_ID[0] != '\0') ? first.Device_ID : first.Serial_no,
               g_gsm_state,
               GSM_IsConnected() ? "CONNECTED" : "DISCONNECTED",
               csq_dbm, (unsigned int)first.Sample_Time,
               first.MQTTServer, (unsigned int)first.MQTTPort,
               first.MQTTUsername, first.MQTTPassword,
               (unsigned int)g_total_logged_frames,
               g_usb_dump_active ? "IN PROGRESS" : "IDLE");

        if(DS3231_GetTime(&current_time)) {
            printf("RTC Time     : 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                   current_time.year, current_time.month, current_time.date,
                   current_time.hour, current_time.minute, current_time.second);
        }
        printf("------------------------------------\r\n");
    }
    /* 17. Command: POLL_SENSOR / READ_SENSOR */
    else if(strcasecmp(cmd, "POLL_SENSOR") == 0 || strcasecmp(cmd, "READ_SENSOR") == 0) {
        printf("[MODBUS] Sending query frame: 02 03 00 00 00 02 C4 38 ...\r\n");
        Modbus_SendQuery();
    }
    /* 18. Command: GET_TIME / TIME / DATE */
    else if(strcasecmp(cmd, "TIME") == 0 || strcasecmp(cmd, "GET_TIME") == 0 || strcasecmp(cmd, "DATE") == 0 || strcasecmp(cmd, "GET_DATE") == 0) {
        if(DS3231_GetTime(&current_time)) {
            int t_int = (int)current_time.temperature;
            int t_dec = (int)((current_time.temperature - t_int) * 10);
            if(t_dec < 0) t_dec = -t_dec;
            printf("[DS3231] 20%02d-%02d-%02d %02d:%02d:%02d (%d.%d C)\r\n",
                   current_time.year, current_time.month, current_time.date,
                   current_time.hour, current_time.minute, current_time.second,
                   t_int, t_dec);
        } else {
            printf("[DS3231] ERROR reading RTC!\r\n");
        }
    }
    /* Command: SET_DATETIME / SET_DATE / SET_TIME */
    else if(strncasecmp(cmd, "SET_DATETIME", 12) == 0 || strncasecmp(cmd, "SET_RTC", 7) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && RTC_WrtFromStr(val)) {
            printf("[RTC] DateTime Set OK\r\n");
        } else {
            printf("[RTC] ERR: Use DD/MM/YY HH:MM:SS\r\n");
        }
    }
    else if(strncasecmp(cmd, "SET_DATE", 8) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && DS3231_SetDateStr(val)) {
            printf("[RTC] Date Set OK\r\n");
        } else {
            printf("[RTC] ERR: Use DD/MM/YY\r\n");
        }
    }
    else if(strncasecmp(cmd, "SET_TIME", 8) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && DS3231_SetTimeStr(val)) {
            printf("[RTC] Time Set OK\r\n");
        } else {
            printf("[RTC] ERR: Use HH:MM:SS\r\n");
        }
    }
    /* 19. Command: SYNC_TIME / SYNC_RTC */
    else if(strcasecmp(cmd, "SYNC_TIME") == 0 || strcasecmp(cmd, "SYNC_RTC") == 0) {
        printf("[RTC] Syncing RTC with 4G Network Time...\r\n");
        if(GSM_SyncNetworkTime()) {
            DS3231_GetTime(&current_time);
            printf("[RTC] Synced OK: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                   current_time.year, current_time.month, current_time.date,
                   current_time.hour, current_time.minute, current_time.second);
        } else {
            printf("[RTC] Sync Failed: GSM Time not available yet\r\n");
        }
    }
    /* 20. Command: FLASH */
    else if(strcasecmp(cmd, "FLASH") == 0) {
        uint32_t fid = W25Qxx_ReadID();
        printf("[FLASH] JEDEC ID: 0x%06X (Logged Frames: %u, Next Addr: 0x%06X)\r\n", 
               (unsigned int)fid, (unsigned int)g_total_logged_frames, (unsigned int)g_current_flash_addr);
    }
    /* 21. Command: CSQ / SIGNAL / CSQ_STATUS */
    else if(strcasecmp(cmd, "CSQ") == 0 || strcasecmp(cmd, "SIGNAL") == 0 || strcasecmp(cmd, "CSQ_STATUS") == 0) {
        int csq_dbm = (g_csq_val <= 31) ? (-113 + (int)g_csq_val * 2) : 0;
        int csq_pct = (g_csq_val <= 31) ? ((int)g_csq_val * 100 / 31) : 0;
        printf("[GSM] Signal Strength: %d dBm (%u%%, CSQ: %u)\r\n", csq_dbm, (unsigned int)csq_pct, (unsigned int)g_csq_val);
    }
    /* 22. Command: REQ_CONFIG / GET_CONFIG_REQ -> Publishes get-config request to SWA/<Device_ID>/CONFIG/REQ */
    else if(strcasecmp(cmd, "REQ_CONFIG") == 0 || strcasecmp(cmd, "GET_CONFIG_REQ") == 0) {
        printf("[MQTT] Sending Config Request (get-config)...\r\n");
        GSM_PublishConfigRequest();
    }
    /* 23. Command: RES_CONFIG / SEND_CONFIG_RES -> Publishes get-config success response to SWA/<Device_ID>/CONFIG/RES */
    else if(strcasecmp(cmd, "RES_CONFIG") == 0 || strcasecmp(cmd, "SEND_CONFIG_RES") == 0) {
        printf("[MQTT] Sending Config Response (success)...\r\n");
        GSM_PublishConfigResponse();
    }
    /* 24. Command: GET_CONFIGSENSOR / SENSOR_CONFIG -> Print JSON to USB */
    else if(strcasecmp(cmd, "GET_CONFIGSENSOR") == 0 || strcasecmp(cmd, "SENSOR_CONFIG") == 0 || strcasecmp(cmd, "GET_SENSOR_CONFIG") == 0) {
        printf("\r\n[CONFIG] Current Sensor Configuration (JSON):\r\n");
        Sensor_PrintConfigJson();
    }
    /* 25. Command: SET_MODBUS_BAUD=<baud> / SET_BAUD=<baud> / BAUD_RATE=<baud> / BAUD=<baud> */
    else if(strncasecmp(cmd, "SET_MODBUS_BAUD", 15) == 0 || strncasecmp(cmd, "SET_BAUD", 8) == 0 ||
            strncasecmp(cmd, "BAUD_RATE", 9) == 0 || strncasecmp(cmd, "BAUD", 4) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && *val) {
            int b = atoi(val);
            if(b == 2400 || b == 4800 || b == 9600 || b == 19200 || b == 38400 || b == 57600 || b == 115200) {
                first.sensor_baud = (uint32_t)b;
                NVM_SaveConfig();
                Modbus_Init(first.sensor_baud);
                printf("[CONFIG] Baud: %u\r\n", (unsigned int)first.sensor_baud);
            }
        } else {
            printf("[CONFIG] Baud: %u\r\n", (unsigned int)first.sensor_baud);
        }
    }
    /* 26. Command: SET_DATA_BITS=<7|8> / DATA_BITS=<7|8> / DATA_BIT=<7|8> */
    else if(strncasecmp(cmd, "SET_DATA_BITS", 13) == 0 || strncasecmp(cmd, "SET_DATABITS", 12) == 0 ||
            strncasecmp(cmd, "SET_DATA_BIT", 12) == 0 || strncasecmp(cmd, "DATA_BITS", 9) == 0 ||
            strncasecmp(cmd, "DATA_BIT", 8) == 0 || strncasecmp(cmd, "DATABITS", 8) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && *val) {
            int db = atoi(val);
            if(db == 7 || db == 8) {
                first.sensor_data_bits = (uint8_t)db;
                NVM_SaveConfig();
                Modbus_Init(first.sensor_baud);
                printf("[CONFIG] Data Bits: %u\r\n", (unsigned int)first.sensor_data_bits);
            }
        } else {
            printf("[CONFIG] Data Bits: %u\r\n", (unsigned int)first.sensor_data_bits);
        }
    }
    /* 27. Command: SET_PARITY=<NONE|EVEN|ODD> / PARITY=<NONE|EVEN|ODD> */
    else if(strncasecmp(cmd, "SET_PARITY", 10) == 0 || strncasecmp(cmd, "PARITY", 6) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && *val) {
            if(strcasecmp(val, "even") == 0 || strcasecmp(val, "E") == 0) strncpy(first.sensor_parity, "even", sizeof(first.sensor_parity)-1);
            else if(strcasecmp(val, "odd") == 0 || strcasecmp(val, "O") == 0) strncpy(first.sensor_parity, "odd", sizeof(first.sensor_parity)-1);
            else strncpy(first.sensor_parity, "none", sizeof(first.sensor_parity)-1);
            first.sensor_parity[sizeof(first.sensor_parity)-1] = '\0';
            NVM_SaveConfig();
            Modbus_Init(first.sensor_baud);
            printf("[CONFIG] Parity: %s\r\n", first.sensor_parity);
        } else {
            printf("[CONFIG] Parity: %s\r\n", first.sensor_parity);
        }
    }
    /* 28. Command: SET_STOP_BITS=<1|2> / STOP_BITS=<1|2> / STOP_BIT=<1|2> */
    else if(strncasecmp(cmd, "SET_STOP_BITS", 13) == 0 || strncasecmp(cmd, "SET_STOPBITS", 12) == 0 ||
            strncasecmp(cmd, "SET_STOP_BIT", 12) == 0 || strncasecmp(cmd, "STOP_BITS", 9) == 0 ||
            strncasecmp(cmd, "STOP_BIT", 8) == 0 || strncasecmp(cmd, "STOPBITS", 8) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && *val) {
            int sb = atoi(val);
            if(sb == 1 || sb == 2) {
                first.sensor_stop_bits = (uint8_t)sb;
                NVM_SaveConfig();
                Modbus_Init(first.sensor_baud);
                printf("[CONFIG] Stop Bits: %u\r\n", (unsigned int)first.sensor_stop_bits);
            }
        } else {
            printf("[CONFIG] Stop Bits: %u\r\n", (unsigned int)first.sensor_stop_bits);
        }
    }
    /* Command: SET_TOTAL_SENSORS=<1..16> / TOTAL_SENSORS=<1..16> / TOTAL_NO_OF_SENSORS=<1..16> */
    else if(strncasecmp(cmd, "SET_TOTAL_SENSORS", 17) == 0 || strncasecmp(cmd, "TOTAL_SENSORS", 13) == 0 ||
            strncasecmp(cmd, "SET_TOTAL_SENSOR", 16) == 0 || strncasecmp(cmd, "TOTAL_SENSOR", 12) == 0 ||
            strncasecmp(cmd, "TOTAL_NO_OF_SENSORS", 19) == 0 || strncasecmp(cmd, "SET_TOTAL_NO_OF_SENSORS", 23) == 0 ||
            strncasecmp(cmd, "SET_SENSORS", 11) == 0) {
        char *val = ExtractParamValue(cmd);
        if(val && *val) {
            int ts = atoi(val);
            if(ts >= 1 && ts <= 16) {
                first.total_sensors = (uint8_t)ts;
                for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
                    first.sensors[i].enabled = (i < first.total_sensors) ? 1 : 0;
                }
                NVM_SaveConfig();
                printf("[CONFIG] Total Sensors set to: %u (Saved to NVM)\r\n", (unsigned int)first.total_sensors);
            } else {
                printf("[CONFIG] Invalid Total Sensors (Range: 1 - 16)\r\n");
            }
        } else {
            printf("[CONFIG] Total Sensors: %u\r\n", (unsigned int)first.total_sensors);
        }
    }
    /* 38. Command: RESET / REBOOT */
    else if(strcasecmp(cmd, "RESET") == 0 || strcasecmp(cmd, "REBOOT") == 0) {
        printf("[SYSTEM] Rebooting...\r\n");
        Delay_Ms(100);
        NVIC_SystemReset();
    }
    /* 39. Command: HELP / ? */
    else if(strcasecmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        printf("CMD: STATUS, RESET_EEPROM, RESET_CONFIG, READ_FLASH, TIME, CSQ, {JSON}\r\n");
    }
}

void Check_USB_Serial_Monitor(void) {
    static char cmd_buf[1024];
    static uint16_t cmd_idx = 0;
    static int16_t brace_depth = 0;
    static uint32_t last_rx_tick = 0;

    while(USB_Data_Available()) {
        char c = (char)USB_Receive_Data();
        last_rx_tick = GetTick();

        if (c == '{') {
            brace_depth++;
        } else if (c == '}') {
            if (brace_depth > 0) brace_depth--;
        }

        if (brace_depth > 0) {
            /* Inside JSON: accumulate everything */
            if (cmd_idx < (sizeof(cmd_buf) - 1)) {
                cmd_buf[cmd_idx++] = c;
            }
        }
        else if (brace_depth == 0 && cmd_idx > 0 && cmd_buf[0] == '{') {
            /* Closing brace was just received! Append '}' and dispatch entire JSON */
            if (cmd_idx < (sizeof(cmd_buf) - 1)) {
                cmd_buf[cmd_idx++] = c;
            }
            cmd_buf[cmd_idx] = '\0';
            Process_USB_Command(cmd_buf);
            cmd_idx = 0;
            brace_depth = 0;
        }
        else if (c == '\r' || c == '\n') {
            /* Single-line text command */
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                Process_USB_Command(cmd_buf);
                cmd_idx = 0;
                brace_depth = 0;
            }
        }
        else if (c == '\b' || c == 0x7F) {
            if (cmd_idx > 0) cmd_idx--;
        }
        else if (cmd_idx < (sizeof(cmd_buf) - 1)) {
            cmd_buf[cmd_idx++] = c;
        }
    }

    /* Reset buffer if incomplete JSON is idle for > 1 second */
    if (cmd_idx > 0 && (GetTick() - last_rx_tick) > 1000) {
        cmd_idx = 0;
        brace_depth = 0;
    }
}

/* ==================== Status LED Indication Logic ====================
 * WHITE  : GPS Connected (Fix Acquired) -> Blink every 3 sec, Else OFF
 * RED    : MCU Power ON -> Continuously ON
 * YELLOW : Cellular Network Connected -> Blink every 3 sec, Else OFF
 * BLUE   : MQTT Connected -> Blink every 3 sec, Else OFF
 * ====================================================================== */
static void Update_Status_LEDs(uint32_t current_tick) {
    /* 1. RED LED: Continuously ON when MCU is powered ON & running */
    LED_RED_ON();

    /* 3-Second Blink Pulse: 200ms ON / 2800ms OFF */
    uint32_t phase = current_tick % 3000UL;
    uint8_t blink_pulse = (phase < 200UL);

    /* 2. WHITE LED: GPS Connected (Fix) -> 3 sec blink, Else -> OFF */
    if (GSM_IsGPSFixed()) {
        if (blink_pulse) {
            LED_WHITE_ON();
        } else {
            LED_WHITE_OFF();
        }
    } else {
        LED_WHITE_OFF();
    }

    /* 3. YELLOW LED: Cellular Network (N/W) Connected -> 3 sec blink, Else -> OFF */
    if (GSM_IsNetworkRegistered()) {
        if (blink_pulse) {
            LED_YELLOW_ON();
        } else {
            LED_YELLOW_OFF();
        }
    } else {
        LED_YELLOW_OFF();
    }

    /* 4. BLUE LED: MQTT Connected -> 3 sec blink, Else -> OFF */
    if (GSM_IsConnected()) {
        if (blink_pulse) {
            LED_BLUE_ON();
        } else {
            LED_BLUE_OFF();
        }
    } else {
        LED_BLUE_OFF();
    }
}

/* ==================== Hardware Independent Watchdog (IWDG) ====================
 * CH32V203 LSI is ~40kHz. With Prescaler 256, tick is 6.4ms.
 * Reload 1250 gives ~8.0 seconds timeout window.
 * Ensures system automatically recovers/reboots if any peripheral or loop hangs.
 * ============================================================================== */
void Watchdog_Init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(1250);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* ==================== Main Application Loop (Cooperative Multi-Tasking) ==================== */
int main(void) {
    uint32_t last_gsm_process      = 0;
    uint32_t last_periodic_publish = 0;
    uint32_t last_led_update       = 0;
    uint32_t current_tick          = 0;

    /* 1. Core & Clock Initialization */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    /* 2. Initialize Status LEDs */
    LED_Init();

    /* 3. Initialize Millisecond System Tick (TIM2) */
    TIM2_Init();

    /* 4. Initialize USB CDC (Serial Monitor Interface) */
    RCC_Configuration();
    Set_USBConfig();
    USB_Init();
    USB_Interrupts_Config();

    /* 5. Initialize Modbus RTU RS-485 (USART1 PA9/PA10, DE=PA8 @ 9600 Baud) */
    Modbus_Init(9600);

    /* 6. Initialize I2C1 & DS3231 RTC */
    DS3231_Init();

    /* 7. Initialize SPI1 & W25Q32 Flash */
    W25Qxx_Init();

    /* 8. Initialize Non-Volatile Memory (NVM) Settings from W25Q32 Flash Sector 0 */
    NVM_Init();

    /* 9. Scan and recover existing logged frames from Flash (preserves data across reboot/power off) */
    Flash_Log_Init();

    /* 10. Initialize Quectel 4G IoT Module Pins & Power */
    GSM_Pins_Init();
    LED_YELLOW_ON();
    GSM_PowerOn();
    LED_YELLOW_OFF();

    /* 11. Initialize GSM UART (USART2 PA2/PA3 @ 115200) */
    GSM_Init();

    /* 12. Startup LED Animation (Blink 3 times) */
    for(int i = 0; i < 3; i++) {
        LED_RED_ON();  LED_BLUE_ON();  LED_WHITE_ON();  LED_YELLOW_ON();
        Delay_Ms(100);
        LED_RED_OFF(); LED_BLUE_OFF(); LED_WHITE_OFF(); LED_YELLOW_OFF();
        Delay_Ms(100);
    }

    /* Turn Red LED ON permanently as Power ON indicator */
    LED_RED_ON();

    /* Print Clean Startup Banner to USB CDC Serial Monitor */
    printf("\r\n=== CH32V203 SWAIOT Gateway ===\r\n");
    W25Qxx_Test();
    NVM_PrintConfig();

    /* 13. Initialize Hardware Independent Watchdog (IWDG) for 24/7 self-healing */
    Watchdog_Init();

    /* ==================== Main Application Loop ==================== */
    while(1) {
        /* Keep hardware watchdog alive on each active iteration */
        IWDG_ReloadCounter();

        current_tick = GetTick();

        /* Task 1: Check for incoming commands from USB CDC Serial Monitor (Docklight) */
        Check_USB_Serial_Monitor();

        /* Task 2: Non-blocking USB Data Stream Task (Runs in background concurrently with IoT) */
        USB_Dump_Process_Slice();

        /* Task 3: Modbus RTU RS-485 Pressure Sensor Query & Parser Task */
        Modbus_Process_Task(current_tick);

        /* Task 4: Process GSM 4G & MQTT state machine every 100ms */
        if((current_tick - last_gsm_process) >= 100) {
            last_gsm_process = current_tick;
            GSM_ProcessState();
        }

        /* Task 5: Update Status LEDs every 50ms based on GPS, N/W, and MQTT states */
        if((current_tick - last_led_update) >= 50) {
            last_led_update = current_tick;
            Update_Status_LEDs(current_tick);
        }

        /* Task 6: Periodic MQTT Data Publish based on configured sampling interval */
        uint32_t publish_interval_ms = (uint32_t)first.Sample_Time * 1000UL;
        if(publish_interval_ms < 1000UL) publish_interval_ms = 1000UL; /* Minimum 1 sec */

        if((current_tick - last_periodic_publish) >= publish_interval_ms) {
            last_periodic_publish = current_tick;
            if(GSM_IsConnected()) {
                GSM_TriggerPublish();
            }
        }
    }
}