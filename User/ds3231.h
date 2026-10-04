/********************************** (C) COPYRIGHT *******************************
 * File Name          : ds3231.h
 * Description        : DS3231 Real-Time Clock + Temperature Driver
 *                      - Interface: I2C1 (PB6=SCL, PB7=SDA)
 *                      - I2C Address: 0x68
 *                      - Features: Time get/set, temperature read
 *******************************************************************************/
#ifndef __DS3231_H
#define __DS3231_H

#include "ch32v20x.h"
#include <stdint.h>
#include "gsm_quectel.h"   /* for GSM_Time_t used in DS3231_SetTime */

/* ==================== DS3231 I2C Address ==================== */
#define DS3231_ADDRESS      0x68

/* ==================== DS3231 Register Map ==================== */
#define DS3231_REG_SEC      0x00   /* Seconds       BCD [00-59] */
#define DS3231_REG_MIN      0x01   /* Minutes       BCD [00-59] */
#define DS3231_REG_HOUR     0x02   /* Hours         BCD [00-23] */
#define DS3231_REG_DOW      0x03   /* Day of Week   [1-7]       */
#define DS3231_REG_DATE     0x04   /* Date          BCD [01-31] */
#define DS3231_REG_MONTH    0x05   /* Month         BCD [01-12] */
#define DS3231_REG_YEAR     0x06   /* Year          BCD [00-99] */
#define DS3231_REG_CONTROL  0x0E   /* Control Register          */
#define DS3231_REG_STATUS   0x0F   /* Status Register           */
#define DS3231_REG_TEMP     0x11   /* Temp MSB (0x11), LSB (0x12) */

/* ==================== RTC Time Structure ==================== */
typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day_of_week;
    uint8_t date;
    uint8_t month;
    uint8_t year;        /* 2-digit: e.g. 26 = 2026 */
    float   temperature; /* degrees Celsius */
} RTC_TimeTypeDef;

/* ==================== Function Prototypes ==================== */
void    I2C_Bus_Unlock(void);
void    I2C1_Init(void);
uint8_t I2C_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t length);
uint8_t I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t length);
uint8_t BCD_to_Dec(uint8_t bcd);
void    DS3231_Init(void);
uint8_t DS3231_GetTime(RTC_TimeTypeDef *time);
uint8_t DS3231_SetTime(GSM_Time_t *time);
uint8_t DS3231_SetDateTimeValues(uint8_t date, uint8_t month, uint8_t year, uint8_t hour, uint8_t min, uint8_t sec);
uint8_t DS3231_SetDateStr(const char *date_str);
uint8_t DS3231_SetTimeStr(const char *time_str);
uint8_t RTC_WrtFromStr(const char *time_str);
float   DS3231_GetTemperature(void);

#endif /* __DS3231_H */
