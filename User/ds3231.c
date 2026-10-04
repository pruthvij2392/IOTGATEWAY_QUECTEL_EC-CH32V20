/********************************** (C) COPYRIGHT *******************************
 * File Name          : ds3231.c
 * Description        : DS3231 Real-Time Clock + Temperature Driver Implementation
 *                      - Interface: I2C1 (PB6=SCL, PB7=SDA, 100kHz)
 *                      - I2C Address: 0x68
 *******************************************************************************/
#include "ds3231.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>

/* ==================== I2C Bus Auto-Recovery (Clock 9 Pulses) ==================== */
void I2C_Bus_Unlock(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Configure PB6 (SCL) and PB7 (SDA) as Open-Drain GPIO outputs */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    Delay_Us(10);

    /* Clock SCL 9 times to release any slave holding SDA low */
    for(int i = 0; i < 9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(10);
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(10);
    }

    /* Generate STOP condition */
    GPIO_ResetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(10);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    Delay_Us(10);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(10);
}

/* ==================== I2C1 Initialization ==================== */
/* PB6 = SCL, PB7 = SDA, Open-Drain, 100kHz                     */
void I2C1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitStructure  = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

/* ==================== Low-Level I2C Write ==================== */
uint8_t I2C_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t length) {
    uint32_t timeout;

    /* Check bus busy */
    timeout = 50000;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if(--timeout == 0) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            I2C_SoftwareResetCmd(I2C1, ENABLE);
            I2C_SoftwareResetCmd(I2C1, DISABLE);
            I2C1_Init();
            return 0;
        }
    }

    /* Send START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Send device address in Write direction */
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Send register pointer */
    I2C_SendData(I2C1, reg_addr);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Send data bytes */
    for(uint8_t i = 0; i < length; i++) {
        I2C_SendData(I2C1, data[i]);
        timeout = 50000;
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
        }
    }

    /* Send STOP */
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 1;
}

/* ==================== Low-Level I2C Read ==================== */
uint8_t I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t length) {
    uint32_t timeout;

    /* Check bus busy */
    timeout = 50000;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if(--timeout == 0) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            I2C_SoftwareResetCmd(I2C1, ENABLE);
            I2C_SoftwareResetCmd(I2C1, DISABLE);
            I2C1_Init();
            return 0;
        }
    }

    /* Send START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Send device address (Write) to point to register */
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Send register address */
    I2C_SendData(I2C1, reg_addr);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Repeated START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Send device address (Read direction) */
    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Receiver);
    timeout = 50000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if(--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
    }

    /* Read bytes */
    for(uint8_t i = 0; i < length; i++) {
        if(i == length - 1) {
            /* NACK on last byte and generate STOP */
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        timeout = 50000;
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if(--timeout == 0) {
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                I2C_GenerateSTOP(I2C1, ENABLE);
                return 0;
            }
        }
        buffer[i] = I2C_ReceiveData(I2C1);
    }

    /* Re-enable ACK for next read */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

/* ==================== BCD <-> Decimal ==================== */
uint8_t BCD_to_Dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

/* ==================== DS3231 Init ==================== */
void DS3231_Init(void) {
    I2C_Bus_Unlock();
    I2C1_Init();

    /* Enable DS3231 Oscillator: Reg 0x0E (EOSC = 0, BBSQW = 0) */
    uint8_t ctrl = 0x00;
    I2C_WriteBytes(DS3231_ADDRESS, DS3231_REG_CONTROL, &ctrl, 1);

    /* Clear Oscillator Stop Flag: Reg 0x0F (OSF = 0) */
    uint8_t status = 0x00;
    I2C_WriteBytes(DS3231_ADDRESS, DS3231_REG_STATUS, &status, 1);
}

/* ==================== DS3231 Get Temperature ==================== */
float DS3231_GetTemperature(void) {
    uint8_t temp_msb = 0, temp_lsb = 0;

    if(!I2C_ReadBytes(DS3231_ADDRESS, DS3231_REG_TEMP,   &temp_msb, 1)) return 29.5f;
    if(!I2C_ReadBytes(DS3231_ADDRESS, DS3231_REG_TEMP+1, &temp_lsb, 1)) return 29.5f;

    return (float)(int8_t)temp_msb + ((temp_lsb >> 6) * 0.25f);
}

/* ==================== DS3231 Get Time ==================== */
uint8_t DS3231_GetTime(RTC_TimeTypeDef *time) {
    uint8_t data[7];

    if(!I2C_ReadBytes(DS3231_ADDRESS, DS3231_REG_SEC, data, 7)) {
        return 0;
    }

    time->second      = BCD_to_Dec(data[0] & 0x7F);
    time->minute      = BCD_to_Dec(data[1] & 0x7F);
    time->hour        = BCD_to_Dec(data[2] & 0x3F); /* 24-hour mode */
    time->day_of_week = BCD_to_Dec(data[3] & 0x07);
    time->date        = BCD_to_Dec(data[4] & 0x3F);
    time->month       = BCD_to_Dec(data[5] & 0x1F);
    time->year        = BCD_to_Dec(data[6]);
    time->temperature = DS3231_GetTemperature();

    return 1;
}

/* ==================== DS3231 Set Time ==================== */
uint8_t DS3231_SetTime(GSM_Time_t *time) {
    uint8_t data[7];

    if(!time) return 0;

    /* Decimal to BCD */
    data[0] = ((time->second / 10) << 4) | (time->second % 10);
    data[1] = ((time->minute / 10) << 4) | (time->minute % 10);
    data[2] = ((time->hour   / 10) << 4) | (time->hour   % 10); /* 24-hr format */
    data[3] = 0x01;  /* day-of-week = Monday (1-7) */
    data[4] = ((time->date   / 10) << 4) | (time->date   % 10);
    data[5] = ((time->month  / 10) << 4) | (time->month  % 10);
    data[6] = ((time->year   / 10) << 4) | (time->year   % 10);

    return I2C_WriteBytes(DS3231_ADDRESS, DS3231_REG_SEC, data, 7);
}

static int ParseNextInt(const char **pp) {
    const char *p = *pp;
    while (*p && (*p < '0' || *p > '9')) p++;
    if (!*p) { *pp = p; return -1; }
    int val = 0;
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        p++;
    }
    *pp = p;
    return val;
}

/* ==================== Write RTC From Network String (AT+QLTS=2 / AT+CCLK) ==================== */
uint8_t RTC_WrtFromStr(const char *time_str) {
    if(!time_str) return 0;

    const char *p = strstr(time_str, "+QLTS:");
    if(!p) p = strstr(time_str, "+CCLK:");
    if(!p) p = time_str;

    int p1 = ParseNextInt(&p);
    int p2 = ParseNextInt(&p);
    int p3 = ParseNextInt(&p);
    int hh = ParseNextInt(&p);
    int mm = ParseNextInt(&p);
    int ss = ParseNextInt(&p);

    if(p1 >= 0 && p2 >= 0 && p3 >= 0 && hh >= 0 && mm >= 0 && ss >= 0) {
        int y = 0, m = 0, d = 0;
        /* Determine year, month, date order */
        if(p1 >= 2000) {
            y = p1 - 2000;
            m = p2;
            d = p3;
        } else if(p1 >= 20 && p1 <= 99 && p2 >= 1 && p2 <= 12 && p3 >= 1 && p3 <= 31) {
            /* Standard Quectel YY/MM/DD (e.g. 26/09/22) */
            y = p1;
            m = p2;
            d = p3;
        } else if(p3 >= 20 && p3 <= 99 && p2 >= 1 && p2 <= 12 && p1 >= 1 && p1 <= 31) {
            /* DD/MM/YY format (e.g. 22/09/26) */
            d = p1;
            m = p2;
            y = p3;
        } else {
            y = (p1 >= 2000) ? (p1 - 2000) : p1;
            m = p2;
            d = p3;
        }

        if(m >= 1 && m <= 12 && d >= 1 && d <= 31 && hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0 && ss <= 59) {
            GSM_Time_t t;
            t.year   = (uint8_t)y;
            t.month  = (uint8_t)m;
            t.date   = (uint8_t)d;
            t.hour   = (uint8_t)hh;
            t.minute = (uint8_t)mm;
            t.second = (uint8_t)ss;
            g_gsm_time = t;
            return DS3231_SetTime(&t);
        }
    }
    return 0;
}

/* ==================== Set Date and Time with Explicit Integer Values ==================== */
uint8_t DS3231_SetDateTimeValues(uint8_t date, uint8_t month, uint8_t year, uint8_t hour, uint8_t min, uint8_t sec) {
    if(month < 1 || month > 12 || date < 1 || date > 31 || hour > 23 || min > 59 || sec > 59) return 0;
    GSM_Time_t t;
    t.date   = date;
    t.month  = month;
    t.year   = (year >= 100) ? (uint8_t)(year - 2000) : year;
    t.hour   = hour;
    t.minute = min;
    t.second = sec;
    g_gsm_time = t;
    return DS3231_SetTime(&t);
}

/* ==================== Set Date String (DD/MM/YY or DD-MM-YYYY) ==================== */
uint8_t DS3231_SetDateStr(const char *date_str) {
    if (!date_str) return 0;
    const char *p = date_str;
    int d = ParseNextInt(&p);
    int m = ParseNextInt(&p);
    int y = ParseNextInt(&p);
    if (d < 0 || m < 0 || y < 0) return 0;

    RTC_TimeTypeDef curr;
    if (!DS3231_GetTime(&curr)) {
        curr.hour = 12; curr.minute = 0; curr.second = 0;
    }
    if (d > 31 && y <= 31) { int tmp = d; d = y; y = tmp; }
    if (y >= 2000) y -= 2000;
    return DS3231_SetDateTimeValues((uint8_t)d, (uint8_t)m, (uint8_t)y, curr.hour, curr.minute, curr.second);
}

/* ==================== Set Time String (HH:MM:SS) ==================== */
uint8_t DS3231_SetTimeStr(const char *time_str) {
    if (!time_str) return 0;
    const char *p = time_str;
    int hh = ParseNextInt(&p);
    int mm = ParseNextInt(&p);
    int ss = ParseNextInt(&p);
    if (hh < 0 || mm < 0) return 0;
    if (ss < 0) ss = 0;

    RTC_TimeTypeDef curr;
    if (!DS3231_GetTime(&curr) || curr.year < 20) {
        curr.date = 29; curr.month = 9; curr.year = 26;
    }
    return DS3231_SetDateTimeValues(curr.date, curr.month, curr.year, (uint8_t)hh, (uint8_t)mm, (uint8_t)ss);
}
