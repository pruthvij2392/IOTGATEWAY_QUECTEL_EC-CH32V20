/********************************** (C) COPYRIGHT *******************************
 * File Name          : modbus_rtu.c
 * Description        : Modbus RTU RS-485 Master Driver for up to 16 Sensors (s1..s16)
 *                      - RS-485 Hardware: USART1 PA9=TX, PA10=RX, PA8=DE (Driver Enable)
 *******************************************************************************/
#include "modbus_rtu.h"
#include "main.h"
#include "comman.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>

Modbus_Sensor_Runtime_t g_sensors_runtime[MAX_MODBUS_SENSORS];
Modbus_Pressure_t       g_modbus_pressure;
uint8_t                 g_current_sensor_idx = 0;

static volatile uint8_t  s_rx_buf[MODBUS_RX_BUF_SIZE];
static volatile uint16_t s_rx_idx = 0;
static volatile uint32_t s_last_rx_byte_tick = 0;
static uint32_t          s_query_sent_tick = 0;
static uint8_t           s_query_in_progress = 0;

/* ==================== Standard Modbus RTU CRC16 Calculation ==================== */
uint16_t Modbus_CRC16(const uint8_t *buffer, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ==================== USART1 Interrupt Handler ==================== */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t byte = (uint8_t)USART_ReceiveData(USART1);
        if (s_rx_idx < (MODBUS_RX_BUF_SIZE - 1)) {
            s_rx_buf[s_rx_idx++] = byte;
        }
        s_last_rx_byte_tick = GetTick();
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/* ==================== Hardware Initialization ==================== */
void Modbus_Init(uint32_t baudrate) {
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA8 -> MAX485 DE + /RE (Driver Enable) */
    GPIO_InitStructure.GPIO_Pin   = RS485_DEN_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RS485_DEN_PORT, &GPIO_InitStructure);
    RS485_MODE_RX;

    /* PA9 -> USART1_TX (Alternate Function Push-Pull) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 -> USART1_RX (Input Floating) */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Baud Rate */
    USART_InitStructure.USART_BaudRate = (baudrate > 0) ? baudrate : ((first.sensor_baud > 0) ? first.sensor_baud : 9600);

    /* Parity & Word Length */
    if (strcasecmp(first.sensor_parity, "even") == 0 || strcasecmp(first.sensor_parity, "E") == 0) {
        USART_InitStructure.USART_Parity     = USART_Parity_Even;
        USART_InitStructure.USART_WordLength = (first.sensor_data_bits == 8) ? USART_WordLength_9b : USART_WordLength_8b;
    } else if (strcasecmp(first.sensor_parity, "odd") == 0 || strcasecmp(first.sensor_parity, "O") == 0) {
        USART_InitStructure.USART_Parity     = USART_Parity_Odd;
        USART_InitStructure.USART_WordLength = (first.sensor_data_bits == 8) ? USART_WordLength_9b : USART_WordLength_8b;
    } else {
        USART_InitStructure.USART_Parity     = USART_Parity_No;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }

    /* Stop Bits */
    if (first.sensor_stop_bits == 2) {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    } else {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 RX Interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

    s_rx_idx = 0;
    s_query_in_progress = 0;
}

/* ==================== Transmit Modbus RTU Query for Default / Active Sensor ==================== */
void Modbus_SendQuery(void) {
    Modbus_SendQueryForIndex(g_current_sensor_idx);
}

/* ==================== Transmit Modbus RTU Query for Sensor Index ==================== */
void Modbus_SendQueryForIndex(uint8_t idx) {
    if (idx >= MAX_MODBUS_SENSORS) return;
    const Modbus_Sensor_Config_t *cfg = &first.sensors[idx];
    if (!cfg->enabled) return;

    uint8_t query[8];
    uint8_t slave = (cfg->slave_id > 0) ? cfg->slave_id : (idx + 1);
    uint8_t fc    = (cfg->function_code > 0) ? cfg->function_code : 0x03;
    uint16_t reg  = cfg->start_register;
    uint16_t qty  = (cfg->num_registers > 0) ? cfg->num_registers : 2;

    query[0] = slave;
    query[1] = fc;
    query[2] = (uint8_t)((reg >> 8) & 0xFF);
    query[3] = (uint8_t)(reg & 0xFF);
    query[4] = (uint8_t)((qty >> 8) & 0xFF);
    query[5] = (uint8_t)(qty & 0xFF);

    uint16_t crc = Modbus_CRC16(query, 6);
    query[6] = (uint8_t)(crc & 0xFF);        /* CRC Low */
    query[7] = (uint8_t)((crc >> 8) & 0xFF); /* CRC High */

    s_rx_idx = 0;
    memset((void*)s_rx_buf, 0, sizeof(s_rx_buf));

    RS485_MODE_TX;
    Delay_Us(100);

    for (uint16_t i = 0; i < 8; i++) {
        USART_SendData(USART1, query[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    Delay_Us(100);
    RS485_MODE_RX;

    g_sensors_runtime[idx].last_poll_tick = GetTick();
    g_sensors_runtime[idx].total_queries++;
    s_query_sent_tick   = GetTick();
    s_query_in_progress = 1;
}

/* ==================== Parse Sensor Value & Assign to Telemetry ==================== */
/* ==================== Safe Embedded Float Formatter with Configurable Decimal Points (0..6) ==================== */
static void FormatFloatDP(char *buf, size_t sz, float val, uint8_t dp) {
    if (!buf || sz == 0) return;
    if (dp > 6) dp = 6;
    if (dp == 0) {
        long iv = (long)(val >= 0.0f ? (val + 0.5f) : (val - 0.5f));
        snprintf(buf, sz, "%ld", iv);
        return;
    }

    uint8_t neg = 0;
    if (val < 0.0f) {
        neg = 1;
        val = -val;
    }

    long mul = 1;
    for (uint8_t i = 0; i < dp; i++) mul *= 10;

    long v_int = (long)val;
    float frac = val - (float)v_int;
    long v_dec = (long)(frac * (float)mul + 0.5f);
    if (v_dec >= mul) {
        v_int++;
        v_dec = 0;
    }

    if (neg) {
        snprintf(buf, sz, "-%ld.%0*ld", v_int, (int)dp, v_dec);
    } else {
        snprintf(buf, sz, "%ld.%0*ld", v_int, (int)dp, v_dec);
    }
}

/* ==================== Parse Sensor Value According to value_format & Sensor Type ==================== */
static void Modbus_ParseSensorValue(uint8_t idx) {
    Modbus_Sensor_Config_t  *cfg = &first.sensors[idx];
    Modbus_Sensor_Runtime_t *rt  = &g_sensors_runtime[idx];

    if (rt->reg_count == 0) return;

    /* Raw 16-bit and 32-bit values */
    uint16_t r0 = rt->registers[0];
    uint16_t r1 = (rt->reg_count > 1) ? rt->registers[1] : 0;
    
    /* 
     * Byte-level extraction from Modbus RTU wire buffer:
     * Address 00 (b0) = MSB of Register 0 -> (r0 >> 8) & 0xFF
     * Address 01 (b1) = LSB of Register 0 -> r0 & 0xFF
     * Address 02 (b2) = MSB of Register 1 -> (r1 >> 8) & 0xFF
     * Address 03 (b3) = LSB of Register 1 -> r1 & 0xFF
     */
    uint8_t b0 = (uint8_t)((r0 >> 8) & 0xFF);
    uint8_t b1 = (uint8_t)(r0 & 0xFF);
    uint8_t b2 = (uint8_t)((r1 >> 8) & 0xFF);
    uint8_t b3 = (uint8_t)(r1 & 0xFF);

    /* 
     * 4-Mode Endianness Decoding for 32-bit & 16-bit:
     * 0: MODBUS_ENDIAN_BIG (ABCD - MSB first, standard Big-Endian)
     *    Address: 00 01 02 03 -> Data: 12 34 56 78 => Value = 0x12345678
     * 1: MODBUS_ENDIAN_LITTLE (DCBA - LSB first, standard Little-Endian)
     *    Address: 00 01 02 03 -> Data: 78 56 34 12 => Value = 0x12345678
     * 2: MODBUS_ENDIAN_BIG_SWAP (BADC - Big-Endian Byte Swap)
     *    Address: 00 01 02 03 -> Data: 34 12 78 56 => Value = 0x12345678
     * 3: MODBUS_ENDIAN_LITTLE_SWAP (CDAB - Little-Endian Byte Swap / Word Swap)
     *    Address: 00 01 02 03 -> Data: 56 78 12 34 => Value = 0x12345678
     */
    uint32_t val32;
    uint16_t val16_0;
    uint16_t val16_1;

    switch(cfg->endianness) {
        case MODBUS_ENDIAN_LITTLE: /* 1: Little-Endian (DCBA) */
            val32   = ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
            val16_0 = ((uint16_t)b1 << 8) | (uint16_t)b0;
            val16_1 = ((uint16_t)b3 << 8) | (uint16_t)b2;
            break;

        case MODBUS_ENDIAN_BIG_SWAP: /* 2: Big-Endian Byte Swap (BADC) */
            val32   = ((uint32_t)b1 << 24) | ((uint32_t)b0 << 16) | ((uint32_t)b3 << 8) | (uint32_t)b2;
            val16_0 = ((uint16_t)b1 << 8) | (uint16_t)b0;
            val16_1 = ((uint16_t)b3 << 8) | (uint16_t)b2;
            break;

        case MODBUS_ENDIAN_LITTLE_SWAP: /* 3: Little-Endian Byte Swap / Word Swap (CDAB) */
            val32   = ((uint32_t)b2 << 24) | ((uint32_t)b3 << 16) | ((uint32_t)b0 << 8) | (uint32_t)b1;
            val16_0 = ((uint16_t)b0 << 8) | (uint16_t)b1;
            val16_1 = ((uint16_t)b2 << 8) | (uint16_t)b3;
            break;

        case MODBUS_ENDIAN_BIG: /* 0: Big-Endian (ABCD) */
        default:
            val32   = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | (uint32_t)b3;
            val16_0 = r0;
            val16_1 = r1;
            break;
    }

    /* Decimal Point scaling: 0 to 6 (divisor = 10^dp) */
    uint8_t dp = (cfg->decimal_point <= 6) ? cfg->decimal_point : 2;
    float divisor = 1.0f;
    for (uint8_t i = 0; i < dp; i++) {
        divisor *= 10.0f;
    }

    const char *fmt = cfg->value_format;

    /* 1. 32-bit Signed Integer ("32bit-signed" / "int32" / "signed32" / "long") */
    if (strcasecmp(fmt, "32bit-signed") == 0 || strcasecmp(fmt, "32bit_signed") == 0 ||
        strcasecmp(fmt, "32-bit-signed") == 0 || strcasecmp(fmt, "signed32") == 0 ||
        strcasecmp(fmt, "int32") == 0 || strcasecmp(fmt, "s32") == 0 || strcasecmp(fmt, "long") == 0 ||
        strcasecmp(fmt, "bigndn") == 0 || strcasecmp(fmt, "littlendn") == 0) {

        int32_t s32 = (int32_t)val32;
        float f_val = ((float)s32) / divisor;
        rt->parsed_value = f_val;
        FormatFloatDP(rt->val_str, sizeof(rt->val_str), f_val, dp);
        strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
        rt->display_str[sizeof(rt->display_str) - 1] = '\0';
        rt->sensor_online = 1;

        if (idx == 0) {
            g_modbus_pressure.raw_pressure  = (uint32_t)s32;
            g_modbus_pressure.pressure_hpa  = f_val;
            g_modbus_pressure.pressure_kpa  = f_val / 10.0f;
            g_modbus_pressure.pressure_bar  = f_val / 1000.0f;
            g_modbus_pressure.sensor_online = 1;
        }
        return;
    }

    /* 2. 32-bit Unsigned Integer ("32bit-unsigned" / "uint32" / "unsigned32" / "ulong") */
    if (strcasecmp(fmt, "32bit-unsigned") == 0 || strcasecmp(fmt, "32bit_unsigned") == 0 ||
        strcasecmp(fmt, "32-bit-unsigned") == 0 || strcasecmp(fmt, "unsigned32") == 0 ||
        strcasecmp(fmt, "uint32") == 0 || strcasecmp(fmt, "u32") == 0 || strcasecmp(fmt, "ulong") == 0) {

        float f_val = ((float)val32) / divisor;
        rt->parsed_value = f_val;
        FormatFloatDP(rt->val_str, sizeof(rt->val_str), f_val, dp);
        strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
        rt->display_str[sizeof(rt->display_str) - 1] = '\0';
        rt->sensor_online = 1;
        return;
    }

    /* 3. 32-bit IEEE 754 Floating Point ("float" / "float32" / "32bit-float") */
    if (strcasecmp(fmt, "float") == 0 || strcasecmp(fmt, "float32") == 0 ||
        strcasecmp(fmt, "32bit-float") == 0 || strcasecmp(fmt, "32bit_float") == 0 ||
        strcasecmp(fmt, "32-bit-float") == 0) {

        if (cfg->num_registers >= 2) {
            union {
                uint32_t u32;
                float f;
            } uf;
            uf.u32 = val32;
            if (uf.f >= -100000.0f && uf.f <= 100000.0f && uf.f != 0.0f) {
                rt->parsed_value = uf.f;
            } else {
                rt->parsed_value = ((float)val32) / divisor;
            }
        } else {
            rt->parsed_value = ((float)val16_0) / divisor;
        }
        FormatFloatDP(rt->val_str, sizeof(rt->val_str), rt->parsed_value, dp);
        strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
        rt->display_str[sizeof(rt->display_str) - 1] = '\0';
        rt->sensor_online = 1;
        return;
    }

    /* 4. 16-bit / 32-bit Signed Integer ("signed" / "int16" / "16bit-signed" / "int") */
    if (strcasecmp(fmt, "signed") == 0 || strcasecmp(fmt, "int16") == 0 ||
        strcasecmp(fmt, "16bit-signed") == 0 || strcasecmp(fmt, "16bit_signed") == 0 ||
        strcasecmp(fmt, "16-bit-signed") == 0 || strcasecmp(fmt, "int") == 0 ||
        strcasecmp(fmt, "s16") == 0) {

        /* If 2 registers and 16-bit signed / int16 / signed, split into 2 channels (e.g. Temp & Humidity) */
        if (cfg->num_registers >= 2 && (strcasecmp(fmt, "signed") == 0 || strcasecmp(fmt, "int16") == 0 || 
            strcasecmp(fmt, "16bit-signed") == 0 || strcasecmp(fmt, "16bit_signed") == 0 || strcasecmp(fmt, "16-bit-signed") == 0)) {

            float temp_val = ((float)((int16_t)val16_0)) / divisor;
            rt->parsed_value = temp_val;
            FormatFloatDP(rt->val_str, sizeof(rt->val_str), temp_val, dp);
            snprintf(rt->display_str, sizeof(rt->display_str), "%s C", rt->val_str);
            rt->sensor_online = 1;

            if (rt->reg_count >= 2 && (idx + 1) < MAX_MODBUS_SENSORS) {
                uint8_t dp_next = (first.sensors[idx + 1].decimal_point <= 6) ? first.sensors[idx + 1].decimal_point : dp;
                float div_next = 1.0f;
                for (uint8_t d = 0; d < dp_next; d++) div_next *= 10.0f;

                float hum_val = ((float)((int16_t)val16_1)) / div_next;
                g_sensors_runtime[idx + 1].parsed_value = hum_val;
                FormatFloatDP(g_sensors_runtime[idx + 1].val_str, sizeof(g_sensors_runtime[idx + 1].val_str), hum_val, dp_next);
                snprintf(g_sensors_runtime[idx + 1].display_str, sizeof(g_sensors_runtime[idx + 1].display_str), "%s %%", g_sensors_runtime[idx + 1].val_str);
                g_sensors_runtime[idx + 1].sensor_online = 1;
            }
            return;
        }

        /* If 32-bit signed */
        if (cfg->num_registers >= 2 || strstr(fmt, "32") != NULL) {
            int32_t s32 = (int32_t)val32;
            float f_val = ((float)s32) / divisor;
            rt->parsed_value = f_val;
            FormatFloatDP(rt->val_str, sizeof(rt->val_str), f_val, dp);
            strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
            rt->display_str[sizeof(rt->display_str) - 1] = '\0';
            rt->sensor_online = 1;

            if (idx == 0) {
                g_modbus_pressure.raw_pressure  = (uint32_t)s32;
                g_modbus_pressure.pressure_hpa  = f_val;
                g_modbus_pressure.pressure_kpa  = f_val / 10.0f;
                g_modbus_pressure.pressure_bar  = f_val / 1000.0f;
                g_modbus_pressure.sensor_online = 1;
            }
            return;
        }

        int16_t s16 = (int16_t)val16_0;
        float f_val = ((float)s16) / divisor;
        rt->parsed_value = f_val;
        FormatFloatDP(rt->val_str, sizeof(rt->val_str), f_val, dp);
        strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
        rt->display_str[sizeof(rt->display_str) - 1] = '\0';
        rt->sensor_online = 1;
        return;
    }

    /* 5. 16-bit Unsigned Integer ("unsigned" / "uint16" / "16bit-unsigned" / "uint") */
    if (strcasecmp(fmt, "unsigned") == 0 || strcasecmp(fmt, "uint16") == 0 ||
        strcasecmp(fmt, "16bit-unsigned") == 0 || strcasecmp(fmt, "16bit_unsigned") == 0 ||
        strcasecmp(fmt, "16-bit-unsigned") == 0 || strcasecmp(fmt, "uint") == 0 ||
        strcasecmp(fmt, "u16") == 0) {

        float f_val = ((float)val16_0) / divisor;
        rt->parsed_value = f_val;
        FormatFloatDP(rt->val_str, sizeof(rt->val_str), f_val, dp);
        strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
        rt->display_str[sizeof(rt->display_str) - 1] = '\0';
        rt->sensor_online = 1;
        return;
    }

    /* 6. Fallback Default */
    float f_val = ((float)val16_0) / divisor;
    rt->parsed_value = f_val;
    FormatFloatDP(rt->val_str, sizeof(rt->val_str), f_val, dp);
    strncpy(rt->display_str, rt->val_str, sizeof(rt->display_str) - 1);
    rt->display_str[sizeof(rt->display_str) - 1] = '\0';
    rt->sensor_online = 1;
}

/* ==================== Multi-Sensor Round-Robin Modbus Processing Task ==================== */
void Modbus_Process_Task(uint32_t current_tick) {
    static uint32_t s_last_query_end_tick = 0;
    static uint8_t  s_active_idx = 0;

    /* 1. If currently waiting for response of active sensor query */
    if (s_query_in_progress) {
        const Modbus_Sensor_Config_t *cfg = &first.sensors[s_active_idx];
        uint8_t slave          = (cfg->slave_id > 0) ? cfg->slave_id : (s_active_idx + 1);
        uint8_t fc             = (cfg->function_code > 0) ? cfg->function_code : 0x03;
        uint8_t expected_bytes = (uint8_t)(cfg->num_registers * 2);
        if (expected_bytes == 0) expected_bytes = 4;
        uint16_t frame_len     = 3 + expected_bytes + 2; /* Addr + FC + ByteCount + DataBytes + 2 CRC */
        uint32_t timeout_val   = (cfg->timeout_ms >= 50) ? cfg->timeout_ms : 1000UL;
        /* If sensor is currently offline, shorten timeout to 250ms to keep bus snappy */
        if (!g_sensors_runtime[s_active_idx].sensor_online && timeout_val > 250UL) {
            timeout_val = 250UL;
        }

        /* Check if response received */
        if (s_rx_idx >= frame_len || (s_rx_idx > 0 && (current_tick - s_last_rx_byte_tick) >= 30)) {
            if (s_rx_idx >= frame_len) {
                int start = -1;
                for (int i = 0; i <= (int)(s_rx_idx - frame_len); i++) {
                    if (s_rx_buf[i] == slave && 
                        s_rx_buf[i + 1] == fc && 
                        s_rx_buf[i + 2] == expected_bytes) {
                        start = i;
                        break;
                    }
                }

                if (start >= 0) {
                    uint8_t *frame = (uint8_t*)&s_rx_buf[start];
                    uint16_t received_crc = ((uint16_t)frame[3 + expected_bytes + 1] << 8) | frame[3 + expected_bytes];
                    uint16_t calculated_crc = Modbus_CRC16(frame, 3 + expected_bytes);

                    if (received_crc == calculated_crc) {
                        /* Valid CRC & Response */
                        uint8_t num_regs = expected_bytes / 2;
                        if (num_regs > 4) num_regs = 4;
                        g_sensors_runtime[s_active_idx].reg_count = num_regs;

                        for (uint8_t r = 0; r < num_regs; r++) {
                            g_sensors_runtime[s_active_idx].registers[r] = 
                                ((uint16_t)frame[3 + (r * 2)] << 8) | frame[3 + (r * 2) + 1];
                        }

                        g_sensors_runtime[s_active_idx].last_response_tick = current_tick;
                        g_sensors_runtime[s_active_idx].total_responses++;
                        g_sensors_runtime[s_active_idx].retry_counter = 0;
                        g_sensors_runtime[s_active_idx].sensor_online = 1;

                        /* Parse and format sensor data */
                        Modbus_ParseSensorValue(s_active_idx);
                    } else {
                        g_sensors_runtime[s_active_idx].total_crc_errors++;
                    }
                }
            }

            s_rx_idx = 0;
            s_query_in_progress = 0;
            s_last_query_end_tick = current_tick;
            s_active_idx++; /* Advance to next sensor */
        }
        /* Check for timeout */
        else if ((current_tick - s_query_sent_tick) > timeout_val) {
            /* Sensor disconnected or no response: reset online state and clear value immediately */
            g_sensors_runtime[s_active_idx].sensor_online = 0;
            g_sensors_runtime[s_active_idx].parsed_value = 0.0f;
            g_sensors_runtime[s_active_idx].val_str[0] = '\0';
            g_sensors_runtime[s_active_idx].display_str[0] = '\0';
            
            if (s_active_idx == 0) {
                g_modbus_pressure.sensor_online = 0;
                g_modbus_pressure.pressure_hpa = 0.0f;
                g_modbus_pressure.raw_pressure = 0;
            }

            /* Also clear linked channel for multi-register sensor */
            if ((s_active_idx + 1) < MAX_MODBUS_SENSORS) {
                g_sensors_runtime[s_active_idx + 1].sensor_online = 0;
                g_sensors_runtime[s_active_idx + 1].parsed_value = 0.0f;
                g_sensors_runtime[s_active_idx + 1].val_str[0] = '\0';
                g_sensors_runtime[s_active_idx + 1].display_str[0] = '\0';
            }

            s_rx_idx = 0;
            s_query_in_progress = 0;
            s_last_query_end_tick = current_tick;
            s_active_idx++; /* Advance to next sensor */
        }
        return;
    }

    /* Inter-query guard time: wait 30ms before sending next Modbus query */
    if ((current_tick - s_last_query_end_tick) < 30) return;

    /* 2. Find next enabled sensor channel to query */
    if (s_active_idx >= MAX_MODBUS_SENSORS) {
        s_active_idx = 0;
    }

    uint8_t found = 0;
    for (uint8_t i = 0; i < MAX_MODBUS_SENSORS; i++) {
        uint8_t check_idx = (s_active_idx + i) % MAX_MODBUS_SENSORS;
        if (first.sensors[check_idx].enabled && first.sensors[check_idx].slave_id > 0) {
            /* If previous sensor (check_idx - 1) is enabled on the same slave_id with >=2 registers, it already polled this slave */
            if (check_idx > 0 && 
                first.sensors[check_idx - 1].enabled && 
                first.sensors[check_idx - 1].slave_id == first.sensors[check_idx].slave_id &&
                first.sensors[check_idx - 1].num_registers >= 2) {
                continue;
            }
            s_active_idx = check_idx;
            found = 1;
            break;
        }
    }

    if (!found) return; /* No enabled sensors */

    /* Send Modbus query immediately */
    g_current_sensor_idx = s_active_idx;
    Modbus_SendQueryForIndex(s_active_idx);
}

uint8_t Modbus_GetPressure(float *out_hpa, uint32_t *out_raw) {
    if (out_hpa) *out_hpa = g_modbus_pressure.pressure_hpa;
    if (out_raw) *out_raw = g_modbus_pressure.raw_pressure;
    return g_modbus_pressure.sensor_online;
}


