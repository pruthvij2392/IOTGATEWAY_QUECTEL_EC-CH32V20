/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.h
 * Description        : CH32V203K IoT Gateway - Common Definitions Header
 *                      - LED pin macros (PB1 RED, PA1 BLUE, PA15 WHITE, PB0 YELLOW)
 *                      - RS-485 / MAX485 pin macros:
 *                          PA9  = USART1_TX -> MAX485 DI (Driver Input)
 *                          PA10 = USART1_RX <- MAX485 RO (Receiver Output)
 *                          PA8  = GPIO OUT  -> MAX485 DE+/RE tied (DEN)
 *                      - SPI Flash defines
 *                      - Debug UART: USART3 PB10 @ 115200 (or configured UART)
 *******************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

#include "ch32v20x.h"
#include "ds3231.h"
#include <stdint.h>

/* ==================== LED Pin Definitions ==================== */
#define LED_RED_PORT        GPIOB
#define LED_RED_PIN         GPIO_Pin_1    /* PB1 - RED    : Error / Connecting  */

#define LED_BLUE_PORT       GPIOA
#define LED_BLUE_PIN        GPIO_Pin_1    /* PA1 - BLUE   : MQTT Connected      */

#define LED_WHITE_PORT      GPIOA
#define LED_WHITE_PIN       GPIO_Pin_15   /* PA15 - WHITE : Data TX / Publish   */

#define LED_YELLOW_PORT     GPIOB
#define LED_YELLOW_PIN      GPIO_Pin_0    /* PB0 - YELLOW : 4G Power / Sensor   */

#define LED_RED_ON()        GPIO_SetBits(LED_RED_PORT, LED_RED_PIN)
#define LED_RED_OFF()       GPIO_ResetBits(LED_RED_PORT, LED_RED_PIN)
#define LED_RED_TOGGLE()    (LED_RED_PORT->OUTDR ^= LED_RED_PIN)

#define LED_BLUE_ON()       GPIO_SetBits(LED_BLUE_PORT, LED_BLUE_PIN)
#define LED_BLUE_OFF()      GPIO_ResetBits(LED_BLUE_PORT, LED_BLUE_PIN)
#define LED_BLUE_TOGGLE()   (LED_BLUE_PORT->OUTDR ^= LED_BLUE_PIN)

#define LED_WHITE_ON()      GPIO_SetBits(LED_WHITE_PORT, LED_WHITE_PIN)
#define LED_WHITE_OFF()     GPIO_ResetBits(LED_WHITE_PORT, LED_WHITE_PIN)
#define LED_WHITE_TOGGLE()  (LED_WHITE_PORT->OUTDR ^= LED_WHITE_PIN)

#define LED_YELLOW_ON()     GPIO_SetBits(LED_YELLOW_PORT, LED_YELLOW_PIN)
#define LED_YELLOW_OFF()    GPIO_ResetBits(LED_YELLOW_PORT, LED_YELLOW_PIN)
#define LED_YELLOW_TOGGLE() (LED_YELLOW_PORT->OUTDR ^= LED_YELLOW_PIN)

/* ==================== RS-485 / MAX485 Pin Definitions ==================== */
#define RS485_DEN_PIN       GPIO_Pin_8    /* PA8 -> MAX485 DE + /RE (tied)   */
#define RS485_DEN_PORT      GPIOA

#define RS485_DE_PIN        RS485_DEN_PIN
#define RS485_DE_PORT       RS485_DEN_PORT

#define RS485_MODE_TX       GPIO_SetBits(RS485_DEN_PORT, RS485_DEN_PIN)    /* TX: DE=1, /RE=1 */
#define RS485_MODE_RX       GPIO_ResetBits(RS485_DEN_PORT, RS485_DEN_PIN)  /* RX: DE=0, /RE=0 */

/* ==================== SPI Flash Defines ==================== */
#define FLASH_LOG_START_ADDR    0x00001000UL
#define FLASH_LOG_SECTOR_SIZE   4096U

/* ==================== RS-485 Sensor Buffer ==================== */
#define RX_BUFFER_SIZE          64U

/* ==================== Function Prototypes ==================== */
void     LED_Init(void);
void     Timer2_Init(void);
uint32_t GetTick(void);
void     Check_USB_Serial_Monitor(void);
void     Watchdog_Init(void);

#endif /* __MAIN_H */
