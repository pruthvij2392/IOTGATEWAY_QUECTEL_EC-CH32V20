/********************************** (C) COPYRIGHT *******************************
 * File Name          : w25qxx.h
 * Description        : W25Q32 SPI Flash Driver Header for CH32V203
 *                      - SPI1: PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI
 *******************************************************************************/
#ifndef __W25QXX_H
#define __W25QXX_H

#include "ch32v20x.h"
#include <stdint.h>

/* ==================== W25Q32 Pin Definitions ==================== */
#define W25Q_CS_PORT            GPIOA
#define W25Q_CS_PIN             GPIO_Pin_4    /* PA4 - SPI1_NSS / CS */

#define W25Q_SCK_PORT           GPIOA
#define W25Q_SCK_PIN            GPIO_Pin_5    /* PA5 - SPI1_SCK */

#define W25Q_MISO_PORT          GPIOA
#define W25Q_MISO_PIN           GPIO_Pin_6    /* PA6 - SPI1_MISO (IO1) */

#define W25Q_MOSI_PORT          GPIOA
#define W25Q_MOSI_PIN           GPIO_Pin_7    /* PA7 - SPI1_MOSI (IO0) */

#define W25Q_CS_LOW()           GPIO_ResetBits(W25Q_CS_PORT, W25Q_CS_PIN)
#define W25Q_CS_HIGH()          GPIO_SetBits(W25Q_CS_PORT, W25Q_CS_PIN)

/* ==================== Flash Geometry ==================== */
#define W25Q32_PAGE_SIZE        256U
#define W25Q32_SECTOR_SIZE      4096U
#define W25Q32_TOTAL_SIZE       (4U * 1024U * 1024U)   /* 4 MBytes */
#define W25Q32_DEVICE_ID        0xEF4016U

/* ==================== Function Prototypes ==================== */
void     W25Qxx_Init(void);
uint32_t W25Qxx_ReadID(void);
void     W25Qxx_EraseSector(uint32_t sector_addr);
void     W25Qxx_WritePage(const uint8_t *buffer, uint32_t write_addr, uint16_t length);
void     W25Qxx_WriteData(const uint8_t *buffer, uint32_t write_addr, uint32_t length);
void     W25Qxx_ReadData(uint8_t *buffer, uint32_t read_addr, uint32_t length);
uint8_t  W25Qxx_Test(void);

#endif /* __W25QXX_H */
