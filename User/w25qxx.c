/********************************** (C) COPYRIGHT *******************************
 * File Name          : w25qxx.c
 * Description        : W25Q32 SPI Flash Driver using Hardware SPI1 on CH32V203
 *                      PA4 = CS (GPIO Out)
 *                      PA5 = SCK (SPI1_SCK, AF_PP)
 *                      PA6 = MISO / IO1 (SPI1_MISO, IN_FLOATING)
 *                      PA7 = MOSI / IO0 (SPI1_MOSI, AF_PP)
 *******************************************************************************/
#include "w25qxx.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>

/* ==================== Commands ==================== */
#define W25X_WRITE_ENABLE       0x06
#define W25X_WRITE_DISABLE      0x04
#define W25X_READ_STATUS_R1     0x05
#define W25X_READ_DATA          0x03
#define W25X_FAST_READ_DATA     0x0B
#define W25X_PAGE_PROGRAM       0x02
#define W25X_SECTOR_ERASE       0x20
#define W25X_JEDEC_ID           0x9F

/* ==================== Low-Level SPI1 Helper ==================== */
static uint8_t SPI1_ReadWriteByte(uint8_t byte) {
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, byte);
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

static void W25Qxx_WaitBusy(void) {
    W25Q_CS_LOW();
    SPI1_ReadWriteByte(W25X_READ_STATUS_R1);
    while((SPI1_ReadWriteByte(0xFF) & 0x01) == 0x01); /* Wait until BUSY=0 */
    W25Q_CS_HIGH();
}

static void W25Qxx_WriteEnable(void) {
    W25Q_CS_LOW();
    SPI1_ReadWriteByte(W25X_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

/* ==================== Hardware Initialization ==================== */
void W25Qxx_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    /* PA4 - CS (Output Push-Pull) */
    GPIO_InitStructure.GPIO_Pin   = W25Q_CS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25Q_CS_PORT, &GPIO_InitStructure);
    W25Q_CS_HIGH();

    /* PA5 (SCK) & PA7 (MOSI) -> AF Push-Pull */
    GPIO_InitStructure.GPIO_Pin   = W25Q_SCK_PIN | W25Q_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA6 (MISO) -> Floating Input */
    GPIO_InitStructure.GPIO_Pin  = W25Q_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* SPI1 Configuration: Master, 8-bit, Mode 0 (CPOL=0, CPHA=0), Prescaler=4 */
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}

/* ==================== Read JEDEC ID ==================== */
uint32_t W25Qxx_ReadID(void) {
    uint32_t id = 0;
    W25Q_CS_LOW();
    SPI1_ReadWriteByte(W25X_JEDEC_ID);
    id |= ((uint32_t)SPI1_ReadWriteByte(0xFF)) << 16;
    id |= ((uint32_t)SPI1_ReadWriteByte(0xFF)) << 8;
    id |= ((uint32_t)SPI1_ReadWriteByte(0xFF));
    W25Q_CS_HIGH();
    return id;
}

/* ==================== Erase Sector (4KB) ==================== */
void W25Qxx_EraseSector(uint32_t sector_addr) {
    W25Qxx_WriteEnable();
    W25Qxx_WaitBusy();

    W25Q_CS_LOW();
    SPI1_ReadWriteByte(W25X_SECTOR_ERASE);
    SPI1_ReadWriteByte((uint8_t)((sector_addr >> 16) & 0xFF));
    SPI1_ReadWriteByte((uint8_t)((sector_addr >> 8) & 0xFF));
    SPI1_ReadWriteByte((uint8_t)(sector_addr & 0xFF));
    W25Q_CS_HIGH();

    W25Qxx_WaitBusy();
}

/* ==================== Write Page (max 256 bytes) ==================== */
void W25Qxx_WritePage(const uint8_t *buffer, uint32_t write_addr, uint16_t length) {
    if(length > W25Q32_PAGE_SIZE) length = W25Q32_PAGE_SIZE;

    W25Qxx_WriteEnable();

    W25Q_CS_LOW();
    SPI1_ReadWriteByte(W25X_PAGE_PROGRAM);
    SPI1_ReadWriteByte((uint8_t)((write_addr >> 16) & 0xFF));
    SPI1_ReadWriteByte((uint8_t)((write_addr >> 8) & 0xFF));
    SPI1_ReadWriteByte((uint8_t)(write_addr & 0xFF));

    for(uint16_t i = 0; i < length; i++) {
        SPI1_ReadWriteByte(buffer[i]);
    }
    W25Q_CS_HIGH();

    W25Qxx_WaitBusy();
}

/* ==================== Continuous Write Data ==================== */
void W25Qxx_WriteData(const uint8_t *buffer, uint32_t write_addr, uint32_t length) {
    uint32_t page_offset = write_addr % W25Q32_PAGE_SIZE;
    uint32_t page_space  = W25Q32_PAGE_SIZE - page_offset;
    uint16_t chunk;

    while(length > 0) {
        chunk = (uint16_t)((length <= page_space) ? length : page_space);
        W25Qxx_WritePage(buffer, write_addr, chunk);
        buffer     += chunk;
        write_addr += chunk;
        length     -= chunk;
        page_space  = W25Q32_PAGE_SIZE;
    }
}

/* ==================== Read Data ==================== */
void W25Qxx_ReadData(uint8_t *buffer, uint32_t read_addr, uint32_t length) {
    W25Q_CS_LOW();
    SPI1_ReadWriteByte(W25X_READ_DATA);
    SPI1_ReadWriteByte((uint8_t)((read_addr >> 16) & 0xFF));
    SPI1_ReadWriteByte((uint8_t)((read_addr >> 8) & 0xFF));
    SPI1_ReadWriteByte((uint8_t)(read_addr & 0xFF));

    for(uint32_t i = 0; i < length; i++) {
        buffer[i] = SPI1_ReadWriteByte(0xFF);
    }
    W25Q_CS_HIGH();
}

/* ==================== Self Test ==================== */
uint8_t W25Qxx_Test(void) {
    uint32_t id = W25Qxx_ReadID();
    printf("[W25Q32] JEDEC ID: 0x%06X\r\n", (unsigned int)id);
    return ((id & 0xFFFF00) == 0xEF4000);
}
