// /********************************** (C) COPYRIGHT *******************************
//  * File Name          : usb_endp.c
//  * Author             : WCH
//  * Version            : V1.0.0
//  * Date               : 2021/08/08
//  * Description        : Endpoint routines
// *********************************************************************************
// * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
// * Attention: This software (modified or not) and binary are used for 
// * microcontroller manufactured by Nanjing Qinheng Microelectronics.
// *******************************************************************************/ 
// #include "usb_lib.h"
// #include "usb_desc.h"
// #include "usb_mem.h"
// #include "hw_config.h"
// #include "usb_istr.h"
// #include "usb_pwr.h"
// #include "usb_prop.h"
// #include "UART.h"

// uint8_t USBD_Endp3_Busy;
// uint16_t USB_Rx_Cnt=0; 

// /*********************************************************************
//  * @fn      EP2_IN_Callback
//  *
//  * @brief  Endpoint 1 IN.
//  *
//  * @return  none
//  */
// void EP1_IN_Callback (void)
// { 
	
// }



// /*********************************************************************
//  * @fn      EP2_OUT_Callback
//  *
//  * @brief  Endpoint 2 OUT.
//  *
//  * @return  none
//  */
// void EP2_OUT_Callback (void)
// { 
// 	uint32_t len;
//     len = GetEPRxCount( EP2_OUT & 0x7F );
//     PMAToUserBufferCopy( &UART2_Tx_Buf[ ( Uart.Tx_LoadNum * DEF_USB_FS_PACK_LEN ) ], GetEPRxAddr( EP2_OUT & 0x7F ), len );
//     Uart.Tx_PackLen[ Uart.Tx_LoadNum ] = len;
//     Uart.Tx_LoadNum++;
//     if( Uart.Tx_LoadNum >= DEF_UARTx_TX_BUF_NUM_MAX )
//     {
//         Uart.Tx_LoadNum = 0x00;
//     }
//     Uart.Tx_RemainNum++;

// 	if( Uart.Tx_RemainNum >= ( DEF_UARTx_TX_BUF_NUM_MAX - 2 ) )
//     {
//         Uart.USB_Down_StopFlag = 0x01;
//     }
//     else
//     {
//         SetEPRxValid( ENDP2 );
//     }
// }
// /*********************************************************************
//  * @fn      EP3_IN_Callback
//  *
//  * @brief  Endpoint 3 IN.
//  *
//  * @return  none
//  */
// void EP3_IN_Callback (void)
// { 
// 	USBD_Endp3_Busy = 0;
// 	Uart.USB_Up_IngFlag = 0x00;
// }

// /*********************************************************************
//  * @fn      USBD_ENDPx_DataUp
//  *
//  * @brief  USBD ENDPx DataUp Function
//  * 
//  * @param   endp - endpoint num.
//  *          *pbuf - A pointer points to data.
//  *          len - data length to transmit.
//  * 
//  * @return  data up status.
//  */
// uint8_t USBD_ENDPx_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len )
// {
// 	if( endp == ENDP3 )
// 	{
// 		if (USBD_Endp3_Busy)
// 		{
// 			return USB_ERROR;
// 		}
// 		USB_SIL_Write( EP3_IN, pbuf, len );
// 		USBD_Endp3_Busy = 1;
// 		SetEPTxStatus( ENDP3, EP_TX_VALID );
// 	}
// 	else
// 	{
// 		return USB_ERROR;
// 	}
// 	return USB_SUCCESS;
// }
/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_endp.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Endpoint routines
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_endp.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Endpoint routines
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "UART.h"
#include <string.h>  // ADD THIS

#include <stdarg.h>
#include <stdio.h>

uint8_t USBD_Endp3_Busy = 0;
uint16_t USB_Rx_Cnt = 0; 

/* USB Command Buffer */
#define USB_CMD_BUFFER_SIZE  512

static uint8_t usb_cmd_buffer[USB_CMD_BUFFER_SIZE];
static volatile uint16_t usb_cmd_in = 0;
static volatile uint16_t usb_cmd_out = 0;

/* Function prototypes */
uint8_t USB_Data_Available(void);
uint8_t USB_Receive_Data(void);
void USB_Data_Send(uint8_t* data, uint16_t len);
void USB_Printf(const char *format, ...);

/*********************************************************************
 * @fn      EP1_IN_Callback
 * @brief   Endpoint 1 IN.
 * @return  none
 */
void EP1_IN_Callback (void)
{ 
}

/*********************************************************************
 * @fn      EP2_OUT_Callback
 * @brief   Endpoint 2 OUT. (Receives data from PC Serial Monitor)
 * @return  none
 */
void EP2_OUT_Callback (void)
{ 
    uint32_t len;
    uint32_t i;
    uint8_t temp_buffer[64];
    
    len = GetEPRxCount( EP2_OUT & 0x7F );
    if(len > 64) len = 64;
    
    PMAToUserBufferCopy(temp_buffer, GetEPRxAddr( EP2_OUT & 0x7F ), len);
    
    // Store in command buffer for direct command processing
    for(i = 0; i < len; i++)
    {
        uint16_t next_in = (usb_cmd_in + 1) % USB_CMD_BUFFER_SIZE;
        if(next_in != usb_cmd_out)
        {
            usb_cmd_buffer[usb_cmd_in] = temp_buffer[i];
            usb_cmd_in = next_in;
        }
    }
    
    SetEPRxValid( ENDP2 );
}

/*********************************************************************
 * @fn      EP3_IN_Callback
 * @brief   Endpoint 3 IN. (Packet sent to PC)
 * @return  none
 */
void EP3_IN_Callback (void)
{ 
    USBD_Endp3_Busy = 0;
    Uart.USB_Up_IngFlag = 0x00;
}

/*********************************************************************
 * @fn      USBD_ENDPx_DataUp
 * @brief   USBD ENDPx DataUp Function
 * @param   endp - endpoint num.
 *          *pbuf - A pointer points to data.
 *          len - data length to transmit.
 * @return  data up status.
 */
uint8_t USBD_ENDPx_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len )
{
    if( endp == ENDP3 )
    {
        if (USBD_Endp3_Busy)
        {
            return USB_ERROR;
        }
        USB_SIL_Write( EP3_IN, pbuf, len );
        USBD_Endp3_Busy = 1;
        SetEPTxStatus( ENDP3, EP_TX_VALID );
    }
    else
    {
        return USB_ERROR;
    }
    return USB_SUCCESS;
}

/*********************************************************************
 * @fn      USB_Data_Available
 * @brief   Check if USB receive data is available from PC
 * @return  1 if data available, 0 if no data
 */
uint8_t USB_Data_Available(void)
{
    return (usb_cmd_in != usb_cmd_out);
}

/*********************************************************************
 * @fn      USB_Receive_Data
 * @brief   Get one byte from USB receive buffer
 * @return  Received byte
 */
uint8_t USB_Receive_Data(void)
{
    uint8_t data = 0;
    if(usb_cmd_in != usb_cmd_out)
    {
        data = usb_cmd_buffer[usb_cmd_out];
        usb_cmd_out = (usb_cmd_out + 1) % USB_CMD_BUFFER_SIZE;
    }
    return data;
}

/*********************************************************************
 * @fn      USB_Data_Send
 * @brief   Send data to PC via USB CDC
 * @param   data - pointer to data buffer
 * @param   len - length of data to send
 */
void USB_Data_Send(uint8_t* data, uint16_t len)
{
    if(bDeviceState != CONFIGURED || data == NULL || len == 0) return;
    
    while(len > 0)
    {
        uint16_t chunk = (len > DEF_USBD_MAX_PACK_SIZE) ? DEF_USBD_MAX_PACK_SIZE : len;
        uint16_t timeout_ms = 100;
        
        while(USBD_Endp3_Busy && timeout_ms > 0)
        {
            Delay_Ms(1);
            timeout_ms--;
        }
        if(USBD_Endp3_Busy) break;
        
        USBD_ENDPx_DataUp(ENDP3, data, chunk);
        data += chunk;
        len -= chunk;
    }

    /* Wait for the final packet to complete transmission */
    uint16_t final_timeout = 50;
    while(USBD_Endp3_Busy && final_timeout > 0)
    {
        Delay_Ms(1);
        final_timeout--;
    }
}

/*********************************************************************
 * @fn      USB_Printf
 * @brief   Formatted print to USB CDC Serial Monitor
 */
void USB_Printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if(len > 0)
    {
        USB_Data_Send((uint8_t*)buffer, (uint16_t)len);
    }
}