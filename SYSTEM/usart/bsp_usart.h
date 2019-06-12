#ifndef __BSP_USART_H
#define __BSP_USART_H
#include "stm32f10x.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define  MAX_RX_CNT 200
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern union Data{
    s16  InRxData[50];
    u8 	 ChRxData[100];
}TXData,RXData;

extern u8 USART3_Rx_Buff[MAX_RX_CNT];
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void USART2_Config(uint32_t BaudRate);
void USART3_Config(uint32_t BaudRate);
void u3_printf(const char *format,...);
uint8_t USART2_DMA_TX(const uint8_t *TxBuff , uint8_t Byte_Cnt);
uint8_t USART3_DMA_TX(const uint8_t *TxBuff , uint8_t Byte_Cnt);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static void USART2_DMA_Config(void);
static void USART2_NVIC_Config(void);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif

