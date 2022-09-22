/*
 *
 *
 *  Created on:
 *      Author:
 */

#ifndef _uart_ST_H
#define _uart_ST_H
//*******************************************************************************************
//*******************************************************************************************

//#include "stm32f10x.h"
#include "main.h"

//*******************************************************************************************
#define APB1CLK 	36000000UL
#define APB2CLK 	72000000UL

//********************************************************
//Указатели обработчиков событий
typedef struct{
	//--------
	uint8_t (*ReceiveByte)        (uint8_t byte);//Обработчик приема байта.
    void    (*ReceiveByteBroken)  (void);   	 //Обработчик битого байта.
	void    (*BufTransferComplete)(void);        //Окончание передачи буфера.
	//--------
}UxHandlers_t;
//*******************************************************************************************
//*******************************************************************************************
//Аппатартный уровень.
void 		  Uart1Init  (uint16_t usartBrr);
void 		  Uart1ManagingRx(uint8_t rxState);
void 		  Uart1StarTx(uint8_t *TxBuf, uint8_t size);
void          Uart1DmaStarTx(uint8_t *TxBuf, uint32_t size);
UxHandlers_t* Uart1Handler(void);

//********************************************************
void USART_DMA_Init(USART_TypeDef *usart, uint32_t baudRate);

//Работа с DMA.
void DMAxChxInitForTx(DMA_Channel_TypeDef *dma, uint32_t *perifAddr);
void DMAxChxStartTx(DMA_Channel_TypeDef *dma, uint8_t *buf, uint32_t size);
//*******************************************************************************************
//*******************************************************************************************
#endif 

















