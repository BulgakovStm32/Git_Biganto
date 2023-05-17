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
void USARTx_Init(USART_TypeDef *usart, uint32_t baudRate); //инициализация usart`a
void USARTx_SendBuff(USART_TypeDef *usart, uint8_t *buff, uint32_t size);

void USARTx_IT_Handler(USART_TypeDef *usart);//Обработчик прерываний USART.

//********************************************************
//Кольцевой приемный буфер
void 	RING_BUFF_FlushRxBuf(void); 			//очистить приемный буфер
uint8_t RING_BUFF_GetRxCount(void); 			//взять число символов в приемном буфере
uint8_t	RING_BUFF_GetByte(void); 				//прочитать символ из приемного буфера
void 	RING_BUFF_PutByteToRxBuff(uint8_t sym);	//помещает символ в приемный буфер
void	RING_BUFF_CopyRxBuff(char *buff);   	//копировать принятые данные в сторонний буфер.

//********************************************************
//Работа с DMA.
void USART_DMA_Init(USART_TypeDef *usart, uint32_t baudRate);

void DMAxChxInitForTx(DMA_Channel_TypeDef *dma, uint32_t *perifAddr);
void DMAxChxStartTx(DMA_Channel_TypeDef *dma, uint8_t *buf, uint32_t size);

void DMA1_Chx_IT_Handler(DMA_Channel_TypeDef *dma);
//*******************************************************************************************
//*******************************************************************************************
#endif 

















