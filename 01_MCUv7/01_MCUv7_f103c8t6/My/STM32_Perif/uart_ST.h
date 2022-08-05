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
#define USART_BAUD_RATE	9600//256000UL //115200UL
//UART3 тактируется от APB1(36МГц).
//UART1 тактируется от APB2(72МГц).
#define APB1CLK 	32000000UL
#define APB2CLK 	72000000UL
#define USART1_BRR	((APB2CLK + USART_BAUD_RATE / 2) / USART_BAUD_RATE)

//#define RS485DirectTX()		(GPIOC->BSRR = GPIO_BSRR_BS14)//Управление микросхемой драйвера RS485.
//#define RS485DirectRX()  		(GPIOC->BSRR = GPIO_BSRR_BR14)
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
//------------------------
//Работа с DMA. Не отлажено!!!
void DMA1Ch4InitForTx(USART_TypeDef *usart);
void DMA1Ch4StartTx  (uint8_t *buf, uint32_t size);

//void    DMA1Ch5InitForRX (void);
//void    StartDMA1Ch5ForRX(uint8_t *BuffAdr, uint16_t LengthBufer);
//uint8_t GetStateDMA1Ch4  (void);
//*******************************************************************************************
//*******************************************************************************************
#endif 

















