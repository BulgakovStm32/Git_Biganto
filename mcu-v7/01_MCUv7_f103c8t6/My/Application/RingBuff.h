/*
 * RingBuff.h
 *
 *  Created on: 2 нояб. 2022 г.
 *      Author: belyaev
 */

#ifndef APPLICATION_RINGBUFF_H_
#define APPLICATION_RINGBUFF_H_
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define RING_BUFF_SIZE 			32 //размер кольцевых буферов
#define RING_BUFF_SIZE_MASK		(RING_BUFF_SIZE - 1)

//**********************************
//Флаги питания
typedef struct{
	uint32_t 	f_receivedCR   	 : 1; //принят символ '\r'
	uint32_t	dummy	     	 : 31;
}RingBuffFlag_t;
//**********************************

typedef struct{
	//Приемный буфер.
	uint8_t rxBuf[RING_BUFF_SIZE];
	uint8_t rxBufTail;
	uint8_t rxBufHead;
	uint8_t rxCount;
	//Передающий буфер.
	uint8_t txBuf[RING_BUFF_SIZE];
	uint8_t txBufTail;
	uint8_t txBufHead;
	uint8_t txCount;
}RingBuff_t;

//*******************************************************************************************
//*******************************************************************************************
void 	 RING_BUFF_FlushRxBuf(void); 			 //очистить приемный буфер
uint32_t RING_BUFF_GetRxCount(void); 			 //взять число символов в приемном буфере
uint8_t  RING_BUFF_GetByteFromRxBuff(void);		 //прочитать символ из приемного буфера
void 	 RING_BUFF_PutByteToRxBuff(uint8_t byte);//помещает символ в приемный буфер
RingBuffFlag_t* RING_BUFF_Flags(void);
void	 RING_BUFF_CopyRxBuff(uint8_t *buff);    //копировать принятые данные в сторонний буфер.

//*******************************************************************************************
//*******************************************************************************************
#endif /* APPLICATION_RINGBUFF_H_ */













