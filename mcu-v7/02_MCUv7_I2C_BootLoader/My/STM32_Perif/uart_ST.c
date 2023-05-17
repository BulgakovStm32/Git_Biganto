/*
 *
 *
 *  Created on:
 *      Author:
 */
#include "uart_ST.h"
//*******************************************************************************************
//*******************************************************************************************
//Рабочие регистры.
//typedef struct{
//	//--------------------
//	volatile uint16_t CounterByte;//Счетчик переданных байт.
//	volatile uint8_t  BufSize;    //Количество передаваемых байт.
//	volatile uint8_t *BufPtr;     //Указатель на передающий буфер.
//	//--------------------
//}UxWorkReg_t;
//----------------------------------------
//static UxWorkReg_t	U1WorkReg;
//static UxHandlers_t	U1Handlers;
//*******************************************************************************************
//*******************************************************************************************
//Инициализация портов.
static void _usart_GpioInit(USART_TypeDef *usart){

	if(usart == USART1)
	{
		//PA9(U1TX), PA10(U1RX).
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_CNF10);
		GPIOA->CRH |=   GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | //PA9(U1TX)  - выход, альтернативный режим push-pull.																							 //PA9(U1TX) - тактирование 50МГц.
					    GPIO_CRH_CNF10_0;				   //PA10(U1RX) - Floating input.
	}
	else if(usart == USART2)
	{
		//PA2(U2TX), PA3(U2RX).
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_CNF3);
		GPIOA->CRL |=   GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2 | //PA2(U2TX) - выход, альтернативный режим push-pull.																							 //PA9(U1TX) - тактирование 50МГц.
				  	    GPIO_CRL_CNF3_0;				   //PA3(U2RX) - Floating input.
	}
	else if(usart == USART3)
	{
		//PB10(U3TX), PB11(U3RX).
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_CNF11);
		GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10 | //PB10(U3TX) - выход, альтернативный режим push-pull.																							 //PA9(U1TX) - тактирование 50МГц.
					  GPIO_CRH_CNF11_0;				       //PB11(U3RX) - Floating input.
	}
}
//**********************************************************
static void _usart_ClockEnable(USART_TypeDef *usart){

	//Включение тактирования USART.
		 if(usart == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	else if(usart == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	else if(usart == USART3) RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
}
//**********************************************************
static void _usart_SetBaudRate(USART_TypeDef *usart, uint32_t baudRate){

	//установка скорости работы.
	if(usart == USART1) usart->BRR = (uint32_t)((APB2CLK + baudRate / 2) / baudRate);//Set baudrate
	else 				usart->BRR = (uint32_t)((APB1CLK + baudRate / 2) / baudRate);//Set baudrate
}
//**********************************************************
//сброс флагов ошибки
static void _usart_ClearErrFlag(USART_TypeDef *usart){

	(void)usart->SR;//Такая последовательность сбрасывает флаги ошибки
	(void)usart->DR;//
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void USARTx_Init(USART_TypeDef *usart, uint32_t baudRate){

	_usart_GpioInit(usart);				//Инициализация портов.
	_usart_ClockEnable(usart);			//вкл. тактирования модуля USART.
	_usart_SetBaudRate(usart, baudRate);//установка скорости работы.

	usart->CR2 &= ~USART_CR2_STOP;  //1 стоп-бит.
	usart->CR1 &= ~USART_CR1_M; 	//8 бит данных.
	usart->CR1 |= USART_CR1_RE    | //Включение RX USART1.
				  USART_CR1_TE    | //Включение TX USART1.
				  //USART_CR1_IDLEIE | //IDLE Interrupt Enable
				  USART_CR1_RXNEIE | //RXNE Interrupt Enable(RXNE-Read data register not empty)
				  //USART_CR1_TCIE   | //Transmission Complete Interrupt Enable
				  //USART_CR1_TXEIE  | //TXE interrupt enable(TXE-Transmit data register empty)
				  USART_CR1_UE;     //Включение USART1.

	RING_BUFF_FlushRxBuf();//Сброс колцевого буфера.

	//Приоритет прерывания.
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	//Разрешаем прерывание.
	NVIC_EnableIRQ(USART2_IRQn);
}
//**********************************************************
void USARTx_SendBuff(USART_TypeDef *usart, uint8_t *buff, uint32_t size){

	size++;
	while(size)
	{
		while((usart->SR & USART_SR_TXE) == 0){}
		usart->DR = *buff;
		buff++;
		size--;
	}
}
//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Обработчик прерываний USART.
void USARTx_IT_Handler(USART_TypeDef *usart){

	/*------------------------------------------------------------------------*/
	/*!< Read Data Register Not Empty ----------------------------------------*/
	if(usart->SR & USART_SR_RXNE)
	{
		//проверяем нет ли ошибок при приеме байта.
		if((usart->SR & (USART_SR_NE | USART_SR_FE | USART_SR_PE | USART_SR_ORE)) == 0)
		{
			//Складываем приемнятый байт в буфер
			//U1Handlers.ReceiveByte(USART1->DR);
			RING_BUFF_PutByteToRxBuff(usart->DR);//помещает символ в приемный буфер
		}
		//--------------------
		//Если байт битый, то пропускаем его и
		//очищаем приемный буфер для запуска приема нового пакета.
		else
		{
			_usart_ClearErrFlag(usart);
			RING_BUFF_FlushRxBuf();//очистить приемный буфер
			//U1Handlers.ReceiveByteBroken();
		}
		//--------------------
	}
	/*------------------------------------------------------------------------*/
	/*!< Overrun error -------------------------------------------------------*/
	if(usart->SR & USART_SR_ORE)
	{
		_usart_ClearErrFlag(usart);
		RING_BUFF_FlushRxBuf();//очистить приемный буфер
		//U1Handlers.ReceiveByteBroken();
	}
	/*------------------------------------------------------------------------*/
	/*!< Transmission Complete -----------------------------------------------*/
	if(usart->SR & USART_SR_TC)
	{
	//      if(++U1WorkReg.CounterByte == U1WorkReg.BufSize)
	//        {
	//           //Запуск приема нового пакета.
	//					 RS485Direction(Rx);
	//           USART1->CR1 &= ~USART_CR1_TE;//Отлючение прерывания TX.
	//					 //USART1->CR1 &= ~USART_CR1_TCIE;//Отлючение прерывания по окончании передачи.
	//           USART1->CR1 |=  USART_CR1_RE;//Включение RX.
	//        }
	//      //Передача очередного байта.
	//      else USART1->DR = *(U1WorkReg.BufPtr + U1WorkReg.CounterByte);
		//---------
		//Работа с ДМА1 канал 4.
		//U1Handlers.BufTransferComplete();//Обработка окнчания предачи буфера.
		usart->CR1 &= ~USART_CR1_TCIE;  //Отлючение прерывания по окончанию передачи
	}
	/*------------------------------------------------------------------------*/
	/*!< Transmit Data Register Empty ----------------------------------------*/
	if(usart->SR & USART_SR_TXE)
	{
		//USART1->SR &= ~USART_SR_TXE; //Сброс флага прерывания.
	}
	/*------------------------------------------------------------------------*/
	/*------------------------------------------------------------------------*/
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
// Кольцевой буфер приема
#define SIZE_BUF_RX 32	//задаем размер кольцевых буферов

static volatile uint8_t	 ringRxBuf[SIZE_BUF_RX];
static volatile uint32_t ringRxBufTail = 0;
static volatile uint32_t ringRxBufHead = 0;
static volatile uint32_t ringRxCount   = 0;

//**********************************************************
void RING_BUFF_Init(void){

	ringRxBufTail = 0;
	ringRxBufHead = 0;
	ringRxCount   = 0;
}
//**********************************************************
//"очищает" приемный буфер
void RING_BUFF_FlushRxBuf(void){

	ringRxBufTail = 0;
	ringRxBufHead = 0;
	ringRxCount   = 0;
}
//**********************************************************
//возвращает колличество символов находящихся в приемном буфере
uint8_t RING_BUFF_GetRxCount(void){

  return ringRxCount;
}
//**********************************************************
//чтение буфера
uint8_t RING_BUFF_GetByte(void){

	uint8_t sym;
	//--------------------
	if(ringRxCount > 0)//если приемный буфер не пустой
	{
		sym = ringRxBuf[ringRxBufHead];		//прочитать из него символ
		ringRxCount--;                  	//уменьшить счетчик символов
		ringRxBufHead++;                	//инкрементировать индекс головы буфера
		//if(rxBufHead == SIZE_BUF_RX) rxBufHead = 0;
		ringRxBufHead &= (SIZE_BUF_RX - 1);	//проверка на преполнение
		return sym;                 		//вернуть прочитанный символ
	}
	return 0;
}
//**********************************************************
//прерывание по завершению приема
//ISR(USART_RXC_vect){
//
//    char data = UDR;
//
//    if (rxCount < SIZE_BUF_RX)//если в буфере еще есть место
//	{
//		usartRxBuf[rxBufTail] = data;//считать символ из UDR в буфер
//		rxBufTail++;                 //увеличить индекс хвоста приемного буфера
//		if (rxBufTail == SIZE_BUF_RX) rxBufTail = 0;
//		rxCount++;                   //увеличить счетчик принятых символов
//    }
//}

//помещает символ в приемный буфер
void RING_BUFF_PutByteToRxBuff(uint8_t sym){

    if(ringRxCount < SIZE_BUF_RX)//если в буфере еще есть место
	{
    	ringRxBuf[ringRxBufTail] = sym;//кладем символ в буфер
    	ringRxBufTail++;           	   //увеличить индекс хвоста приемного буфера
		//if(rxBufTail == SIZE_BUF_RX) rxBufTail = 0;
    	ringRxBufTail &= (SIZE_BUF_RX - 1);//Проверка на переполнение
    	ringRxCount++;                     //увеличить счетчик принятых символов
    }
}
//**********************************************************
//копировать принятые данные в сторонний буфер.
void RING_BUFF_CopyRxBuff(char *buff){

	while(RING_BUFF_GetRxCount())
	{
		*buff = RING_BUFF_GetByte();
		buff++;
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void USART_DMA_Init(USART_TypeDef *usart, uint32_t baudRate){

	USARTx_Init(usart, baudRate); //инициализация usart`a
	//--------------------
	//Инициализация DMA.
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;//Включить тактирование DMA1
	if(usart == USART1)
	{
		//USART1_TX -> DMA1_Ch4
		//USART1_RX -> DMA1_Ch5
		USART1->CR3 |= USART_CR3_DMAT;   //Подключение TX USART к DMA.
		DMAxChxInitForTx(DMA1_Channel4, (uint32_t*)&USART1->DR);
	}
	else if(usart == USART2)
	{
		//USART2_TX -> DMA1_Ch7
		//USART2_RX -> DMA1_Ch6
		USART2->CR3 |= USART_CR3_DMAT;   //Подключение TX USART к DMA.
		DMAxChxInitForTx(DMA1_Channel7, (uint32_t*)&USART2->DR);
	}
	else if(usart == USART3)
	{
		//USART3_TX -> DMA1_Ch2
		//USART3_RX -> DMA1_Ch3
		USART3->CR3 |= USART_CR3_DMAT;   //Подключение TX USART к DMA.
		DMAxChxInitForTx(DMA1_Channel2, (uint32_t*)&USART3->DR);
	}
}
//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
//Инициализация канала 4 DMA1 передачи данных через USART1.   
void DMAxChxInitForTx(DMA_Channel_TypeDef *dma, uint32_t *perifAddr){
  
	//Задать адрес источника и приемника и количество данных для обмена
	dma->CPAR = (uint32_t)perifAddr;//адрес регистра перефирии
	//----------------- Манипуляции с регистром конфигурации  ----------------
	//Следующие действия можно обьединить в одну команду (разбито для наглядности)
	//После сброса значение CCR = 0.
	//DMA1_Channel4->CCR  =  0;            //предочистка регистра конфигурации
	//DMA1_Channel4->CCR &= ~DMA_CCR4_CIRC;//выключить циклический режим
	dma->CCR |= DMA_CCR_DIR;     //направление: чтение из памяти
	//Настроить работу с переферийным устройством
	//DMA1_Channel4->CCR &= ~DMA_CCR4_PSIZE;//размерность данных 8 бит
	//DMA1_Channel4->CCR &= ~DMA_CCR4_PINC; //неиспользовать инкремент указателя
	//Настроить работу с памятью
	//DMA1_Channel4->CCR &= ~DMA_CCR4_MSIZE;//размерность данных 8 бит
	dma->CCR |= DMA_CCR_MINC;  //использовать инкремент указателя

	//прерывания.
//	dma->CCR |= DMA_CCR_TCIE;  //Разрешить прерывание по завершении обмена
//		 if(dma == DMA1_Channel2) NVIC_EnableIRQ(DMA1_Channel2_IRQn);
//	else if(dma == DMA1_Channel3) NVIC_EnableIRQ(DMA1_Channel3_IRQn);
//	else if(dma == DMA1_Channel4) NVIC_EnableIRQ(DMA1_Channel4_IRQn);
//	else if(dma == DMA1_Channel5) NVIC_EnableIRQ(DMA1_Channel5_IRQn);
//	else if(dma == DMA1_Channel6) NVIC_EnableIRQ(DMA1_Channel6_IRQn);
//	else if(dma == DMA1_Channel7) NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}
//**********************************************************
//Старт обмена в канале "память-DMA-USART1"                                                                   
void DMAxChxStartTx(DMA_Channel_TypeDef *dma, uint8_t *buf, uint32_t size){
  
	dma->CCR  &= ~DMA_CCR_EN;  //запретить работу канала
	dma->CMAR  = (uint32_t)buf;//адрес буфера в памяти
	dma->CNDTR = size;         //загрузить количество данных для обмена
	dma->CCR  |= DMA_CCR_EN;   //разрешить работу канала
}
//*******************************************************************************************
//*******************************************************************************************
//обработчик прерывания DMA.
void DMA1_Chx_IT_Handler(DMA_Channel_TypeDef *dma){


	//-------------------------
	//Если обмен завершен
	if(DMA1->ISR & DMA_ISR_TCIF4)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF4;    //сбросить флаг окончания обмена.
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;//запретить работу канала.


		//Это нужно что бы дождаться завершения передачи последнего байта.
		//USART1->CR1 |= USART_CR1_TCIE;	  //Включение прерывания по окончанию передачи USART1.
	}
	//-------------------------
	//Если передана половина буфера
	if(DMA1->ISR & DMA_ISR_HTIF4)
	{

	}
	//-------------------------
	//Если произошла ошибка при обмене
	if(DMA1->ISR & DMA_ISR_TEIF4)
	{

	}
	//-------------------------
}
//**********************************************************





//Прерываение от DMA1_Ch4.
void DMA1_Channel4_Handler(void){
  
	//-------------------------
	//Если обмен завершен
	if(DMA1->ISR & DMA_ISR_TCIF4)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF4;    //сбросить флаг окончания обмена.
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;//запретить работу канала.
		//Это нужно что бы дождаться завершения передачи последнего байта.
		USART1->CR1 |= USART_CR1_TCIE;	  //Включение прерывания по окончанию передачи USART1.
	}
//  //-------------------------
//  //Если передана половина буфера
//  if(DMA1->ISR & DMA_ISR_HTIF4)
//    {

//    }  
//  //-------------------------
//  //Если произошла ошибка при обмене
//  if(DMA1->ISR & DMA_ISR_TEIF4)
//    {

//    }   
//  //-------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************




