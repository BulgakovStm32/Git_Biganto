/*
 *
 *
 *  Created on:
 *      Author:
 */
//*******************************************************************************************
//*******************************************************************************************

#include "uart_ST.h"

//*******************************************************************************************
//*******************************************************************************************
//Инициализация портов.
static void _usart_GpioInit(USART_TypeDef *usart){

	if(usart == USART1)
	{
		//PA9(U1TX), PA10(U1RX).
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_CNF10);
		GPIOA->CRH |=   GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | //PA9(U1TX)  - выход, альтернативный режим push-pull.
					    GPIO_CRH_CNF10_0;				   //PA10(U1RX) - Floating input.
	}
	else if(usart == USART2)
	{
		//PA2(U2TX), PA3(U2RX).
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_CNF3);
		GPIOA->CRL |=   GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2 | //PA2(U2TX) - выход, альтернативный режим push-pull.
				  	    GPIO_CRL_CNF3_0;				   //PA3(U2RX) - Floating input.
	}
	else if(usart == USART3)
	{
		//PB10(U3TX), PB11(U3RX).
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_CNF11);
		GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10 | //PB10(U3TX) - выход, альтернативный режим push-pull.
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

	//Сброс колцевого буфера.
	RING_BUFF_FlushRxBuf();

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
			RING_BUFF_PutByteToRxBuff(usart->DR);
		}
		//--------------------
		//Если байт битый, то пропускаем его и
		//очищаем приемный буфер для запуска приема нового пакета.
		else
		{
			_usart_ClearErrFlag(usart);
			RING_BUFF_FlushRxBuf();//очистить приемный буфер
		}
		//--------------------
	}
	/*------------------------------------------------------------------------*/
	/*!< Overrun error -------------------------------------------------------*/
	if(usart->SR & USART_SR_ORE)
	{
		_usart_ClearErrFlag(usart);
		RING_BUFF_FlushRxBuf();//очистить приемный буфер
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
void USART_DMA_Init(USART_TypeDef *usart, uint32_t baudRate){

	//--------------------
	//Инициализация USARTx.
	_usart_GpioInit(usart);				//Инициализация портов.
	_usart_ClockEnable(usart);			//вкл. тактирования модуля USART.
	_usart_SetBaudRate(usart, baudRate);//установка скорости работы.

	usart->CR1 &= ~USART_CR1_M; 	//8 бит данных.
	usart->CR2 &= ~USART_CR2_STOP;  //1 стоп-бит.
	usart->CR1 |= USART_CR1_RE    | //Включение RX USART1.
				  USART_CR1_TE    | //Включение TX USART1.
				  USART_CR1_RXNEIE| //Включение прерывания от приемника USART1.
				  USART_CR1_UE;     //Включение USART1.

	//Сброс колцевого буфера.
	RING_BUFF_FlushRxBuf();

	//Приоритет прерывания.
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
	//Разрешаем прерывание.
	NVIC_EnableIRQ(USART2_IRQn);
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
void USART_DMA_Disable(USART_TypeDef *usart){

	usart->CR1 &= ~USART_CR1_UE; //Откл. USART
}
//**********************************************************
void USART_DMA_Enable(USART_TypeDef *usart){

	usart->CR1 |= USART_CR1_UE; //Вкл. USART
}
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
	dma->CCR |= DMA_CCR_MINC;    //использовать инкремент указателя

//	DMA1_Channel4->CCR |= DMA_CCR_TCIE;  //Разрешить прерывание по завершении обмена
//	NVIC_EnableIRQ (DMA1_Channel4_IRQn); //Разрешить прерывания от DMA канал №4
}
//**********************************************************
//Старт обмена в канале "память-DMA-USART1"                                                                   
void DMAxChxStartTx(DMA_Channel_TypeDef *dma, uint8_t *buf, uint32_t size){
  
	dma->CCR  &= ~DMA_CCR_EN;  //запретить работу канала
	dma->CMAR  = (uint32_t)buf;//адрес буфера в памяти
	dma->CNDTR = size;         //загрузить количество данных для обмена
	dma->CCR  |= DMA_CCR_EN;   //разрешить работу канала
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




