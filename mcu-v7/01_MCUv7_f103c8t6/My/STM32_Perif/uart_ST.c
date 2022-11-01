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
typedef struct{
	//--------------------
	volatile uint16_t CounterByte;//Счетчик переданных байт. 
	volatile uint8_t  BufSize;    //Количество передаваемых байт.
	volatile uint8_t *BufPtr;     //Указатель на передающий буфер.
	//--------------------
}UxWorkReg_t;
//----------------------------------------
static UxWorkReg_t	U1WorkReg;
static UxHandlers_t	U1Handlers;
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



//*******************************************************************************************
//*******************************************************************************************

















void Uart1Init(uint16_t usartBrr){
  
 	//Инициализация портов. PA9(U1TX), PA10(U1RX).
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_CNF10);	
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;//PA9(U1TX)  - выход, альтернативный режим push-pull.																							 //PA9(U1TX) - тактирование 50МГц.
	GPIOA->CRH |= GPIO_CRH_CNF10_0;				   //PA10(U1RX) - Floating input.
  //--------------------
  //Инициализация USART1.
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;//Включение тактирование USART1.
	
  USART1->BRR  = usartBrr;  	   //Set baudrate
  USART1->CR1 &= ~USART_CR1_M; 	   //8 бит данных.
  USART1->CR2 &= ~USART_CR2_STOP;  //1 стоп-бит.
  USART1->CR1 =  USART_CR1_RE    | //Включение RX USART1.	
                 USART_CR1_TE    | //Включение TX USART1.
                 USART_CR1_RXNEIE| //Включение прерывания от приемника USART1.		
                 USART_CR1_UE;     //Включение USART1.
  USART1->CR3 |= USART_CR3_DMAT;   //Подключение TX USART1 к DMA.          
  NVIC_SetPriority(USART1_IRQn, 15);//Приоритет прерывания USART1.
  NVIC_EnableIRQ(USART1_IRQn);      //Разрешаем прерывание от приемника USART1.
  //--------------------
  //Инициализация DMA. TX USART1 работает с DMA1 Ch4.
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;//Включить тактирование DMA1
  DMAxChxInitForTx(DMA1_Channel4, (uint32_t*)&USART1->DR);
}
//**********************************************************
//Включение/отключение приемника USART1. 
void Uart1ManagingRx(uint8_t rxState){

	if(rxState)USART1->CR1 |=  USART_CR1_RE;
	else 	   USART1->CR1 &= ~USART_CR1_RE;
}
//**********************************************************
//Запуск передачи буфера по прерыванию.
void Uart1StarTx(uint8_t *TxBuf, uint8_t size){

	U1WorkReg.CounterByte = 0;	  //Cброс счетчика байт.
	U1WorkReg.BufSize	  = size; //Количество байт на передачу.
	U1WorkReg.BufPtr	  = TxBuf;//Указатель на передающий буфер.
	//----------------
	USART1->CR1 &= ~USART_CR1_RE; //Отлючение RX USART1.
	USART1->DR   = *TxBuf;        //Передача первого байта.
	USART1->CR1 |= USART_CR1_TE  |//Включение TX USART1.
                   USART_CR1_TCIE;//Включение прерывания от TX USART1.
}
//**********************************************************
void Uart1DmaStarTx(uint8_t *TxBuf, uint32_t size){

	//DMA1Ch4StartTx(TxBuf, size);
	DMAxChxStartTx(DMA1_Channel7, TxBuf, size);
}
//**********************************************************
//Указатели обработчиков событий
UxHandlers_t* Uart1Handler(void){
	
	return &U1Handlers;
}
//**********************************************************
//сброс флагов ошибки 
static void Usart1ClrErrFlag(void){

	USART1->SR;//Такая последовательность сбрасывает флаги ошибки 
	USART1->DR;//
}
//**********************************************************
//Прерывание от USART1.
void USART1_Handler(void){

	//--------------------
	/*!< Read Data Register Not Empty */
	if(USART1->SR & USART_SR_RXNE)
		{
		  //проверяем нет ли ошибок при приеме байта.
		  if((USART1->SR & (USART_SR_NE | USART_SR_FE | USART_SR_PE | USART_SR_ORE)) == 0)
			{
			  //Принимаем запрос от МВ.
			  //U1Handlers.ReceiveByte(USART1->DR);
			}
		  //--------------------
		  //Если байт битый, то пропускаем его и
		  //очищаем приемный буфер для запуска приема нового пакета.
		  else
			{
			  Usart1ClrErrFlag();
			  //U1Handlers.ReceiveByteBroken();
			}
		  //--------------------
		}
	//--------------------
	/*!< Overrun error */
	if(USART1->SR & USART_SR_ORE)
		{
			Usart1ClrErrFlag();
			//U1Handlers.ReceiveByteBroken();
		}
	//--------------------
	/*!< Transmission Complete */
	if(USART1->SR & USART_SR_TC)
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
			USART1->CR1 &= ~USART_CR1_TCIE;  //Отлючение прерывания по окончанию передачи		
			//---------
		}
	//--------------------
	/*!< Transmit Data Register Empty */
	if(USART1->SR & USART_SR_TXE)
		{
		  //USART1->SR &= ~USART_SR_TXE; //Сброс флага прерывания.
		  //---------
		}
	//--------------------
}
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
			//Принимаем запрос от МВ.
			//U1Handlers.ReceiveByte(USART1->DR);
		}
		//--------------------
		//Если байт битый, то пропускаем его и
		//очищаем приемный буфер для запуска приема нового пакета.
		else
		{
			Usart1ClrErrFlag();
			//U1Handlers.ReceiveByteBroken();
		}
		//--------------------
	}
	/*------------------------------------------------------------------------*/
	/*!< Overrun error -------------------------------------------------------*/
	if(usart->SR & USART_SR_ORE)
	{
		Usart1ClrErrFlag();
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
void USART_DMA_Init(USART_TypeDef *usart, uint32_t baudRate){

	//Инициализация портов.
	_usart_GpioInit(usart);

	//Включение тактирования USART и установка скорости работы.
	if(usart == USART1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;			   //Включение тактирование USART1.
		usart->BRR    = (uint32_t)((APB2CLK + baudRate / 2) / baudRate);//Set baudrate
	}
	else if(usart == USART2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;			   //Включение тактирование USART2.
		usart->BRR    = (uint32_t)((APB1CLK + baudRate / 2) / baudRate);//Set baudrate
	}
	else if(usart == USART3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;			   //Включение тактирование USART3.
		usart->BRR    = (uint32_t)((APB1CLK + baudRate / 2) / baudRate);//Set baudrate
	}
	else return;
	//--------------------
	//Инициализация USARTx.
	usart->CR1 &= ~USART_CR1_M; 	//8 бит данных.
	usart->CR2 &= ~USART_CR2_STOP;  //1 стоп-бит.
	usart->CR1 |= USART_CR1_RE    | //Включение RX USART1.
				  USART_CR1_TE    | //Включение TX USART1.
				  //USART_CR1_RXNEIE| //Включение прерывания от приемника USART1.
				  USART_CR1_UE;     //Включение USART1.

//	NVIC_SetPriority(USART1_IRQn, 15);//Приоритет прерывания USART1.
//	NVIC_EnableIRQ(USART1_IRQn);      //Разрешаем прерывание от приемника USART1.
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




