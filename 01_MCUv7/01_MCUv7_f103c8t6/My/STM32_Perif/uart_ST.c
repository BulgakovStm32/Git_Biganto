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
  DMA1Ch4InitForTx(USART1);  
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

	DMA1Ch4StartTx(TxBuf, size);
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
void USART1_IRQHandler(void){

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
//Инициализация канала 4 DMA1 передачи данных через USART1.   
void DMA1Ch4InitForTx(USART_TypeDef *usart){
  
	//Задать адрес источника и приемника и количество данных для обмена
	DMA1_Channel4->CPAR = (uint32_t)&usart->DR;//адрес регистра перефирии
	//----------------- Манипуляции с регистром конфигурации  ----------------
	//Следующие действия можно обьединить в одну команду (разбито для наглядности)
	//После сброса значение CCR = 0.
	//DMA1_Channel4->CCR  =  0;            //предочистка регистра конфигурации
	//DMA1_Channel4->CCR &= ~DMA_CCR4_CIRC;//выключить циклический режим
	DMA1_Channel4->CCR |= DMA_CCR_DIR;     //направление: чтение из памяти
	//Настроить работу с переферийным устройством
	//DMA1_Channel4->CCR &= ~DMA_CCR4_PSIZE;//размерность данных 8 бит
	//DMA1_Channel4->CCR &= ~DMA_CCR4_PINC; //неиспользовать инкремент указателя
	//Настроить работу с памятью
	//DMA1_Channel4->CCR &= ~DMA_CCR4_MSIZE;//размерность данных 8 бит
	DMA1_Channel4->CCR |= DMA_CCR_MINC;    //использовать инкремент указателя

//	DMA1_Channel4->CCR |= DMA_CCR_TCIE;  //Разрешить прерывание по завершении обмена
//	NVIC_EnableIRQ (DMA1_Channel4_IRQn); //Разрешить прерывания от DMA канал №4
}
//**********************************************************
//Старт обмена в канале "память-DMA-USART1"                                                                   
void DMA1Ch4StartTx(uint8_t *buf, uint32_t size){
  
	DMA1_Channel4->CCR  &= ~DMA_CCR_EN;  //запретить работу канала
	DMA1_Channel4->CMAR  = (uint32_t)buf;//адрес буфера в памяти
	DMA1_Channel4->CNDTR = size;         //загрузить количество данных для обмена
	DMA1_Channel4->CCR  |= DMA_CCR_EN;   //разрешить работу канала
}
//**********************************************************
//Прерываение от DMA1_Ch4.
void DMA1_Channel4_IRQHandler(void){
  
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




