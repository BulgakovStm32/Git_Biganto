
#include "spi_ST.h"

//**************************************************************************************
static uint8_t Spi1StatusReg = 0;
static uint8_t Spi2StatusReg = 0;
//**************************************************************************************
//**************************************************************************************
//**************************************************************************************
//Инициализация SPI2. PB13(SPI2_SCK), PB14(SPI2_MISO), PB15(SPI2_MOSI).
void Spi1Init(void){
  
	if(Spi1StatusReg & SPI_INIT) return;
	Spi1StatusReg |= SPI_INIT;
	//--------------------
	//Включаем тактирование порта B.
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF7);
																
	GPIOA->CRL |= GPIO_CRL_CNF5_1;//PA5(SPI1_SCK) - выход, альтернативный режим push-pull.
  GPIOA->CRL |= GPIO_CRL_MODE5;	//PA6(SPI1_SCK) - тактирование 50МГц.
	
  GPIOA->CRL &= ~GPIO_CRL_MODE6;
  GPIOA->CRL &= ~GPIO_CRL_CNF6;
	GPIOA->CRL |=  GPIO_CRL_CNF6_1;//PA6(SPI1_MISO) - вход, 
  GPIOA->BSRR =  GPIO_BSRR_BS6;   
  
	GPIOA->CRL |= GPIO_CRL_CNF7_1;//PA7(SPI1_MOSI) - выход, альтернативный режим push-pull.
  GPIOA->CRL |= GPIO_CRL_MODE7;	//PA7(SPI1_MOSI) - тактирование 50МГц.
	//--------------------																
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;	//Включение тактирования SPI1.                                      /
  
	SPI1->CR1	 = 0;
	SPI1->CR1 |=(SPI_CR1_MSTR |     //режим "мастер".  
							 SPI_CR1_BR_0 |     //задаем скорость. Fpclk/32 = 36MHz/32 = 1.125MHz
							 //SPI_CR1_BR_1 |
							 //SPI_CR1_BR_2 |
               //SPI_CR1_LSBFIRST |//Младшим битом вперед
               //SPI_CR1_CPHA |      
               //SPI_CR1_CPOL |
               //SPI_CR1_DFF  |	 // 16 бит данных.
               SPI_CR1_SSI  |   //обеспечить высокий уровень программного NSS
               SPI_CR1_SSM  |   //разрешить программное формирование NSS
               SPI_CR1_SPE   ); //разрешить работу модуля SPI
	//--------------------	                  
//	SPI2->CR1    |= SPI_CR1_LSBFIRST;
//	SPI2->CR1    |= SPI_CR1_DFF;				// 16 бит данных.
//	SPI2->CR1    |= SPI_CR1_SSI;        //обеспечить высокий уровень программного NSS
//	SPI2->CR1    |= SPI_CR1_SSM;        //разрешить программное формирование NSS
//	SPI2->CR1    |= SPI_CR1_SPE;        //разрешить работу модуля SPI
	
// 	SPI2->CR2 |= SPI_CR2_TXEIE;        //разрешить прерывание по окончанию передачи               /
// 	SPI2->CR2 |= SPI_CR2_RXNEIE;       //разрешить прерывание, если принят байт данных
// 	SPI2->CR2 |= SPI_CR2_ERRIE;        //разрешить прерывание при возникновении ошибки
//	NVIC_EnableIRQ (SPI2_IRQn); 
}
//-----------------------------------------------------------------------------
//Передача данных(8 бит) в SPI1.
uint8_t	Spi1TxRxByte(uint8_t byte){

  volatile uint32_t spiWaitCount = 0;
  //--------------------
	//Если SPI не проинециализирован ,то выходим.
	if(!(Spi1StatusReg & SPI_INIT)) return 0;
	//Ожидание освобождения передающего буфера.
	while(!(SPI1->SR & SPI_SR_TXE))
		{
			if(++spiWaitCount > SPI_WAIT) return 0;
		}
		
	spiWaitCount = 0;
	SPI1->DR = byte;
		
	while(SPI1->SR & SPI_SR_BSY)
		{
			if(++spiWaitCount > SPI_WAIT) return 0;
		}
	//--------------------
	return (uint8_t)SPI1->DR;
} 
//**************************************************************************************
//**************************************************************************************
//**************************************************************************************
//Инициализация SPI2. PB13(SPI2_SCK), PB14(SPI2_MISO), PB15(SPI2_MOSI).
void Spi2Init(void){
  
	if(Spi2StatusReg & SPI_INIT) return;
	Spi2StatusReg |= SPI_INIT;
	//--------------------
	//Включаем тактирование порта B.
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
	GPIOB->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_CNF15);
																
	GPIOB->CRH |= GPIO_CRH_CNF13_1;	//PB13(SPI2_SCK) - выход, альтернативный режим push-pull.
  GPIOB->CRH |= GPIO_CRH_MODE13;	//PB13(SPI2_SCK) - тактирование 50МГц.
	
  GPIOB->CRH &= ~GPIO_CRH_MODE14;
  GPIOB->CRH &= ~GPIO_CRH_CNF14;
	GPIOB->CRH |=  GPIO_CRH_CNF14_1;//PB14(SPI2_MISO) - вход, 
  GPIOB->BSRR =  GPIO_BSRR_BS14;   
  
	GPIOB->CRH |= GPIO_CRH_CNF15_1;	//PB15(SPI2_MOSI) - выход, альтернативный режим push-pull.
  GPIOB->CRH |= GPIO_CRH_MODE15;	//PB15(SPI2_MOSI) - тактирование 50МГц.
	//--------------------																
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;	//Включение тактирования SPI2.                                      /
  
	SPI2->CR1	 = 0;
	SPI2->CR1 |=(SPI_CR1_MSTR |     //режим "мастер".  
							 //SPI_CR1_BR_0 |     //задаем скорость. Fpclk/32 = 36MHz/32 = 1.125MHz
							 //SPI_CR1_BR_1 |
							 SPI_CR1_BR_2 |
              //SPI_CR1_LSBFIRST |//Младшим битом вперед
               SPI_CR1_CPHA |      
               SPI_CR1_CPOL |
               //SPI_CR1_DFF  |	 // 16 бит данных.
               SPI_CR1_SSI  |   //обеспечить высокий уровень программного NSS
               SPI_CR1_SSM  |   //разрешить программное формирование NSS
               SPI_CR1_SPE   ); //разрешить работу модуля SPI
	//--------------------	                  
//	SPI2->CR1    |= SPI_CR1_LSBFIRST;
//	SPI2->CR1    |= SPI_CR1_DFF;				// 16 бит данных.
//	SPI2->CR1    |= SPI_CR1_SSI;        //обеспечить высокий уровень программного NSS
//	SPI2->CR1    |= SPI_CR1_SSM;        //разрешить программное формирование NSS
//	SPI2->CR1    |= SPI_CR1_SPE;        //разрешить работу модуля SPI
	
// 	SPI2->CR2 |= SPI_CR2_TXEIE;        //разрешить прерывание по окончанию передачи               /
// 	SPI2->CR2 |= SPI_CR2_RXNEIE;       //разрешить прерывание, если принят байт данных
// 	SPI2->CR2 |= SPI_CR2_ERRIE;        //разрешить прерывание при возникновении ошибки
//	NVIC_EnableIRQ (SPI2_IRQn); 
}
//-----------------------------------------------------------------------------
uint8_t Spi2GetStat(void){
  
  return (SPI2->SR & SPI_SR_BSY);
}
//-----------------------------------------------------------------------------
//Передача данных(8 бит) в SPI2.
uint8_t	Spi2TxRxByte(uint8_t byte){

  volatile uint32_t SpiWaitCount = 0;
  //--------------------
	//Если SPI не проинециализирован ,то выходим.
	if(!(Spi2StatusReg & SPI_INIT)) return 0;
	//Ожидание освобождения передающего буфера.
	while(!(SPI2->SR & SPI_SR_TXE))
		{
			if(++SpiWaitCount > SPI_WAIT) return 0;
		}
		
	SpiWaitCount = 0;
	SPI2->DR = byte;
		
	while(SPI2->SR & SPI_SR_BSY)
		{
			if(++SpiWaitCount > SPI_WAIT) return 0;
		}
	//--------------------
	return (uint8_t)SPI2->DR;
} 
//-----------------------------------------------------------------------------
//Прерываени от SPI2.
void SPI2_IRQHandler(void){
  
  //uint16_t tmp;
  //--------------------
  //причина прерывания - окончание передачи байта
  if(SPI2->SR & SPI_SR_TXE)
    {
      //SPI2->CR1 &= ~SPI_CR1_SPE;  //Запрет работы модуля SPI
      //SPI2->SR  &= ~SPI_SR_TXE;   //сбрасываем флаг события, которое вызвало прерывание
      //SPI2->CR2 |=  SPI_CR2_TXEIE;//Запрет прерывания по окончанию передачи 
      
      //StpicCSHight;
      //StpicCSLow; 

      //LC2SostGreenLedToggel;//Мигаем для отладки          
    }
  //--------------------
  //причина прерывания - принят байт данных
  if(SPI2->SR &= SPI_SR_RXNE)
    {
      //tmp = SPI1->DR;                //прочитать принятые данные
      //выполняем какие-то действия      
    }
  //--------------------
  //причина прерывания - ошибка режима ведущего
  if(SPI2->SR &= SPI_SR_MODF)
    {
      //сбрасываем флаг события, которое вызвало прерывание и делаем что-то полезное
    }
  //--------------------
  //причина прерывания - переполнение приемника данных
  if(SPI2->SR &= SPI_SR_OVR)
    {
      //сбрасываем флаг события, которое вызвало прерывание и делаем что-то полезное
    }
  //--------------------
  //причина прерывания - ошибка данных
//  if(SPI2->SR &= SSPI_SR_CRCERR)
//    {
//      //сбрасываем флаг события, которое вызвало прерывание и делаем что-то полезное
//    }
  //--------------------
}

//-----------------------------------------------------------------------------
