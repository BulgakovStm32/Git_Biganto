//*******************************************************************************************
//*******************************************************************************************

#include "spi_ST.h"

//*******************************************************************************************
//*******************************************************************************************



//*******************************************************************************************
//*******************************************************************************************
void _spi_GPIO_Init(SPI_TypeDef *spi){

	if(spi == SPI1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;//Включаем тактирование порта A.
		//PA5(SPI1_SCK)  - выход
		//PA7(SPI1_MOSI) - выход
		//PA6(SPI1_MISO) - вход
		GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
		//PA5(SPI1_SCK) - выход, альтернативный режим push-pull, тактирование 50МГц.
		GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5;
		//PA6(SPI1_MISO) - вход
		GPIOA->CRL |=  GPIO_CRL_CNF6_1;
		GPIOA->BSRR =  GPIO_BSRR_BS6;
		//PA7(SPI1_MOSI) - выход, альтернативный режим push-pull, тактирование 50МГц.
		//GPIOA->CRL |= GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	}
	else if(spi == SPI2)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;//Включаем тактирование порта B.
		//PB13(SPI2_SCK)  - выход
		//PB15(SPI2_MOSI) - выход
		//PB14(SPI2_MISO) - вход
		GPIOB->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE14 | GPIO_CRH_CNF14 | GPIO_CRH_CNF15);
		//PB13(SPI2_SCK) - выход, альтернативный режим push-pull, тактирование 50МГц.
		GPIOB->CRH |= GPIO_CRH_CNF13_1 | GPIO_CRH_MODE13;
		//PB14(SPI2_MISO) - вход
		GPIOB->CRH |=  GPIO_CRH_CNF14_1;
		GPIOB->BSRR =  GPIO_BSRR_BS14;
		//PB15(SPI2_MOSI) - выход, альтернативный режим push-pull, тактирование 50МГц.
		GPIOB->CRH |= GPIO_CRH_CNF15_1 | GPIO_CRH_MODE15;
	}
	else return;
}
//*******************************************************************************************
//*******************************************************************************************
void SPI_Init(SPI_TypeDef *spi){

	_spi_GPIO_Init(spi);
		 if(spi == SPI1) RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;//Включение тактирования SPI1.
	else if(spi == SPI2) RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;//Включение тактирования SPI2.
	else return;
	//--------------------
	spi->CR1  = 0;
	spi->CR1 |= (SPI_CR1_MSTR |	 //режим "мастер".
				 //Скорость. Fpclk/BR = 72MHz/BR = 4.5MHz
				 (3 << SPI_CR1_BR_Pos) | // Bits 5:3 BR[2:0]: Baud rate control
				 	 	 	 	 	 	 // 000: fPCLK/2
										 // 001: fPCLK/4
										 // 010: fPCLK/8
										 // 011: fPCLK/16
										 // 100: fPCLK/32
										 // 101: fPCLK/64
										 // 110: fPCLK/128
										 // 111: fPCLK/256
				 //Настройки для работы с энодером.
				 //SPI_CR1_BIDIMODE |
				 SPI_CR1_RXONLY | // Output disabled (Receive-only mode)
				 SPI_CR1_CPOL   |
				 SPI_CR1_CPHA   |
				 //SPI_CR1_LSBFIRST |//Младшим битом вперед
				 //SPI_CR1_DFF  |	 //16 бит данных.
				 SPI_CR1_SSI    | //обеспечить высокий уровень программного NSS
				 SPI_CR1_SSM    | //разрешить программное формирование NSS
				 SPI_CR1_SPE);    //разрешить работу модуля SPI
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
//**********************************************************
uint8_t SPI_RxByte(SPI_TypeDef *spi){

	uint32_t SpiWaitCount = 0;
	//--------------------
	while(!(spi->SR & SPI_SR_RXNE))
	{
		if(++SpiWaitCount > SPI_WAIT) return 0;
	}
	return (uint8_t)spi->DR;
}
//**********************************************************
//Передача данных(8 бит) в SPI1.
uint8_t	SPI_TxRxByte(SPI_TypeDef *spi, uint32_t byte){

	uint32_t spiWaitCount = 0;
 	//--------------------
	//Ожидание освобождения передающего буфера.
	while(!(spi->SR & SPI_SR_TXE)){};
	{
		if(++spiWaitCount > SPI_WAIT) return 0;
	}
	spiWaitCount = 0;
	spi->DR = (uint8_t)byte;

	while(spi->SR & SPI_SR_BSY){};
	{
		if(++spiWaitCount > SPI_WAIT) return 0;
	}
	//--------------------
	return (uint8_t)spi->DR;
}
//**********************************************************
uint32_t SPI_Rx3Byte(SPI_TypeDef *spi){

	volatile uint32_t temp = 0;
	//--------------------
	spi->CR1 |= SPI_CR1_SPE;//Запуск модуля SPI1.
	(void)spi->DR;			//Это нужно для кооректного чтения данных из SPI

	temp |= SPI_RxByte(spi) << 16;
	temp |= SPI_RxByte(spi) << 8;
	temp |= SPI_RxByte(spi);

	spi->CR1 &= ~SPI_CR1_SPE;  //Останов модуля SPI1.

	return temp;
}
//**********************************************************



//*******************************************************************************************
//*******************************************************************************************
