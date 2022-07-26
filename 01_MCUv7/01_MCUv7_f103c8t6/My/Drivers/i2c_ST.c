/*
 * i2c_ST.c
 *
 *  Created on: 20 дек. 2020 г.
 *      Author:
 */
//*******************************************************************************************
//*******************************************************************************************

#include "i2c_ST.h"

//*******************************************************************************************
//*******************************************************************************************
static uint32_t _i2c_LongWait(I2C_TypeDef *i2c, uint32_t flag){

	uint32_t wait_count = 0;
	//---------------------
	while(!(i2c->SR1 & flag))//Ожидаем окончания ....
	{
		if(++wait_count >= I2C_WAIT) return 1;
	}
	return 0;
}
//**********************************************************
static void _i2c_GPIO_Init(I2C_TypeDef *const i2c, uint32_t remap){

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;//Включаем тактирование GPIOB
	//Тактирование I2C_1
	if(i2c == I2C1)
	{
		//Ремап: I2C1_SCL - PB8, I2C1_SDA - PB9.
		if(remap == I2C_GPIO_REMAP)
		{
			AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
			GPIOB->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1 |
						  GPIO_CRH_CNF8    | GPIO_CRH_CNF9;
		}
		else
		{
			//I2C1_SCL - PB6, I2C1_SDA - PB7.
			GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1 |
						  GPIO_CRL_CNF6    | GPIO_CRL_CNF7;

		}
	}
	//Тактирование I2C_2
	else if(i2c == I2C2)
	{
		//I2C2_SCL - PB10, I2C2_SDA - PB11.
		GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_MODE11_1 |
					  GPIO_CRH_CNF10    | GPIO_CRH_CNF11;
	}
}
//*******************************************************************************************
//*******************************************************************************************
void I2C_Slave_Init(I2C_TypeDef *i2c, uint32_t slaveAddr, uint32_t remap){

	_i2c_GPIO_Init(i2c, remap);
	if(i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	else		    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	//Инициализация I2C в режиме Slave.
	i2c->CR1 |=  I2C_CR1_PE;    //Включение модуля I2C1.
	i2c->CR1 &= ~I2C_CR1_SMBUS; //модуль работает в режиме I2C
	i2c->SR2 &= ~I2C_SR2_MSL;   //режим Slave.

	i2c->OAR1 = slaveAddr << 1; //адрес устройства на шине.
	i2c->CR1 |= I2C_CR1_ACK;	//разрешаем отправлять ACK/NACK после приема байта адреса.

	//i2c->CR1 |= I2C_CR1_NOSTRETCH;

	//Скорость работы.
	i2c->CR2  &= ~I2C_CR2_FREQ;   		  //
	i2c->CR2  |= (36 << I2C_CR2_FREQ_Pos);//APB1 = 36MHz

	i2c->CCR  &= ~I2C_CCR_CCR;
	i2c->CCR  |=  I2C_CCR_FS; //1 - режим FastMode(400kHz), 0 - режим STANDART(100kHz).
	//i2c->CCR   =  120;//100кГц
	i2c->CCR   =  30; //400кГц  45;//I2C_CCR_VALUE;  //(36MHz/I2C_BAUD_RATE/2)
	i2c->TRISE =  12;//37;//I2C_TRISE_VALUE;//(1mcs/(1/36MHz)+1)

	//i2c->CR1  |=  I2C_CR1_PE; //Включение модуля I2C1.
}
//**********************************************************



//*******************************************************************************************
//*******************************************************************************************
void I2C_Init(I2C_TypeDef *i2c, uint32_t remap){

	_i2c_GPIO_Init(i2c, remap);
	if(i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	else		    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	//Инициализация I2C.
	i2c->CR2  &= ~I2C_CR2_FREQ;   		  //
	i2c->CR2  |= (36 << I2C_CR2_FREQ_Pos);//APB1 = 36MHz

	//Скорость работы.
	i2c->CCR  &= ~I2C_CCR_CCR;    //
	//i2c->CCR   =  120;//100кГц
	i2c->CCR   =  30; //400кГц  45;//I2C_CCR_VALUE;  //(36MHz/I2C_BAUD_RATE/2)

	i2c->CCR  |=  I2C_CCR_FS; //1 - режим FastMode(400kHz), 0 - режим STANDART(100kHz).
	i2c->TRISE =  12;//37;//I2C_TRISE_VALUE;//(1mcs/(1/36MHz)+1)
	i2c->CR1  |=  I2C_CR1_PE; //Включение модуля I2C1.
}
//**********************************************************
uint8_t I2C_StartAndSendDeviceAddr(I2C_TypeDef *i2c, uint8_t deviceAddr){

	uint32_t wait = 0;
	//---------------------
	//Ждем освобождения шины. - не отлажено!!!
	while(i2c->SR2 & I2C_SR2_BUSY)
		{
			if(++wait >= I2C_WAIT)
			{
				i2c->CR1 |=	I2C_CR1_SWRST;
				return 1;
			}
		}
	wait = 0;
	//Формирование Start condition.
	i2c->CR1 |= I2C_CR1_START;
	while(!(i2c->SR1 & I2C_SR1_SB))//Ожидание формирования Start condition.
		{
			if(++wait >= I2C_WAIT)
				{
					i2c->CR1 |=	I2C_CR1_SWRST;
					return 1;
				}
		}
	wait = 0;
	(void)i2c->SR1;				    //Для сброса флага SB необходимо прочитать SR1
	//Передаем адрес slave.
	i2c->DR = deviceAddr;
	while(!(i2c->SR1 & I2C_SR1_ADDR))//Ожидаем окончания передачи адреса и
		{
			if(++wait >= I2C_WAIT)
				{
					i2c->CR1 |=	I2C_CR1_SWRST;
					return 1;
				}
		}
	wait = 0;
	(void)i2c->SR1;				      //сбрасываем бит ADDR (чтением SR1 и SR2):
	(void)i2c->SR2;				      //

	return 0;
}
//**********************************************************
void I2C_SendData(I2C_TypeDef *i2c, uint8_t *pBuf, uint16_t len){

	for(uint16_t i = 0; i < len; i++)
		{
			i2c->DR = *(pBuf + i);
			while(!(i2c->SR1 & I2C_SR1_TXE));//Ждем освобождения буфера
		}
}
//**********************************************************
void I2C_ReadData(I2C_TypeDef *i2c, uint8_t *pBuf, uint16_t len){

	if(len == 1)
		{
			i2c->CR1 &= ~I2C_CR1_ACK;           //Фомирование NACK после приема последнего байта.
			while(!(i2c->SR1 & I2C_SR1_RXNE)){};//ожидаем окончания приема байта
			*(pBuf + 0) = i2c->DR;				//считали принятый байт.
			i2c->CR1 |= I2C_CR1_STOP;           //Формируем Stop
		}
	else
		{
			for(uint16_t i = 0; i < (len-1); i++)
				{
					i2c->CR1 |= I2C_CR1_ACK; 			//Фомирование ACK после приема байта
					while(!(i2c->SR1 & I2C_SR1_RXNE)){};//ожидаем окончания приема байта
					*(pBuf + i) = i2c->DR;			    //считали принятый
				}

			i2c->CR1 &= ~I2C_CR1_ACK;           //Фомирование NACK после приема последнего байта.
			while(!(i2c->SR1 & I2C_SR1_RXNE)){};//ожидаем окончания приема байта
			*(pBuf + len - 1) = i2c->DR;		//считали принятый байт.
			i2c->CR1 |= I2C_CR1_STOP;           //Формируем Stop
		}
}
//**********************************************************
void I2C_Stop(I2C_TypeDef *i2c){

	while(!(i2c->SR1 & I2C_SR1_BTF));//Ждем окончания передачи
	i2c->CR1 |= I2C_CR1_STOP;		 //Формируем Stop
}
//*******************************************************************************************
//*******************************************************************************************
void I2C_Write(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint16_t len){

	//Формирование Start condition.
	i2c->CR1 |= I2C_CR1_START;
	if(_i2c_LongWait(i2c, I2C_SR1_SB)) return;//Ожидание формирования Start condition.
	(void)i2c->SR1;				      		  //Для сброса флага SB необходимо прочитать SR1

	//Передаем адрес slave + Запись.
	i2c->DR = deviceAddr | I2C_MODE_WRITE;
	if(_i2c_LongWait(i2c, I2C_SR1_ADDR)) return;//Ожидаем окончания передачи адреса
	(void)i2c->SR1;				        	    //сбрасываем бит ADDR (чтением SR1 и SR2):
	(void)i2c->SR2;				        	    //

	//Передача адреса в который хотим записать.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return;
	//передача данных на запись.
	for(uint16_t i = 0; i < len; i++)
		{
			i2c->DR = *(pBuf + i);
			if(_i2c_LongWait(i2c, I2C_SR1_TXE)) goto STOP;//Ждем освобождения буфера
		}
	STOP:
	i2c->CR1 |= I2C_CR1_STOP;//Формируем Stop
}
//**********************************************************
void I2C_Read(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint16_t len){

	//Формирование Start condition.
	i2c->CR1 |= I2C_CR1_START;
	if(_i2c_LongWait(i2c, I2C_SR1_SB)) return;//Ожидание формирования Start condition.
	(void)i2c->SR1;							 //Для сброса флага SB необходимо прочитать SR1

	//Передаем адрес slave + Запись.
	i2c->DR = deviceAddr | I2C_MODE_WRITE;
	if(_i2c_LongWait(i2c, I2C_SR1_ADDR)) return;//Ожидаем окончания передачи адреса
	(void)i2c->SR1;							   //сбрасываем бит ADDR (чтением SR1 и SR2):
	(void)i2c->SR2;							   //

	//Передача адреса с которого начинаем чтение.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return;
	//---------------------
	//Формирование reStart condition.
	i2c->CR1 |= I2C_CR1_STOP; //Это команда нужня для работы с DS2782. Без нее не работает
	i2c->CR1 |= I2C_CR1_START;
	if(_i2c_LongWait(i2c, I2C_SR1_SB)) return;//Ожидание формирования Start condition.
	(void)i2c->SR1;							 //Для сброса флага SB необходимо прочитать SR1

	//Передаем адрес slave + Чтение.
	i2c->DR = deviceAddr | I2C_MODE_READ;
	if(_i2c_LongWait(i2c, I2C_SR1_ADDR)) return;//Ожидаем окончания передачи адреса
	(void)i2c->SR1;							   //сбрасываем бит ADDR (чтением SR1 и SR2):
	(void)i2c->SR2;							   //
	//прием даннных
	if(len == 1)
		{
			i2c->CR1 &= ~I2C_CR1_ACK;                	  //Фомирование NACK после приема последнего байта.
			if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) goto STOP;//ожидаем окончания приема байта
			*(pBuf + 0) = i2c->DR;				          //считали принятый байт.
		}
	else
		{
			for(uint16_t i = 0; i < (len-1); i++)
				{
					i2c->CR1 |= I2C_CR1_ACK; 			          //Фомирование ACK после приема байта
					if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) goto STOP;//ожидаем окончания приема байта
					*(pBuf + i) = i2c->DR;			         	  //считали принятый
				}
			i2c->CR1 &= ~I2C_CR1_ACK;                	  //Фомирование NACK после приема последнего байта.
			if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) goto STOP;//ожидаем окончания приема байта
			*(pBuf + len - 1) = i2c->DR;		     	  //считали принятый байт.
		}
	//---------------------
	STOP:
	i2c->CR1 |= I2C_CR1_STOP;//Формируем Stop
}
//*******************************************************************************************
//*******************************************************************************************
//****************************Работа I2C по прерываниям.*************************************
static uint32_t I2cMode	   = 0;
static uint32_t I2cItState = 0;

static uint8_t SlaveAddr    = 0;
static uint8_t SlaveRegAddr = 0;

static uint8_t *ptrTxBuf  	= 0;
static uint32_t TxBufSize 	= 0;

static uint8_t *ptrRxBuf  	= 0;
static uint32_t RxBufSize 	= 0;

//**********************************************************
void I2C_IT_Init(I2C_TypeDef *i2c, uint32_t remap){

	I2C_Init(i2c, remap);
	//Инит-я прерывания.
	i2c->CR2 |= I2C_CR2_ITEVTEN |      //Разрешение прерывания по событию.
				I2C_CR2_ITERREN;       //Разрешение прерывания по ошибкам.

	NVIC_SetPriority(I2C1_EV_IRQn, 15);//Приоритет прерывания.
	NVIC_SetPriority(I2C1_ER_IRQn, 15);//Приоритет прерывания.

	NVIC_EnableIRQ(I2C1_EV_IRQn);      //Разрешаем прерывание.
	NVIC_EnableIRQ(I2C1_ER_IRQn);      //Разрешаем прерывание.
}
//**********************************************************
void I2C_IT_StartTx(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pTxBuf, uint32_t len){

	SlaveAddr    = deviceAddr;
	SlaveRegAddr = regAddr;
	ptrTxBuf     = pTxBuf;
	TxBufSize    = len;

	i2c->CR1  |= I2C_CR1_START;
	I2cItState = 1;
}
//**********************************************************
void I2C_IT_StartRx(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pRxBuf, uint32_t len){

	SlaveAddr    = deviceAddr;
	SlaveRegAddr = regAddr;
	ptrRxBuf     = pRxBuf;
	RxBufSize    = len;

	i2c->CR1  |= I2C_CR1_START;
	I2cItState = 1;
}
//**********************************************************
uint8_t I2C_IT_GetState(void){

	return I2cItState;
}
//*******************************************************************************************
//*******************************************************************************************
void I2C_IT_Slave_Init(I2C_TypeDef *i2c, uint32_t slaveAddr, uint32_t remap){

	I2C_Slave_Init(i2c, slaveAddr, remap);

	I2cMode = I2C_MODE_SLAVE;

	//Инит-я прерывания.
	i2c->CR2 |= I2C_CR2_ITEVTEN | //Разрешение прерывания по событию.
				I2C_CR2_ITERREN;  //Разрешение прерывания по ошибкам.

	NVIC_SetPriority(I2C1_EV_IRQn, 15);//Приоритет прерывания.
	NVIC_SetPriority(I2C1_ER_IRQn, 15);//Приоритет прерывания.

	NVIC_EnableIRQ(I2C1_EV_IRQn);      //Разрешаем прерывание.
	NVIC_EnableIRQ(I2C1_ER_IRQn);      //Разрешаем прерывание.
}
//**********************************************************
void I2C_IT_Slave_StartRx(uint8_t *pRxBuf, uint32_t len){

	ptrRxBuf  = pRxBuf;
	RxBufSize = len;
}
//**********************************************************
void I2C_IT_Slave_StartTx(uint8_t *pTxBuf, uint32_t len){

	ptrTxBuf  = pTxBuf;
	TxBufSize = len;
}
//*******************************************************************************************
//*******************************************************************************************
//Обработчик прерывания событий I2C
void I2C1_EV_IRQHandler(void){

//	volatile uint32_t sr1 = I2C1->SR1; //при чтении рагистров SR1 и SR2 происходит сброс флаго
//	volatile uint32_t sr2 = I2C1->SR2;
	static   uint32_t txCount = 0;
	static   uint32_t rxCount = 0;
	//LedPC13Toggel();
	//------------------------------
	//Сформирована START последоательность.
	if(I2C1->SR1 & I2C_SR1_SB)
	{
		(void)I2C1->SR1;				      //Для сброса флага SB необходимо прочитать SR1
		I2C1->DR = SlaveAddr | I2C_MODE_WRITE;//Передаем адрес slave + Запись.
		return;
	}
	//------------------------------
	//Окончание приема/передачи адреса.
	if(I2C1->SR1 & I2C_SR1_ADDR)
	{
		(void)I2C1->SR1; //сбрасываем бит ADDR (чтением SR1 и SR2):
		(void)I2C1->SR2;
		//Slave
		if(I2cMode == I2C_MODE_SLAVE)
		{
			//От Мастера пришла команда на запись.
			if((I2C1->SR2 & I2C_SR2_TRA) == 0)
			{
				*(ptrRxBuf + 0) = I2C1->DR;//Сохранили первый байт
				rxCount++;
			}
			//От Мастера пришла команда на чтение.
			else
			{
				I2C1->DR = *(ptrTxBuf + 0);
				txCount  = 1;
			}
		}
		//Master
		else
		{
			I2C1->DR   = SlaveRegAddr;   //Передача адреса в который хотим записать.
			I2C1->CR2 |= I2C_CR2_ITBUFEN;//Разрешение прерывания по опустощению буфера Tx.
		}
		return;
	}
	//------------------------------
	//Передающий буфер свободен.
	if(I2C1->SR1 & I2C_SR1_TXE)
	{
		(void)I2C1->SR1; //сбрасываем бит TXE (чтением SR1 и SR2):
		(void)I2C1->SR2;
		//Slave
		if(I2cMode == I2C_MODE_SLAVE)
		{
			I2C1->DR = *(ptrTxBuf + txCount);
			txCount++;

			//if(txCount >= TxBufSize)txCount = 0;
		}
		//Master
		else
		{
			if(txCount < TxBufSize)
			{
				I2C1->DR = *(ptrTxBuf + txCount);
				txCount++;
			}
			else
			{
				I2C1->CR2 &= ~I2C_CR2_ITBUFEN;//Запрет прерывания по опостощению буфера Tx.
				I2C1->CR1 |= I2C_CR1_STOP;    //Формируем Stop
				I2cItState = 0;
				txCount    = 0;
			}
		}
		return;
	}
	//------------------------------
	//Принят байт.
	if(I2C1->SR1 & I2C_SR1_RXNE)
	{
		if(rxCount < RxBufSize)
		{
			*(ptrRxBuf + rxCount) = I2C1->DR;
			rxCount++;
		}
		else
		{
			rxCount = 0;
		}

		return;
	}
	//------------------------------
	//STOP
	if(I2C1->SR1 & I2C_SR1_STOPF)
	{
		(void)I2C1->SR1; 		   //сбрасываем бит STOPF
		I2C1->CR1 &= ~I2C_CR1_STOP;//
	}
	//------------------------------
}
//**********************************************************
//Обработчик прерывания ошибок I2C
void I2C1_ER_IRQHandler(void){

	LedPC13Toggel();
	//------------------------------
	//NACK - Acknowledge failure
	if(I2C1->SR1 & I2C_SR1_AF)
	{
		I2C1->SR1 &= ~I2C_SR1_AF; //Сброс AF.
		//I2C1->CR1 |= I2C_CR1_STOP;//Формируем Stop
		//return;
	}
	//------------------------------
	//Bus error
	if(I2C1->SR1 & I2C_SR1_BERR)
	{
		I2C1->SR1 &= ~I2C_SR1_BERR; //Сброс BERR.
	}
	//------------------------------
	//Arbitration loss (Master)
	if(I2C1->SR1 & I2C_SR1_ARLO)
	{
		I2C1->SR1 &= ~I2C_SR1_ARLO; //Сброс ARLO.
	}
	//------------------------------
	//Overrun/Underrun
	if(I2C1->SR1 & I2C_SR1_OVR)
	{
		I2C1->SR1 &= ~I2C_SR1_OVR; //Сброс OVR.
	}
	//------------------------------
	//PEC error
	if(I2C1->SR1 & I2C_SR1_PECERR)
	{
		I2C1->SR1 &= ~I2C_SR1_PECERR; //Сброс PECERR.
	}
	//------------------------------
	//Timeout/Tlow error
	if(I2C1->SR1 & I2C_SR1_TIMEOUT)
	{
		I2C1->SR1 &= ~I2C_SR1_TIMEOUT; //Сброс TIMEOUT.
	}
	//------------------------------
	//SMBus Alert
	if(I2C1->SR1 & I2C_SR1_SMBALERT)
	{
		I2C1->SR1 &= ~I2C_SR1_SMBALERT; //Сброс SMBALERT.
	}
	//------------------------------
}
//*******************************************************************************************
//*******************************************************************************************




