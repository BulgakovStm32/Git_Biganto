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
static uint32_t I2C1NacCount = 0;
static uint32_t I2C2NacCount = 0;
//*******************************************************************************************
//*******************************************************************************************

//static uint32_t _i2c_LongWait(I2C_TypeDef *i2c, uint32_t flag){
//
//	uint32_t wait_count = 0;
//	//---------------------
//	while(!(i2c->SR1 & flag))//Ожидаем окончания ....
//	{
//		if(++wait_count >= I2C_WAIT_TIMEOUT) return 1;
//	}
//	return 0;
//}

static uint32_t _i2c_LongWait(I2C_TypeDef *i2c, uint32_t flag){

	uint32_t wait_count = 0;
	//---------------------
	while(!(i2c->SR1 & flag))//Ждем отпускания флага.
	{
		//if(++wait_count >= I2C_WAIT_TIMEOUT) return 1;
		if((i2c->SR1 & I2C_SR1_AF) ||		   //Если NAC или
		   (++wait_count >= I2C_WAIT_TIMEOUT)) //вышел таймаут
		{
			i2c->SR1 &= ~(I2C_SR1_AF   |  //Сброс флагов ошибок.
					      I2C_SR1_BERR |  //
						  I2C_SR1_ARLO |
						  I2C_SR1_OVR);

			i2c->CR1 |= I2C_CR1_STOP|   //Формируем Stop
					    I2C_CR1_ACK;    //to be ready for another reception
//			(void)i2c->SR1;
//			(void)i2c->SR2;
			return 1;
		}
	}
	return 0;
}
//**********************************************************
static void _i2c_GPIO_Init(I2C_TypeDef *i2c, uint32_t remap){

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
		//I2C1_SCL - PB6, I2C1_SDA - PB7.
		else
		{
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
//**********************************************************
static void _i2c_ClearFlags(I2C_TypeDef *i2c){

	(void)i2c->SR1; //сбрасываем флаги чтением SR1 и SR2.
	(void)i2c->SR2;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**************************Общие функции для работы с I2C***********************************
I2C_State_t I2C_StartAndSendDeviceAddr(I2C_TypeDef *i2c, uint32_t deviceAddr){

	//Формирование Start condition.
	i2c->CR1 |= I2C_CR1_START;
	if(_i2c_LongWait(i2c, I2C_SR1_SB)) return I2C_ERR_START;//Ожидание формирования Start condition.
	(void)i2c->SR1;			   		//Для сброса флага SB необходимо прочитать SR1

	//Передаем адрес.
	i2c->DR = deviceAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_ADDR))//Ожидаем окончания передачи адреса
	{
		if(i2c == I2C1) I2C1NacCount++;
		else			I2C2NacCount++;
		return I2C_ERR_ADDR;
	}
	(void)i2c->SR1;	//сбрасываем бит ADDR (чтением SR1 и SR2):
	(void)i2c->SR2;	//
	return I2C_OK;
}
//**********************************************************
I2C_State_t I2C_SendByte(I2C_TypeDef *i2c, uint8_t byte){

	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
	i2c->DR = byte;
	return I2C_OK;
}
//**********************************************************
I2C_State_t I2C_ReadData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){

	//receiving 1 byte
	if(len == 1)
	{
		i2c->CR1 &= ~I2C_CR1_ACK;				   //Фомирование NACK.
		i2c->CR1 |= I2C_CR1_STOP;				   //Формируем Stop.
		if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;//ожидаем окончания приема байта
		*(pBuf + 0) = i2c->DR;				       //считали принятый байт.
		i2c->CR1 |= I2C_CR1_ACK;				   //to be ready for another reception
	}
	//receiving 2 bytes
	else if(len == 2)
	{
		i2c->CR1 |=  I2C_CR1_POS;//
		i2c->CR1 &= ~I2C_CR1_ACK;//Фомирование NACK.
		if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
		i2c->CR1 |= I2C_CR1_STOP;//Формируем Stop.
		*(pBuf + 0) = i2c->DR;	 //считали принятый байт.
		*(pBuf + 1) = i2c->DR;	 //считали принятый байт.
		i2c->CR1 &= ~I2C_CR1_POS;//
		i2c->CR1 |= I2C_CR1_ACK; //to be ready for another reception
	}
	//receiving more than 2 bytes
	else
	{
		uint32_t i;
		for(i=0; i<(len-3); i++)
		{
			if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;
			*(pBuf + i) = i2c->DR;//Read DataN
		}
		//Вычитываем оставшиеся 3 байта.
		if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
		i2c->CR1 &= ~I2C_CR1_ACK; //Фомирование NACK
		*(pBuf + i + 0) = i2c->DR;//Read DataN-2
		i2c->CR1 |= I2C_CR1_STOP; //Формируем Stop
		*(pBuf + i + 1) = i2c->DR;//Read DataN-1
		if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;
		*(pBuf + i + 2) = i2c->DR;//Read DataN
		i2c->CR1 |= I2C_CR1_ACK;
	}
	return I2C_OK;
}
//**********************************************************
void I2C_Stop(I2C_TypeDef *i2c){

	if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return;
	i2c->CR1 |= I2C_CR1_STOP;		 //Формируем Stop
}
//**********************************************************
I2C_State_t I2C_SendData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){

	for(uint32_t i = 0; i < len; i++)
	{
		if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
		i2c->DR = *(pBuf + i);
	}
	if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
	i2c->CR1 |= I2C_CR1_STOP;		 //Формируем Stop
	return I2C_OK;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**************************Функции для работы в режиме Master*******************************
void I2C_Master_Init(I2C_TypeDef *i2c, uint32_t remap){

	_i2c_GPIO_Init(i2c, remap);
	if(i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	else		    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//Инициализация I2C.
	i2c->CR1 &= ~I2C_CR1_PE;   //Откл. модуля I2C.
	i2c->CR1 |=  I2C_CR1_SWRST;//Программный сброс модуля I2C
	i2c->CR1 &= ~I2C_CR1_SWRST;//Это нужно для востановления работоспособноси после КЗ на линии.

	i2c->CR2  &= ~I2C_CR2_FREQ;   		  		//
	i2c->CR2  |= (I2C_FREQ << I2C_CR2_FREQ_Pos);//APB1 = 36MHz

	//Скорость работы.
	i2c->CCR  &= ~I2C_CCR_CCR;    //
	//i2c->CCR   =  120;//100кГц
	i2c->CCR   =  30; //400кГц  45;//I2C_CCR_VALUE;  //(36MHz/I2C_BAUD_RATE/2)

	i2c->CCR  |=  I2C_CCR_FS; //1 - режим FastMode(400kHz), 0 - режим STANDART(100kHz).
	i2c->TRISE =  12;//37;//I2C_TRISE_VALUE;//(1mcs/(1/36MHz)+1)
	i2c->CR1  |=  I2C_CR1_PE; //Включение модуля I2C1.
}
//**********************************************************
uint32_t I2C_Master_GetNacCount(I2C_TypeDef *i2c){

	if(i2c == I2C1) return I2C1NacCount;
					return I2C2NacCount;
}
//**********************************************************
/*  Ф-ия передачи массива данных в Slave-устройство.
 *  Ф-ия блокирует работу основной программы пока не будет передан весь массив данных.
 *  Вход: *i2c 		 - используемый порт i2c,
 *  	  deviceAddr - адресс Slave-устройства,
 *  	  regAddr    - адрес регистра Slave-устройства куда хотим записать массив,
 *  	  *pBuf      - указать на буфер передачи,
 *  	  len		 - размер буфера передачи.
 *
 *  Выход: статус передачи I2C_State_t.
 */
I2C_State_t I2C_Master_Write(I2C_TypeDef *i2c, uint32_t deviceAddr, uint32_t regAddr, uint8_t *pBuf, uint32_t len){

	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_WRITE) != I2C_OK) return I2C_ERR_ADDR;

	//Передача адреса в который хотим записать.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;

	//передача данных на запись.
	return(I2C_SendData(i2c, pBuf, len));
}
//**********************************************************
/*  Ф-ия чтения массива данных из Slave-устройство.
 *  Ф-ия блокирует работу основной программы пока не будет вычитан весь буфер.
 *  Вход: *i2c 		 - используемый порт i2c,
 *  	  deviceAddr - адресс Slave-устройства,
 *  	  regAddr    - адрес регистра Slave-устройства откуда хотим читать данные,
 *  	  *pBuf      - указать на буфер приема,
 *  	  len		 - размер буфера приема.
 *
 *  Выход: статус передачи I2C_State_t.
 */
I2C_State_t I2C_Master_Read(I2C_TypeDef *i2c, uint32_t deviceAddr, uint32_t regAddr, uint8_t *pBuf, uint32_t len){

	//if(I2C_DMA_State() != I2C_DMA_READY) return I2C_BUSY;

	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_WRITE) != I2C_OK) return I2C_ERR_ADDR;

	//Передача адреса с которого начинаем чтение.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;
	//---------------------
	//Формирование reStart condition.
	i2c->CR1 |= I2C_CR1_STOP; //Это команда нужна для работы с DS2782. Без нее не работает

	//Формирование Start + AddrSlave|Read.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_READ) != I2C_OK) return I2C_ERR_ADDR;

	//прием даннных
	return(I2C_ReadData(i2c, pBuf, len));
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void I2C_Slave_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t slaveAddr){

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
	i2c->CR2  &= ~I2C_CR2_FREQ;   		        //
	i2c->CR2  |= (I2C_FREQ << I2C_CR2_FREQ_Pos);//APB1 = 36MHz

	i2c->CCR   &= ~I2C_CCR_CCR;
	i2c->CCR   |=  I2C_CCR_FS; //1 - режим FastMode(400kHz), 0 - режим STANDART(100kHz).
	i2c->CCR   |= (I2C_FM_CCR << I2C_CCR_CCR_Pos);
	i2c->TRISE |= (I2C_FM_TRISE << I2C_TRISE_TRISE_Pos);

	//i2c->CCR   =  120;//100кГц
	//i2c->CCR   =  30; //400кГц  45;//I2C_CCR_VALUE;  //(36MHz/I2C_BAUD_RATE/2)
	//i2c->TRISE =  12; //37;//I2C_TRISE_VALUE;//(1mcs/(1/36MHz)+1)

	//i2c->CR1  |=  I2C_CR1_PE; //Включение модуля I2C1.
}
//**********************************************************

//**********************************************************

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//****************************Работа I2C по прерываниям.*************************************
/*
 * Указатели на контекст шины I2C.
 * Они нужны для работы обработчиков прерываний.
 */
static I2C_IT_t	*i2c1_IT_Define;
static I2C_IT_t	*i2c2_IT_Define;
//*******************************************************************************************
//*******************************************************************************************
void I2C_IT_Init(I2C_IT_t *i2cIt){

		 if(i2cIt->i2c == I2C1)  i2c1_IT_Define = i2cIt;
	else if(i2cIt->i2c == I2C2)  i2c2_IT_Define = i2cIt;
	else return;

	if(i2cIt->i2cMode == I2C_MODE_MASTER) I2C_Master_Init(i2cIt->i2c, i2cIt->i2cGpioRemap);
	else								  I2C_Slave_Init (i2cIt->i2c, i2cIt->i2cGpioRemap, i2cIt->slaveAddr);
	//Инициализация прерывания.
	i2cIt->i2c->CR2 |= I2C_CR2_ITEVTEN | //Разрешение прерывания по событию.
					   I2C_CR2_ITERREN;  //Разрешение прерывания по ошибкам.

	if(i2cIt->i2c == I2C1)
	{
		NVIC_SetPriority(I2C1_EV_IRQn, 15);//Приоритет прерывания.
		NVIC_SetPriority(I2C1_ER_IRQn, 15);//Приоритет прерывания.

		NVIC_EnableIRQ(I2C1_EV_IRQn);      //Разрешаем прерывание.
		NVIC_EnableIRQ(I2C1_ER_IRQn);      //Разрешаем прерывание.
	}
	else
	{
		NVIC_SetPriority(I2C2_EV_IRQn, 15);//Приоритет прерывания.
		NVIC_SetPriority(I2C2_ER_IRQn, 15);//Приоритет прерывания.

		NVIC_EnableIRQ(I2C2_EV_IRQn);      //Разрешаем прерывание.
		NVIC_EnableIRQ(I2C2_ER_IRQn);      //Разрешаем прерывание.
	}
}
//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//***************************Обработчики прерывания******************************************
//Обработчик прерывания событий I2C
static void I2C_IT_Event(I2C_IT_t *i2cIt){

	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	//Start bit (Master mode) - Сформирована START последоательность.
	if(i2c->SR1 & I2C_SR1_SB)
	{
		_i2c_ClearFlags(i2c);//Для сброса флага SB необходимо прочитать SR1
		i2c->DR = i2cIt->slaveAddr | I2C_MODE_WRITE;//Передаем адрес slave + Запись.
		//return;
	}
	//------------------------------
	//Окончание приема/передачи адреса.
	if(i2c->SR1 & I2C_SR1_ADDR)
	{
		_i2c_ClearFlags(i2c);
		//Master
		if(i2c->SR2 & I2C_SR2_MSL)
		{
			i2c->DR   = i2cIt->slaveRegAddr;//Передача адреса в который хотим записать.
			i2c->CR2 |= I2C_CR2_ITBUFEN;	//Разрешение прерывания по опустощению буфера Tx.
		}
		//Slave
		else
		{
			//От Мастера пришла команда на чтение - SLA+R
			if(i2c->SR2 & I2C_SR2_TRA)
			{
				i2c->DR = *(i2cIt->pTxBuf + 0);
				i2cIt->txBufIndex = 1;
			}
			//От Мастера пришла команда на запись - SLA+W
//			else
//			{
//				*(i2cIt->pRxBuf + 0) = i2c->DR; //Сохранили первый байт
//				i2cIt->rxBufIndex = 1;
//			}
		}
		//return;
	}
	//------------------------------
	//Передающий буфер свободен.
	if(i2c->SR1 & I2C_SR1_TXE)
	{
		_i2c_ClearFlags(i2c);//сбрасываем бит TXE (чтением SR1 и SR2):
		//Master
		if(i2c->SR2 & I2C_SR2_MSL)
		{
			if(i2cIt->txBufIndex < i2cIt->txBufSize)
			{
				i2c->DR = *(i2cIt->pTxBuf + i2cIt->txBufIndex);
				i2cIt->txBufIndex++;
			}
			else
			{
				i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Запрет прерывания по опостощению буфера Tx.
				i2c->CR1 |= I2C_CR1_STOP;    //Формируем Stop
				i2cIt->i2cItState = 0;
				i2cIt->txBufIndex = 0;
			}
		}
		//Slave
		else
		{
			//Передаем очередной байт
			i2c->DR = *(i2cIt->pTxBuf + i2cIt->txBufIndex);
			i2cIt->txBufIndex++;
			//Проверка окончания приема пакета производится в I2C_IT_ER_Handler
		}
		//return;
	}
	//------------------------------
	//Принят байт.
	if(i2c->SR1 & I2C_SR1_RXNE)
	{
		//Складываем принятый байт в приемный буфер.
		if(i2cIt->rxBufIndex < I2C_IT_RX_BUF_LEN_MAX)
		{
			*(i2cIt->pRxBuf + i2cIt->rxBufIndex) = i2c->DR;
			i2cIt->rxBufIndex++;
		}





//		//Складываем принятый байт в приемный буфер.
//		if(i2cIt->rxBufIndex < i2cIt->rxBufSize)
//		{
//			*(i2cIt->pRxBuf + i2cIt->rxBufIndex) = i2c->DR;
//			i2cIt->rxBufIndex++;
//		}
//		//Приняли нужно количество байтов.
//		else
//		{
//			i2cIt->i2cRxCallback();//Обработка принятого пакета
//			i2cIt->i2cItState = 0;
//			i2cIt->rxBufIndex = 0;
//		}
		//return;
	}
	//------------------------------
	//STOP
	if(i2c->SR1 & I2C_SR1_STOPF)
	{
		_i2c_ClearFlags(i2c);	  //сбрасываем бит STOPF
		i2c->CR1 &= ~I2C_CR1_STOP;//

		i2cIt->i2cRxCallback();//Обработка принятого пакета
		i2cIt->i2cItState = 0;
		i2cIt->rxBufIndex = 0;
		return;
	}
	//------------------------------
	//Byte transfer finished
	if(i2c->SR1 & I2C_SR1_BTF)
	{
		return;
	}
	//------------------------------
}
//**********************************************************
//Обработчик прерывания ошибок I2C
static void I2C_IT_Error(I2C_IT_t *i2cIt){

	I2C_TypeDef *i2c = i2cIt->i2c;
	//LedPC13Toggel();
	//------------------------------
	//NACK - Acknowledge failure
	if(i2c->SR1 & I2C_SR1_AF)
	{
		//передали нужное количество байтов.
		if(i2cIt->txBufIndex >= i2cIt->txBufSize)
		{
			i2cIt->i2cTxCallback();//Обработки события передачи буфера
			i2cIt->i2cItState = 0;
			i2cIt->txBufIndex = 0;
		}
		i2c->SR1 &= ~I2C_SR1_AF; //Сброс AF.
		//i2c->CR1 |= I2C_CR1_STOP;//Формируем Stop
		return;
	}
	//------------------------------
	//Bus error
	if(i2c->SR1 & I2C_SR1_BERR)
	{
		i2c->SR1 &= ~I2C_SR1_BERR; //Сброс BERR.
	}
	//------------------------------
	//Arbitration loss (Master)
	if(i2c->SR1 & I2C_SR1_ARLO)
	{
		i2c->SR1 &= ~I2C_SR1_ARLO; //Сброс ARLO.
	}
	//------------------------------
	//Overrun/Underrun
	if(i2c->SR1 & I2C_SR1_OVR)
	{
		i2c->SR1 &= ~I2C_SR1_OVR; //Сброс OVR.
	}
	//------------------------------
	//PEC error
	if(i2c->SR1 & I2C_SR1_PECERR)
	{
		i2c->SR1 &= ~I2C_SR1_PECERR; //Сброс PECERR.
	}
	//------------------------------
	//Timeout/Tlow error
	if(i2c->SR1 & I2C_SR1_TIMEOUT)
	{
		i2c->SR1 &= ~I2C_SR1_TIMEOUT; //Сброс TIMEOUT.
	}
	//------------------------------
	//SMBus Alert
	if(i2c->SR1 & I2C_SR1_SMBALERT)
	{
		i2c->SR1 &= ~I2C_SR1_SMBALERT; //Сброс SMBALERT.
	}
	//------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//Обработчик прерывания событий I2C
void I2C_IT_EV_Handler(I2C_TypeDef *i2c){

	if(i2c == I2C1) I2C_IT_Event(i2c1_IT_Define);
	else			I2C_IT_Event(i2c2_IT_Define);
}
//Обработчик прерывания ошибок I2C
void I2C_IT_ER_Handler(I2C_TypeDef *i2c){

	if(i2c == I2C1) I2C_IT_Error(i2c1_IT_Define);
	else			I2C_IT_Error(i2c2_IT_Define);
}
//**********************************************************
////Обработчик прерывания событий I2C
//void I2C1_EV_Handler(void){
//
//	I2C_IT_EV_Handler(i2c1_IT_Define);
//}
////**********************************************************
////Обработчик прерывания ошибок I2C
//void I2C1_ER_Handler(void){
//
//	I2C_IT_ER_Handler(i2c1_IT_Define);
//}
////**********************************************************
////Обработчик прерывания событий I2C
//void I2C2_EV_Handler(void){
//
//	I2C_IT_EV_Handler(i2c2_IT_Define);
//}
////**********************************************************
////Обработчик прерывания ошибок I2C
//void I2C2_ER_Handler(void){
//
//	I2C_IT_ER_Handler(i2c2_IT_Define);
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**********************************Работа с I2C через DMA.**********************************
//I2C1_TX -> DMA1_Ch6
//I2C1_RX -> DMA1_Ch7

//I2C2_TX -> DMA1_Ch4
//I2C2_RX -> DMA1_Ch5

//static volatile I2C_DMA_State_t I2cDmaStateReg = I2C_DMA_NOT_INIT;
//*******************************************************************************************
void I2C_DMA_Init(I2C_IT_t *i2cIt){

	I2C_IT_Init(i2cIt);

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;//Enable the peripheral clock DMA1
	i2cIt->i2cDmaState = I2C_DMA_READY;
}
//**********************************************************
uint32_t I2C_DMA_State(I2C_IT_t *i2cIt){

	return i2cIt->i2cDmaState;
}
//**********************************************************
//I2C_DMA_State_t I2C_DMA_Write(I2C_TypeDef *i2c, uint32_t deviceAddr, uint32_t regAddr, uint8_t *pBuf, uint32_t size){
uint32_t I2C_DMA_Write(I2C_IT_t *i2cIt){

	DMA_Channel_TypeDef *dma;
	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	if(i2cIt->i2cDmaState != I2C_DMA_READY) return I2C_DMA_BUSY;

	__disable_irq();
	//DMA1Channel config
	if(i2c == I2C1) dma = DMA1_Channel6;//I2C1_TX -> DMA1_Ch6
	else				   dma = DMA1_Channel4;//I2C2_TX -> DMA1_Ch4

	dma->CCR  &= ~DMA_CCR_EN;		  		//Channel disable
	dma->CPAR  = (uint32_t)&(i2c->DR);		//Peripheral address.
	dma->CMAR  = (uint32_t)i2cIt->pTxBuf;	//Memory address.
	dma->CNDTR = i2cIt->txBufSize;	  		//Data size.
	dma->CCR   = (3 << DMA_CCR_PL_Pos)   | 	//PL[1:0]: Channel priority level - 11: Very high.
			     (0 << DMA_CCR_PSIZE_Pos)| 	//PSIZE[1:0]: Peripheral size - 00: 8-bits.
				 (0 << DMA_CCR_MSIZE_Pos)| 	//MSIZE[1:0]: Memory size     - 00: 8-bits.
			     DMA_CCR_MINC |			 	//MINC: Memory increment mode - Memory increment mode enabled.
				 DMA_CCR_DIR  |           	//DIR:  Data transfer direction: 1 - Read from memory.
				 //DMA_CCR_CIRC | 		 	//CIRC: Circular mode
				 //DMA_CCR_TEIE | 		 	//TEIE: Transfer error interrupt enable
				 //DMA_CCR_HTIE | 		 	//HTIE: Half transfer interrupt enable
				 DMA_CCR_TCIE;// | 		 	//TCIE: Transfer complete interrupt enable
				 //DMA_CCR_EN;			 	//EN: Channel enable
	NVIC_SetPriority(DMA1_Channel6_IRQn, 0);//Set priority
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);     //Enable DMA1_Channel6_IRQn

	i2c->CR2 |= I2C_CR2_DMAEN;//DMAEN(DMA requests enable)
	//Формирование Start + AddrSlave|Write.
//	if(I2C_StartAndSendDeviceAddr(i2c, (i2cIt->slaveAddr<<1)|I2C_MODE_WRITE) != I2C_OK)
//	{
//		__enable_irq();
//		return I2C_DMA_NAC;
//	}
//	//Передача адреса в который хотим записать.
//	i2c->DR = i2cIt->slaveRegAddr;

	dma->CCR |= DMA_CCR_EN;//DMA Channel enable
	i2cIt->i2cDmaState = I2C_DMA_BUSY;
	__enable_irq();
	return I2C_DMA_BUSY;
}
//**********************************************************
uint32_t I2C_DMA_Read (I2C_IT_t *i2cIt){

	DMA_Channel_TypeDef *dma;
	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	if(i2cIt->i2cDmaState != I2C_DMA_READY) return I2C_DMA_BUSY;

	__disable_irq();
	//DMA1Channel config
	if(i2c == I2C1) dma = DMA1_Channel7;//I2C1_RX -> DMA1_Ch7
	else			dma = DMA1_Channel5;//I2C2_RX -> DMA1_Ch5

	dma->CCR  &= ~DMA_CCR_EN;		  	 	//Channel disable
	dma->CPAR  = (uint32_t)&(i2c->DR);	 	//Peripheral address.
	dma->CMAR  = (uint32_t)i2cIt->pRxBuf; 	//Memory address.
	dma->CNDTR = i2cIt->rxBufSize;	  	 	//Data size.
	dma->CCR   = (3 << DMA_CCR_PL_Pos)   | 	//PL[1:0]: Channel priority level - 11: Very high.
			     (0 << DMA_CCR_PSIZE_Pos)| 	//PSIZE[1:0]: Peripheral size - 00: 8-bits.
				 (0 << DMA_CCR_MSIZE_Pos)| 	//MSIZE[1:0]: Memory size     - 00: 8-bits.
				 DMA_CCR_MINC |			 	//MINC: Memory increment mode - Memory increment mode enabled.
				 //DMA_CCR_DIR  |           //DIR:  Data transfer direction: 1 - Read from memory.
				 //DMA_CCR_CIRC | 		    //CIRC: Circular mode
				 //DMA_CCR_TEIE | 		    //TEIE: Transfer error interrupt enable
				 //DMA_CCR_HTIE | 		    //HTIE: Half transfer interrupt enable
				 DMA_CCR_TCIE;// | 		    //TCIE: Transfer complete interrupt enable
				//DMA_CCR_EN;			    //EN: Channel enable
	NVIC_SetPriority(DMA1_Channel7_IRQn, 0);//Set priority
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);     //Enable DMA1_Channel6_IRQn

	i2c->CR2 |= I2C_CR2_DMAEN | //DMA Requests Enable.
				I2C_CR2_LAST;	//DMA Last Transfer.
	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, (i2cIt->slaveAddr<<1)|I2C_MODE_READ) != I2C_OK)
	{
		__enable_irq();
		return I2C_DMA_NAC;
	}
	i2c->CR1 |= I2C_CR1_ACK;//to be ready for another reception

	dma->CCR |= DMA_CCR_EN;//DMA Channel enable
	i2cIt->i2cDmaState = I2C_DMA_BUSY;
	__enable_irq();
	return I2C_DMA_BUSY;
}
//*******************************************************************************************
//*******************************************************************************************
//Прерываение от DMA1.
//I2C1_TX -> DMA1_Ch6
//I2C1_RX -> DMA1_Ch7

//I2C2_TX -> DMA1_Ch4
//I2C2_RX -> DMA1_Ch5

//**********************************************************
static void DMA_ChDisableAndITFlagClear(DMA_Channel_TypeDef *dma, uint32_t flag){

	DMA1->IFCR |=  flag;      //сбросить флаг окончания обмена.
	dma->CCR   &= ~DMA_CCR_EN;//отключение канала DMA.
}
//**********************************************************
static void I2C_DMA_TX_Handler(I2C_IT_t *i2cIt){

	//-------------------------
	//Обмен завершен.
	if(DMA1->ISR & DMA_ISR_TCIF6)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel6, DMA_IFCR_CTCIF6);
		//Ожидаем окончания передачи последних байтов.
		while(!(i2cIt->i2c->SR1 & I2C_SR1_BTF)){};//Ждем отпускания флага.
		i2cIt->i2c->CR1 |= I2C_CR1_STOP | //Формируем Stop
					       I2C_CR1_ACK;
		i2cIt->i2cDmaState = I2C_DMA_READY;
	}
	//-------------------------
	//Передана половина буфера
	if(DMA1->ISR & DMA_ISR_HTIF6)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF6;//сбросить флаг.
	}
	//-------------------------
	//Произошла ошибка при обмене
	if(DMA1->ISR & DMA_ISR_TEIF6)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel6 ,DMA_IFCR_CTEIF6);
		i2cIt->i2cDmaState = I2C_DMA_READY;
	}
	//-------------------------

}
//**********************************************************
static void I2C_DMA_RX_Handler(I2C_IT_t *i2cIt){

	//-------------------------
	//Обмен завершен.
	if(DMA1->ISR & DMA_ISR_TCIF7)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel7, DMA_IFCR_CTCIF7);

//		//Ожидаем окончания приема последних байтов.
//		if(I2C_LongWait(I2C1, I2C_SR1_BTF) != 0)
//		{
//			I2C1->CR1 |= I2C_CR1_STOP | //Формируем Stop
//					     I2C_CR1_ACK;
//		}

		i2cIt->i2c->CR1 |= I2C_CR1_STOP | //Формируем Stop
				     I2C_CR1_ACK;
		i2cIt->i2cDmaState = I2C_DMA_READY;
	}
	//-------------------------
	//Передана половина буфера
	if(DMA1->ISR & DMA_ISR_HTIF7)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF7;//сбросить флаг.
	}
	//-------------------------
	//Произошла ошибка при обмене
	if(DMA1->ISR & DMA_ISR_TEIF7)
	{
		DMA_ChDisableAndITFlagClear(DMA1_Channel7, DMA_IFCR_CTEIF7);
		i2cIt->i2cDmaState = I2C_DMA_READY;
	}
	//-------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//I2C1_TX -> DMA1_Ch6
void I2C1_IT_DMA_TX_Handler(void){

	I2C_DMA_TX_Handler(i2c1_IT_Define);


//	//-------------------------
//	//Обмен завершен.
//	if(DMA1->ISR & DMA_ISR_TCIF6)
//	{
//		DMA_ChDisableAndITFlagClear(DMA1_Channel6, DMA_IFCR_CTCIF6);
//		//Ожидаем окончания передачи последних байтов.
//		while(!(I2C1->SR1 & I2C_SR1_BTF)){};//Ждем отпускания флага.
//		I2C1->CR1 |= I2C_CR1_STOP | //Формируем Stop
//					 I2C_CR1_ACK;
//
//		I2cDmaStateReg = I2C_DMA_READY;
//	}
//	//-------------------------
//	//Передана половина буфера
//	if(DMA1->ISR & DMA_ISR_HTIF6)
//	{
//		DMA1->IFCR |= DMA_IFCR_CHTIF6;//сбросить флаг.
//	}
//	//-------------------------
//	//Произошла ошибка при обмене
//	if(DMA1->ISR & DMA_ISR_TEIF6)
//	{
//		DMA_ChDisableAndITFlagClear(DMA1_Channel6 ,DMA_IFCR_CTEIF6);
//		I2cDmaStateReg = I2C_DMA_READY;
//	}
//	//-------------------------
}
//**********************************************************
//I2C1_RX -> DMA1_Ch7
void I2C1_IT_DMA_RX_Handler(void){

	I2C_DMA_RX_Handler(i2c1_IT_Define);

//	//-------------------------
//	//Обмен завершен.
//	if(DMA1->ISR & DMA_ISR_TCIF7)
//	{
//		DMA_ChDisableAndITFlagClear(DMA1_Channel7, DMA_IFCR_CTCIF7);
//
////		//Ожидаем окончания приема последних байтов.
////		if(I2C_LongWait(I2C1, I2C_SR1_BTF) != 0)
////		{
////			I2C1->CR1 |= I2C_CR1_STOP | //Формируем Stop
////					     I2C_CR1_ACK;
////		}
//
//		I2C1->CR1 |= I2C_CR1_STOP | //Формируем Stop
//				     I2C_CR1_ACK;
//		I2cDmaStateReg = I2C_DMA_READY;
//	}
//	//-------------------------
//	//Передана половина буфера
//	if(DMA1->ISR & DMA_ISR_HTIF7)
//	{
//		DMA1->IFCR |= DMA_IFCR_CHTIF7;//сбросить флаг.
//	}
//	//-------------------------
//	//Произошла ошибка при обмене
//	if(DMA1->ISR & DMA_ISR_TEIF7)
//	{
//		DMA_ChDisableAndITFlagClear(DMA1_Channel7, DMA_IFCR_CTEIF7);
//		I2cDmaStateReg = I2C_DMA_READY;
//	}
//	//-------------------------
}
//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************















