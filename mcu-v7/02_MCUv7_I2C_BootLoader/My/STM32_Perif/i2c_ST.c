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
static uint32_t _i2c_LongWait(I2C_TypeDef *i2c, uint32_t flag){

	uint32_t count = 0;
	//---------------------
	while(!(i2c->SR1 & flag))//Ждем отпускания флага.
	{
		if(++count >= I2C_WAIT_TIMEOUT) return 1;
	}
	return 0;
}
//**********************************************************
static void _i2c_GPIO_Init(I2C_TypeDef *i2c, uint32_t remap){

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;//Включаем тактирование GPIOB
	//Тактирование I2C_1
	if(i2c == I2C1)
	{
		//Ремап:
		//I2C1_SCL - PB8,
		//I2C1_SDA - PB9.
		if(remap == I2C_GPIO_REMAP)
		{
			AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
			GPIOB->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1 |
						  GPIO_CRH_CNF8    | GPIO_CRH_CNF9;
		}
		//I2C1_SCL - PB6,
		//I2C1_SDA - PB7.
		else
		{
			GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1 |
						  GPIO_CRL_CNF6    | GPIO_CRL_CNF7;
		}
	}
	//Тактирование I2C_2
	else if(i2c == I2C2)
	{
		//I2C2_SCL - PB10,
		//I2C2_SDA - PB11.
		GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_MODE11_1 |
					  GPIO_CRH_CNF10    | GPIO_CRH_CNF11;
	}
}
//**********************************************************
static void _i2c_ClockEnable(I2C_TypeDef *i2c){

	if(i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	else		    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
}
//**********************************************************
static void _i2c_ModeInit(I2C_TypeDef *i2c, uint32_t mode){

	//Инициализация I2C.
	i2c->CR1 &= ~I2C_CR1_PE;    //Откл. модуля I2C.
	i2c->CR1 |=  I2C_CR1_SWRST; //Программный сброс модуля I2C
	i2c->CR1 &= ~I2C_CR1_SWRST; //Это нужно для востановления работоспособноси после КЗ на линии.
	i2c->CR1 |=  I2C_CR1_PE;    //Включение модуля I2C1.
	i2c->CR1 &= ~I2C_CR1_SMBUS; //модуль работает в режиме I2C
	i2c->SR1  = 0; 			    //Сброс флагов ошибок.

	if(mode == I2C_MODE_MASTER) i2c->SR2 |=  I2C_SR2_MSL;//режим Master.
	else						i2c->SR2 &= ~I2C_SR2_MSL;//режим Slave
}
//**********************************************************
static void _i2c_SetSlaveAddress(I2C_TypeDef *i2c, uint8_t slaveAddr){

	i2c->OAR1 = (slaveAddr << 1);//адрес устройства на шине.
	i2c->CR1 |= I2C_CR1_ACK;	 //разрешаем отправлять ACK/NACK после приема байта адреса.
}
//**********************************************************
static void _i2c_SetSpeed(I2C_TypeDef *i2c, uint32_t speed){

	i2c->CR2  = 0;
	i2c->CR2 |= (I2C_FREQ << I2C_CR2_FREQ_Pos);//APB1 = 36MHz
	i2c->CCR  = 0;
	//FastMode(400kHz)
	if(speed >= 400000)
	{
		i2c->CCR   |=  I2C_CCR_FS; //1 - режим FastMode(400kHz).
		i2c->CCR   |= (I2C_FM_CCR << I2C_CCR_CCR_Pos);
		i2c->TRISE |= (I2C_FM_TRISE << I2C_TRISE_TRISE_Pos);
	}
	//StandartMode(100kHz).
	else
	{
		i2c->CCR   &= ~I2C_CCR_FS; //0 - режим STANDART(100kHz).
		i2c->CCR   |= (I2C_SM_CCR << I2C_CCR_CCR_Pos);
		i2c->TRISE |= (I2C_SM_TRISE << I2C_TRISE_TRISE_Pos);
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**************************Общие функции для работы с I2C***********************************
// !!! НЕОТЛАЖЕНО !!!
I2C_State_t I2C_StartAndSendDeviceAddr(I2C_TypeDef *i2c, uint8_t deviceAddr){
//
//	//Формирование Start condition.
//	i2c->CR1 |= I2C_CR1_START;
//	if(_i2c_LongWait(i2c, I2C_SR1_SB)) return I2C_ERR_START;//Ожидание формирования Start condition.
//	(void)i2c->SR1;			   		//Для сброса флага SB необходимо прочитать SR1
//
//	//Передаем адрес.
//	i2c->DR = deviceAddr;
//	if(_i2c_LongWait(i2c, I2C_SR1_ADDR))//Ожидаем окончания передачи адреса
//	{
//		i2c->SR1 &= ~I2C_SR1_AF;  //Сброс флагов ошибок.
//		if(i2c == I2C1) I2C1NacCount++;
//		else			I2C2NacCount++;
//
//		return I2C_ERR_ADDR;
//	}
//	(void)i2c->SR1;	//сбрасываем бит ADDR (чтением SR1 и SR2):
//	(void)i2c->SR2;	//
	return I2C_OK;
}
////**********************************************************
//I2C_State_t I2C_SendByte(I2C_TypeDef *i2c, uint8_t byte){
//
//	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
//	i2c->DR = byte;
//	return I2C_OK;
//}
////**********************************************************
// !!! НЕОТЛАЖЕНО !!!
I2C_State_t I2C_ReadData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){
//
//	//receiving 1 byte
//	if(len == 1)
//	{
//		i2c->CR1 &= ~I2C_CR1_ACK;				   //Фомирование NACK.
//		i2c->CR1 |= I2C_CR1_STOP;				   //Формируем Stop.
//		if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;//ожидаем окончания приема байта
//		*(pBuf + 0) = i2c->DR;				       //считали принятый байт.
//		i2c->CR1 |= I2C_CR1_ACK;				   //to be ready for another reception
//	}
//	//receiving 2 bytes
//	else if(len == 2)
//	{
//		i2c->CR1 |=  I2C_CR1_POS;//
//		i2c->CR1 &= ~I2C_CR1_ACK;//Фомирование NACK.
//		if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
//		i2c->CR1 |= I2C_CR1_STOP;//Формируем Stop.
//		*(pBuf + 0) = i2c->DR;	 //считали принятый байт.
//		*(pBuf + 1) = i2c->DR;	 //считали принятый байт.
//		i2c->CR1 &= ~I2C_CR1_POS;//
//		i2c->CR1 |= I2C_CR1_ACK; //to be ready for another reception
//	}
//	//receiving more than 2 bytes
//	else
//	{
//		uint32_t i;
//		for(i=0; i<(len-3); i++)
//		{
//			if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;
//			*(pBuf + i) = i2c->DR;//Read DataN
//		}
//		//Вычитываем оставшиеся 3 байта.
//		if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
//		i2c->CR1 &= ~I2C_CR1_ACK; //Фомирование NACK
//		*(pBuf + i + 0) = i2c->DR;//Read DataN-2
//		i2c->CR1 |= I2C_CR1_STOP; //Формируем Stop
//		*(pBuf + i + 1) = i2c->DR;//Read DataN-1
//		if(_i2c_LongWait(i2c, I2C_SR1_RXNE)) return I2C_ERR_RX_BYTE;
//		*(pBuf + i + 2) = i2c->DR;//Read DataN
//		i2c->CR1 |= I2C_CR1_ACK;
//	}
	return I2C_OK;
}
////**********************************************************
//void I2C_Stop(I2C_TypeDef *i2c){
//
//	if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return;
////	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return;
//	i2c->CR1 |= I2C_CR1_STOP;		 //Формируем Stop
//}
////**********************************************************
// !!! НЕОТЛАЖЕНО !!!
I2C_State_t I2C_SendDataWithStop(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){
//
//	for(uint32_t i = 0; i < len; i++)
//	{
//		if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
//		i2c->DR = *(pBuf + i);
//	}
//	I2C_Stop(i2c);	 //Формируем Stop
	return I2C_OK;
}
//**********************************************************
// !!! НЕОТЛАЖЕНО !!!
I2C_State_t I2C_SendDataWithoutStop(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len){

	for(uint32_t i = 0; i < len; i++)
	{
		if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;//Ждем освобождения буфера
		i2c->DR = *(pBuf + i);
	}
	//if(_i2c_LongWait(i2c, I2C_SR1_BTF)) return I2C_ERR_BTF;
	return I2C_OK;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//**************************Функции для работы в режиме Master*******************************
void I2C_Master_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t speed){

	_i2c_GPIO_Init(i2c, remap);		    //Инициализация портов
	_i2c_ClockEnable(i2c);			    //Вкл. тактирования модуля I2C
	_i2c_ModeInit(i2c, I2C_MODE_MASTER);//Инициализация I2C в режиме Master.
	_i2c_SetSpeed(i2c, speed);			//Скорость работы.

	//Включение модуля I2C.
	i2c->CR1 |= I2C_CR1_PE;
}
//**********************************************************
uint32_t I2C_Master_GetNacCount(I2C_TypeDef *i2c){

	if(i2c == I2C1) return I2C1NacCount;
					return I2C2NacCount;
}
//**********************************************************
uint32_t I2C_Master_CheckSlave(I2C_TypeDef *i2c, uint8_t deviceAddr){

	uint32_t err = I2C_StartAndSendDeviceAddr(i2c, deviceAddr);
	i2c->CR1 |= I2C_CR1_STOP; //Формируем Stop

	if(err == I2C_OK) return 1;
					  return 0;
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
I2C_State_t I2C_Master_Write(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint32_t len){

	//Формирование Start + AddrSlave|Write.
	if(I2C_StartAndSendDeviceAddr(i2c, deviceAddr|I2C_MODE_WRITE) != I2C_OK) return I2C_ERR_ADDR;

	//Передача адреса в который хотим записать.
	i2c->DR = regAddr;
	if(_i2c_LongWait(i2c, I2C_SR1_TXE)) return I2C_ERR_TX_BYTE;

	//передача данных на запись.
	return(I2C_SendDataWithStop(i2c, pBuf, len));
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
I2C_State_t I2C_Master_Read(I2C_TypeDef *i2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *pBuf, uint32_t len){

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
void I2C_Slave_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t slaveAddr, uint32_t speed){

	_i2c_GPIO_Init(i2c, remap);		    		  //Инициализация портов
	_i2c_ClockEnable(i2c);			    		  //Вкл. тактирования модуля I2C
	_i2c_ModeInit(i2c, I2C_MODE_SLAVE); 		  //I2C в режиме Slave.
	_i2c_SetSlaveAddress(i2c, (uint8_t)slaveAddr);//адрес устройства на шине.
	_i2c_SetSpeed(i2c, speed);					  //Скорость работы.

	//i2c->CR1 &= ~I2C_CR1_NOSTRETCH;

	//Включение модуля I2C.
	//i2c->CR1 |= I2C_CR1_PE;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//****************************Работа I2C по прерываниям.*************************************
/*
 * Указатели на контекст шины I2C. Они нужны для работы обработчиков прерываний.
 */
static I2C_IT_t	*i2c1_IT_Define;
static I2C_IT_t	*i2c2_IT_Define;
//*******************************************************************************************
//*******************************************************************************************
void I2C_IT_Init(I2C_IT_t *i2cIt){

		 if(i2cIt->i2c == I2C1)  i2c1_IT_Define = i2cIt;
	else if(i2cIt->i2c == I2C2)  i2c2_IT_Define = i2cIt;
	else return;

	//Инициализация I2C
	if(i2cIt->i2cMode == I2C_MODE_MASTER) I2C_Master_Init(i2cIt->i2c, i2cIt->gpioRemap, i2cIt->i2cSpeed);
	else								  I2C_Slave_Init (i2cIt->i2c, i2cIt->gpioRemap, i2cIt->slaveAddr, i2cIt->i2cSpeed);

	//Инициализация прерывания.
	I2C_IT_InterruptEnable(i2cIt);

	if(i2cIt->i2c == I2C1)
	{
		//Приоритет прерывания.
		NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		//Разрешаем прерывание.
		NVIC_EnableIRQ(I2C1_EV_IRQn);
		NVIC_EnableIRQ(I2C1_ER_IRQn);
	}
	else
	{
		//Пример...
		///* 2 bits for pre-emption priority and 2 bits for subpriority */
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		//
		///* Set USART1 interrupt preemption priority to 1 */
		//NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		//
		///* Set SysTick interrupt preemption priority to 3 */
		//NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));

		//Приоритет прерывания.
		NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		//Разрешаем прерывание.
		NVIC_EnableIRQ(I2C2_EV_IRQn);
		NVIC_EnableIRQ(I2C2_ER_IRQn);
	}
	i2cIt->state = I2C_IT_STATE_READY;
	//Включение модуля I2C.
	i2cIt->i2c->CR1 |= I2C_CR1_PE;
}
//**********************************************************
void I2C_IT_DeInit(I2C_IT_t *i2cIt){

	//Отключение модуля I2C.
	i2cIt->i2c->CR1 &= ~I2C_CR1_PE;

	//Отключение
	if(i2cIt->i2c == I2C1)
	{
		NVIC_DisableIRQ(I2C1_EV_IRQn);
		NVIC_DisableIRQ(I2C1_ER_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(I2C2_EV_IRQn);
		NVIC_DisableIRQ(I2C2_ER_IRQn);
	}
	i2cIt->state = I2C_IT_STATE_RESET;
}
//**********************************************************
void I2C_IT_InterruptDisable(I2C_IT_t *i2cIt){

	//Откл. прерываний
	i2cIt->i2c->CR2 &= ~(I2C_CR2_ITEVTEN | //прерывания по событию.
				  	  	 I2C_CR2_ITBUFEN | //прерывания по приему/передаче байта
						 I2C_CR2_ITERREN); //прерывания по ошибкам.
}
//**********************************************************
void I2C_IT_InterruptEnable(I2C_IT_t *i2cIt){

	//Включение прерываний
	i2cIt->i2c->CR2 |= I2C_CR2_ITEVTEN | //прерывания по событию.
				  	   I2C_CR2_ITBUFEN | //прерывания по приему/передаче байта
					   I2C_CR2_ITERREN;  //прерывания по ошибкам.
}
//**********************************************************
void I2C_IT_SetpBuf(I2C_IT_t *i2cIt, uint8_t *pBuf){

	i2cIt->pBuf = pBuf;
}
//**********************************************************
void I2C_IT_SetDataSize(I2C_IT_t *i2cIt, uint32_t size){

	__disable_irq();
	i2cIt->bufSize = size;
	__enable_irq();
}
//**********************************************************
uint32_t I2C_IT_GetDataCount(I2C_IT_t *i2cIt){

	return i2cIt->bufCount;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//static void _i2c_IT_ReadByteToBuffer(I2C_IT_t *i2cIt){
//
//	i2cIt->itState = I2C_IT_STATE_BUSY_RX;
//	//Складываем принятый байт в приемный буфер.
//	*(i2cIt->rxBuf + i2cIt->rxBufIndex) = (uint8_t)i2cIt->i2c->DR;
//	i2cIt->rxBufIndex++;
//}
//**********************************************************
//static void _i2c_IT_WriteByteFromBuffer(I2C_IT_t *i2cIt){
//
//	i2cIt->state = I2C_IT_STATE_BUSY_TX;
//	//Передаем очередной байт
//	//i2cIt->i2c->DR = (uint8_t)*(i2cIt->txBuf + i2cIt->txBufIndex);
//	//i2cIt->txBufIndex++;
//
//	//i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
//	//i2cIt->bufCount++;
//
//	//Если есть что передавать то передаем.
//	if(i2cIt->bufSize > 0)
//	{
//		i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
//		i2cIt->bufCount++;
//		i2cIt->bufSize--;
//
//		//Если все передали - отключаем прерывание
//		if(i2cIt->bufSize == 0)
//		{
//			i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;
//		}
//	}
//}
//*******************************************************************************************
//*******************************************************************************************
/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__attribute__((weak)) void I2C_IT_SlaveTxCpltCallback(I2C_IT_t *i2cIt){


	/* Prevent unused argument(s) compilation warning */
	//UNUSED(hi2c);

	/* NOTE : This function should not be modified, when the callback is needed,
			the HAL_I2C_SlaveTxCpltCallback can be implemented in the user file
	*/
}
//**********************************************************
/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__attribute__((weak)) void I2C_IT_SlaveRxCpltCallback(I2C_IT_t *i2cIt){

	/* Prevent unused argument(s) compilation warning */
	//UNUSED(hi2c);

	/* NOTE : This function should not be modified, when the callback is needed,
			the HAL_I2C_SlaveRxCpltCallback can be implemented in the user file
	*/
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//***************************Обработчики прерывания******************************************
//static void I2C_IT_Master(I2C_IT_t *i2cIt){
//
//
//}
//*******************************************************************************************
//*******************************************************************************************
static void I2C_IT_Slave(I2C_IT_t *i2cIt){

//	После проверки содержимого регистра SR1 пользователь должен выполнить полную
//	последовательность очистки для каждого найденного набора флагов.
//	Таким образом, для флагов ADDR и STOPF внутри процедуры прерывания I2C требуется следующая последовательность:
//	READ SR1
//	if (ADDR == 1) {READ SR1; READ SR2}
//	if (STOPF == 1) {READ SR1; WRITE CR1}
//	Цель состоит в том, чтобы убедиться, что оба флага ADDR и STOPF сброшены, если они оба установлены.

	I2C_TypeDef *i2c = i2cIt->i2c;
	//---------------------
	(void)i2c->SR1;//рекомендация из даташита
	/*------------------------------------------------------------------------*/
	/* ADDR set --------------------------------------------------------------*/
	if((i2c->SR1 & I2C_SR1_ADDR) && (i2c->CR2 & I2C_CR2_ITEVTEN))
	{
		//Рекомендация из RM0008: if (ADDR == 1) {READ SR1; READ SR2}
		(void)i2c->SR1;				//сбрасываем I2C_IT_ADDR чтением SR1 и SR2
		(void)i2c->SR2;
		i2cIt->bufCount = 0;		//Сброс счетчика байтов

		//Принят адрес+Rd - мастер читает данные
		if(i2c->SR2 & I2C_SR2_TRA)
		{
			i2cIt->state = I2C_IT_STATE_ADDR_RD;
			//Если есть что передавать то передаем.
			if(i2cIt->bufSize > 0)
			{
				i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
				i2cIt->bufCount++;
				//i2cIt->bufSize--;
				//Если все передали - отключаем прерывание
				//if(i2cIt->bufSize == 0)
				//{
				//	i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;
				//}
			}
		}
		//Принят адрес+Wr - мастер записывает данные
		else
		{
			i2cIt->state = I2C_IT_STATE_ADDR_WR;
		}
	}
	/*------------------------------------------------------------------------*/
	/* STOPF set -------------------------------------------------------------*/
	else if((i2c->SR1 & I2C_SR1_STOPF) && (i2c->CR2 & I2C_CR2_ITEVTEN))
	{
		//Рекомендация из RM0008: if (STOPF == 1) {READ SR1; WRITE CR1}
		(void)i2c->SR1;						//EV4: STOPF = 1, сбрасывается чтением регистра SR1
		i2c->CR1 |= 0x1;					//и записью в регистр CR1
		i2cIt->state = I2C_IT_STATE_STOP;	//От Мастери принят STOP

		I2C_IT_InterruptDisable(i2cIt);		//Откл. прерывания
		I2C_IT_SlaveRxCpltCallback(i2cIt);	//Обрабатываем принятые данные
	}
	/*------------------------------------------------------------------------*/
	/* I2C in mode Transmitter -----------------------------------------------*/
	else if(i2c->SR2 & I2C_SR2_TRA)//От Мастера пришла команда на чтение - SLA+Rd
	{
		i2cIt->state = I2C_IT_STATE_BUSY_TX;
		/* TXE set and BTF reset --------------------*/
		if((i2c->SR1 & I2C_SR1_TXE)     &&
		   (i2c->CR2 & I2C_CR2_ITBUFEN) &&
		  !(i2c->SR1 & I2C_SR1_BTF) )
		{
			//Если есть что передавать то передаем.
			if(i2cIt->bufSize > 0)
			{
				i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
				i2cIt->bufCount++;
				//i2cIt->bufSize--;
			}
			//Если все передали - отключаем прерывание
			if(i2cIt->bufSize == 0)
			{
				i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;
			}
		}
		/* BTF set ----------------------------------*/
		else if((i2c->SR1 & I2C_SR1_BTF) && (i2c->CR2 & I2C_CR2_ITEVTEN))
		{
			//Если есть что передавать то передаем.
			if(i2cIt->bufSize > 0)
			{
				i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
				i2cIt->bufCount++;
				//i2cIt->bufSize--;
			}
		}
		else{ /* Do nothing */ }
	}
	/*------------------------------------------------------------------------*/
	/* I2C in mode Receiver --------------------------------------------------*/
	else	//Принят адрес+Wr - мастер записывает данные
	{
		/* RXNE set and BTF reset -------------*/
		if((i2c->SR1 & I2C_SR1_RXNE)    &&
		   (i2c->CR2 & I2C_CR2_ITBUFEN) &&
		  !(i2c->SR1 & I2C_SR1_BTF))
		{
			i2cIt->state = I2C_IT_STATE_BUSY_RX;
			//EV2: RxNE = 1, сбрасывается чтением регистра DR
			//Складываем принятый байт в приемный буфер.
			*(i2cIt->pBuf + i2cIt->bufCount) = (uint8_t)i2cIt->i2c->DR;
			i2cIt->bufCount++;

		}
		/* BTF set ---------------------------*/
		else if((i2c->SR1 & I2C_SR1_BTF) && (i2c->CR2 & I2C_CR2_ITEVTEN))
		{
			i2cIt->state = I2C_IT_STATE_BUSY_RX;
			//EV2: RxNE = 1, сбрасывается чтением регистра DR
			//Складываем принятый байт в приемный буфер.
			*(i2cIt->pBuf + i2cIt->bufCount) = (uint8_t)i2cIt->i2c->DR;
			i2cIt->bufCount++;
		}
		else{ /* Do nothing */ }
	}
	/*------------------------------------------------------------------------*/
	/*------------------------------------------------------------------------*/
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//static void I2C_IT_Slave2(I2C_IT_t *i2cIt){
//
//	I2C_TypeDef *i2c = i2cIt->i2c;
//	volatile uint32_t flag1 = i2c->SR1;
//	volatile uint32_t flag2 = i2c->SR2;
//	volatile uint32_t event = 0;
//	//---------------------
//	//Собятия на шине I2C
//	flag2 = flag2 << 16;
//	event = (flag1 | flag2) & 0x00FFFFFF;
//	/*------------------------------------------------------------------------*/
//	/* I2C-SLAVE in mode Receiver --------------------------------------------*/
//	if(event == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)//От Мастера пришла команда на запись - SLA+Wr
//	{
//		(void)i2c->SR1;							//EV1: ADDR = 1, сбрасывается чтением регистров SR1
//		(void)i2c->SR2;							//и SR2
//		i2cIt->state = I2C_IT_STATE_ADDR_WR;	//Принят адрес+Wr - мастер записывает данные
//		//i2c->CR2 |= I2C_CR2_ITBUFEN;			//включаем прерывание I2C_IT_BUF - вначале было отключено
//		//i2cIt->rxBufIndex = 0;				//Сброс счетчика байтов RX
//		i2cIt->bufCount = 0;
//	}
//	//От Мастера принят байт данных.
//	else if(event == I2C_EVENT_SLAVE_BYTE_RECEIVED)
//	{
//		i2cIt->state = I2C_IT_STATE_BUSY_RX;
//		//EV2: RxNE = 1, сбрасывается чтением регистра DR
//		//Складываем принятый байт в приемный буфер.
//		//*(i2cIt->rxBuf + i2cIt->rxBufIndex) = (uint8_t)i2cIt->i2c->DR;
//		//i2cIt->rxBufIndex++;
//
//		*(i2cIt->pBuf + i2cIt->bufCount) = (uint8_t)i2cIt->i2c->DR;
//		i2cIt->bufCount++;
//	}
//	/*------------------------------------------------------------------------*/
//	/* I2C-SLAVE in mode Transmitter ------------------------------------------*/
//	else if(event == I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) //От Мастера пришла команда на чтение - SLA+Rd
//	{
//		(void)i2c->SR1;							//EV1: ADDR = 1, сбрасывается чтением регистров SR1
//		(void)i2c->SR2;							//и SR2
//		i2cIt->state = I2C_IT_STATE_ADDR_RD;	//Принят адрес+Rd - мастер читает данные
//		//i2c->CR2 |= I2C_CR2_ITBUFEN;			//включаем прерывание I2C_IT_BUF - вначале было отключено
//		//i2cIt->txBufIndex = 0;				//Сброс счетчика байтов TX
//		i2cIt->bufCount = 0;
//		//_i2c_IT_WriteByteFromBuffer(i2cIt);		//EV3-1: TxE = 1 ... - Передадим первый байт
//
//		//Если есть что передавать то передаем.
//		if(i2cIt->bufSize > 0)
//		{
//			i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
//			i2cIt->bufCount++;
//			i2cIt->bufSize--;
//
//			//Если все передали - отключаем прерывание
//			if(i2cIt->bufSize == 0)
//			{
//				i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;
//			}
//		}
//	}
//	//Слайв передал байт
//	else if(event == I2C_EVENT_SLAVE_BYTE_TRANSMITTED)	/* TRA, BUSY, TXE and BTF flags */
//	{
//		//Передаем очередной байт
//		//_i2c_IT_WriteByteFromBuffer(i2cIt);
//
//		//Если есть что передавать то передаем.
//		if(i2cIt->bufSize > 0)
//		{
//			i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
//			i2cIt->bufCount++;
//			i2cIt->bufSize--;
//		}
//	}
//	//Слайв передает байт
//	else if(event == I2C_EVENT_SLAVE_BYTE_TRANSMITTING) /* TRA, BUSY and TXE flags */
//	{
//		//Передаем очередной байт
//		//_i2c_IT_WriteByteFromBuffer(i2cIt);
//
//		//Если есть что передавать то передаем.
//		if(i2cIt->bufSize > 0)
//		{
//			i2cIt->i2c->DR = (uint8_t)*(i2cIt->pBuf + i2cIt->bufCount);
//			i2cIt->bufCount++;
//			i2cIt->bufSize--;
//
//			//Если все передали - отключаем прерывание
//			if(i2cIt->bufSize == 0)
//			{
//				i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;
//			}
//		}
//	}
//	/*------------------------------------------------------------------------*/
//	/* Мастер передал СТОП-----------------------------------------------------*/
//	else if(event==I2C_EVENT_SLAVE_STOP_DETECTED)
//	{
//		(void)i2c->SR1;						//EV4: STOPF = 1, сбрасывается чтением регистра SR1
//		i2c->CR1 |= 0x1;					//и записью в регистр CR1
//		i2cIt->state = I2C_IT_STATE_STOP;	//От Мастери принят STOP
//		i2cIt->i2c->CR2 &= ~I2C_CR2_ITBUFEN;//Отключаем прерывание
//		I2C_IT_SlaveRxCpltCallback(i2cIt);	//Обрабатываем принятые данные
//	}
//	/*------------------------------------------------------------------------*/
//	/*------------------------------------------------------------------------*/
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Сброс флага ошибки.
//static void _i2c_ClearFlag(I2C_TypeDef *i2c, uint32_t flag){
//
//	i2c->SR1 &= ~(flag);
//}
//**************************************************
//Обработчик прерывания ошибок I2C
static void I2C_IT_Error(I2C_IT_t *i2cIt){

	I2C_TypeDef *i2c = i2cIt->i2c;
	//------------------------------
	//NACK - Acknowledge failure
	//этот флаг устанавливается когда от Мастера приходит NACK при завершении чтения байтов.
	if(i2c->SR1 & I2C_SR1_AF)
	{
		i2c->SR1 &= ~I2C_SR1_AF;			//
		i2c->CR1 |= I2C_CR1_ACK;			//разрешаем генерацию NACK
		i2cIt->state = I2C_IT_STATE_NAC;	//от Мастера принят NACK -признак завершения чтения байтов.

		I2C_IT_InterruptDisable(i2cIt);		//Откл. прерывания
		I2C_IT_SlaveTxCpltCallback(i2cIt);	//Что нибудь делаем по окончению передачи данных.
	}
	//------------------------------
	//Bus error
	if(i2c->SR1 & I2C_SR1_BERR)
	{
		i2c->SR1 &= ~I2C_SR1_BERR; //Сброс BERR.
		//_i2c_ClearFlag(i2c, I2C_SR1_BERR);

		//Генерируем СТОП
		if(i2c->SR1 & I2C_SR1_STOPF)
		{
			(void)i2c->SR1;		//EV4: STOPF = 1, сбрасывается чтением регистра SR1
			i2c->CR1 |= 0x1;	//и записью в регистр CR1
		}
	}
	//------------------------------
	//Arbitration loss (Master)
	if(i2c->SR1 & I2C_SR1_ARLO)
	{
		i2c->SR1 &= ~I2C_SR1_ARLO; //Сброс ARLO.
		//_i2c_ClearFlag(i2c, I2C_SR1_ARLO);
	}
	//------------------------------
	//Overrun/Underrun
	if(i2c->SR1 & I2C_SR1_OVR)
	{
		i2c->SR1 &= ~I2C_SR1_OVR; //Сброс OVR.
		//_i2c_ClearFlag(i2c, I2C_SR1_OVR);
	}
	//------------------------------
	//PEC error
	if(i2c->SR1 & I2C_SR1_PECERR)
	{
		i2c->SR1 &= ~I2C_SR1_PECERR; //Сброс PECERR.
		//_i2c_ClearFlag(i2c, I2C_SR1_PECERR);
	}
	//------------------------------
	//Timeout/Tlow error
	if(i2c->SR1 & I2C_SR1_TIMEOUT)
	{
		i2c->SR1 &= ~I2C_SR1_TIMEOUT; //Сброс TIMEOUT.
		//_i2c_ClearFlag(i2c, I2C_SR1_TIMEOUT);
	}
	//------------------------------
	//SMBus Alert
	if(i2c->SR1 & I2C_SR1_SMBALERT)
	{
		i2c->SR1 &= ~I2C_SR1_SMBALERT; //Сброс SMBALERT.
		//_i2c_ClearFlag(i2c, I2C_SR1_SMBALERT);
	}
	//------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Обработчик прерывания событий I2C
void I2C_IT_EV_Handler(I2C_TypeDef *i2c){

	I2C_IT_t *i2cIt;
	//---------------------
	//Определение для какого I2C прерывание.
		 if(i2c == I2C1) i2cIt = i2c1_IT_Define;
	else if(i2c == I2C2) i2cIt = i2c2_IT_Define;
	else return;
	//Мастер или Слейв
//	if(i2cIt->i2cMode == I2C_MODE_MASTER) I2C_IT_Master(i2cIt);
//	else								  I2C_IT_Slave2(i2cIt);//I2C_IT_Slave(i2cIt);
	I2C_IT_Slave(i2cIt); //I2C_IT_Slave2(i2cIt);//
}
//**********************************************************
//Обработчик прерывания ошибок I2C
void I2C_IT_ER_Handler(I2C_TypeDef *i2c){

	if(i2c == I2C1) I2C_IT_Error(i2c1_IT_Define);
	else			I2C_IT_Error(i2c2_IT_Define);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************








