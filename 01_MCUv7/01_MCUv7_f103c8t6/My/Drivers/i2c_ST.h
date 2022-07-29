/*
 * i2c_ST.h
 *
 *  Created on: 20 дек. 2020 г.
 *      Author: Zver
 */

#ifndef I2C_ST_H_
#define I2C_ST_H_
//*******************************************************************************************
//*******************************************************************************************
#include "main.h"

//*******************************************************************************************
#define I2C_BAUD_RATE	400000   //Частота в Гц
#define I2C_TRISE       300		 //значение в нС
#define APB1_CLK		36000000 //Частота шины APB1 в Гц



//#define I2C_CR2_VALUE   (APB1_CLK / 1000000)
//#define I2C_CCR_VALUE	((APB1_CLK / I2C_BAUD_RATE) / 2)
//#define I2C_TRISE_VALUE ((1 / (I2C_TRISE * (APB1_CLK / 1000000))) + 1)
#define I2C_WAIT_TIMEOUT	5000U
//--------------------------
#define I2C_MODE_READ  			1
#define I2C_MODE_WRITE 			0
#define I2C_ADDRESS(addr, mode) ((addr<<1) | mode)
//--------------------------
#define I2C_GPIO_NOREMAP	0
#define I2C_GPIO_REMAP		1

#define I2C_MODE_MASTER	0
#define I2C_MODE_SLAVE	1

//--------------------------
typedef enum{
	I2C_OK = 0,
	I2C_BUSY,		//Шина I2C занята (передача/прием данных)
	I2C_ERR_START,	//Ошибка при фоормировании Старт-последовательности
	I2C_ERR_ADDR,	//Ошибка адреса, Slave не отвечает.
	I2C_ERR_TX_BYTE,//Вышел таймаут передачи байта.
	I2C_ERR_RX_BYTE,//Вышел таймаут приема байта.
	I2C_ERR_BTF		//Вышел таймаут Byte transfer finished
}I2C_State_t;
//--------------------------

//*******************************************************************************************
//*******************************************************************************************
//Общие функции.
I2C_State_t I2C_StartAndSendDeviceAddr(I2C_TypeDef *i2c, uint32_t deviceAddr);
I2C_State_t I2C_SendByte(I2C_TypeDef *i2c, uint8_t byte);
I2C_State_t I2C_ReadData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len);
void		I2C_Stop(I2C_TypeDef *i2c);

I2C_State_t I2C_SendData(I2C_TypeDef *i2c, uint8_t *pBuf, uint32_t len);
//************************************
//Функции для работы в режиме Master
void 		I2C_Master_Init(I2C_TypeDef *i2c, uint32_t remap);
uint32_t 	I2C_Master_GetNacCount(I2C_TypeDef *i2c);

I2C_State_t I2C_Master_Write(I2C_TypeDef *i2c, uint32_t deviceAddr, uint32_t regAddr, uint8_t *pBuf, uint32_t len);
I2C_State_t I2C_Master_Read (I2C_TypeDef *i2c, uint32_t deviceAddr, uint32_t regAddr, uint8_t *pBuf, uint32_t len);

//************************************
//Функции для работы в режиме Slave
void I2C_Slave_Init(I2C_TypeDef *i2c, uint32_t remap, uint32_t slaveAddr);

//*******************************************************************************************
//*******************************************************************************************
//Работа по прерываниям.


//--------------------------
//Структура контекста для работы с портом I2C по прерываниям.
typedef struct{
	I2C_TypeDef *i2c;
	uint32_t 	i2cMode;		// Master или Slave
	uint32_t 	i2cGpioRemap;	// Ремап выводов для I2C1, для I2C2 ремапа нет.
	uint32_t 	i2cItState;		//
	uint32_t 	i2cDmaState;	//

	uint32_t 	slaveAddr;		// В режиме Master - адрес Slave-устройства к которому идет обращение,
								// в режиме Slave  - адрес устройста на шине.

	uint32_t 	slaveRegAddr;	// В режиме Master - адрес регистра Slave-устройства куда хотим писать/читать данные.
								// в режиме Slave  - ???

	uint8_t 	*pTxBuf;		// указатель на буфер передачи.
	uint32_t 	txBufSize;		// размер буфера передачи
	uint32_t	txBufIndex;		// индекс буфера передачи.

	uint8_t 	*pRxBuf;		// указатель на буфер приема.
	uint32_t 	rxBufSize;		// размер буфера приема.
	uint32_t	rxBufIndex;		// индекс буфера приема.

	void(*i2cRxCallback)(void);
	void(*i2cTxCallback)(void);

}I2C_IT_t;
//--------------------------

//*******************************************************************************************
void I2C_IT_Init(I2C_IT_t *i2c);

//Обработчики прерывания
void I2C_IT_EV_Handler(I2C_IT_t *i2cIt); //Обработчик прерывания событий I2C
void I2C_IT_ER_Handler(I2C_IT_t *i2cIt); //Обработчик прерывания ошибок I2C

//*******************************************************************************************
//*******************************************************************************************
//Работа чере DMA.
#define I2C1_TX_DMAChannel	DMA1_Channel6
#define I2C1_RX_DMAChannel	DMA1_Channel7

#define I2C2_TX_DMAChannel	DMA1_Channel4
#define I2C2_RX_DMAChannel	DMA1_Channel5

typedef enum{
	I2C_DMA_READY = 0,	//I2C и DMA готовы к передаче данных.
	I2C_DMA_NOT_INIT,	//I2C и DMA не инициализированны.
	I2C_DMA_NAC,		//Slave не ответил на свой адрес.
	I2C_DMA_BUSY,		//I2C и DMA заняты, идет передача/прием данных.
	I2C_DMA_ERR			//Ошибка DMA.
}I2C_DMA_State_t;
//*******************************************************************************************
void 	 I2C_DMA_Init(I2C_IT_t *i2cIt);
uint32_t I2C_DMA_State(I2C_IT_t *i2cIt);
uint32_t I2C_DMA_Write(I2C_IT_t *i2cIt);
uint32_t I2C_DMA_Read (I2C_IT_t *i2cIt);
//*******************************************************************************************
//*******************************************************************************************
#endif /* I2C_ST_H_ */






















