/*
 * 	main.c
 *
 *  Created on: 22.07.2022
 *  Autho     : Беляев А.А.
 *
 *
 *	Описание  : MCUv7
 *
 *	Последнее редактирование: 22.07.2022
 *
 */
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
DS18B20_t Sensor_1;
DS18B20_t Sensor_2;
DS18B20_t Sensor_3;


I2C_IT_t		I2cWire;
static uint8_t 	slaveTxBuf[32] = {0};
static uint8_t  slaveRxBuf[32] = {0};
//*******************************************************************************************
//*******************************************************************************************
uint32_t Led_Blink(uint32_t millis, uint32_t period, uint32_t switch_on_time){

	static uint32_t millisOld = 0;
	static uint32_t flag      = 0;
	//-------------------
	if((millis - millisOld) >= (flag ? (period - switch_on_time) : switch_on_time ))
	{
		millisOld = millis;
		flag = !flag;
	}
	return flag;
}
//************************************************************


//*******************************************************************************************
//*******************************************************************************************
//Запросы для отлаживания STM32 I2C в режиме Slave.
#define STM32_SLAVE_I2C		  I2C1
#define STM32_SLAVE_I2C_ADDR (0x05)

//static uint8_t txBuf[32] = {0};
//static uint8_t rxBuf[32] = {0};
//************************************************************
void Task_STM32_Slave_Write(void){

//	txBuf[0]++;
//	txBuf[1] = txBuf[0] + 1;
//	txBuf[2] = txBuf[1] + 1;
//	txBuf[3] = txBuf[2] + 1;
}
//************************************************************
void Task_STM32_Slave_Read(void){

//	LedPC13Toggel();
}
//************************************************************
void Task_Temperature_Read(void){

	TemperatureSens_ReadTemperature(&Sensor_1);
	TemperatureSens_ReadTemperature(&Sensor_2);
	TemperatureSens_ReadTemperature(&Sensor_3);

	I2cWire.pTxBuf[0] = (uint8_t) Sensor_1.TemperatureSign;
	I2cWire.pTxBuf[1] = (uint8_t)(Sensor_1.Temperature >> 8);
	I2cWire.pTxBuf[2] = (uint8_t) Sensor_1.Temperature;

	I2cWire.pTxBuf[3] = (uint8_t) Sensor_2.TemperatureSign;
	I2cWire.pTxBuf[4] = (uint8_t)(Sensor_2.Temperature >> 8);
	I2cWire.pTxBuf[5] = (uint8_t) Sensor_2.Temperature;

	I2cWire.pTxBuf[6] = (uint8_t) Sensor_3.TemperatureSign;
	I2cWire.pTxBuf[7] = (uint8_t)(Sensor_3.Temperature >> 8);
	I2cWire.pTxBuf[8] = (uint8_t) Sensor_3.Temperature;

	I2cWire.pTxBuf[9] = 0xC9;
}
//*******************************************************************************************
//*******************************************************************************************
int main(void){

	//-----------------------------
	//Drivers.
	Sys_Init();
	Gpio_Init();
	SysTick_Init();
	microDelay_Init();
	//Uart1Init(USART1_BRR);
	//Adc_Init();

	microDelay(100000);//Эта задержка нужна для стабилизации напряжения патания.
					   //Без задержки LCD-дисплей не работает.
	//***********************************************
	//Инициализация DS18B20.
	Sensor_1.SensorNumber = 1;
	Sensor_1.GPIO_PORT    = GPIOA;
	Sensor_1.GPIO_PIN     = 1;
	Sensor_1.Resolution   = DS18B20_Resolution_12_bit;
	TemperatureSens_GpioInit(&Sensor_1);
	TemperatureSens_SetResolution(&Sensor_1);
	TemperatureSens_StartConvertTemperature(&Sensor_1);

	Sensor_2.SensorNumber = 2;
	Sensor_2.GPIO_PORT    = GPIOA;
	Sensor_2.GPIO_PIN     = 2;
	Sensor_2.Resolution   = DS18B20_Resolution_12_bit;
	TemperatureSens_GpioInit(&Sensor_2);
	TemperatureSens_SetResolution(&Sensor_2);
	TemperatureSens_StartConvertTemperature(&Sensor_2);

	Sensor_3.SensorNumber = 3;
	Sensor_3.GPIO_PORT    = GPIOA;
	Sensor_3.GPIO_PIN     = 3;
	Sensor_3.Resolution   = DS18B20_Resolution_12_bit;
	TemperatureSens_GpioInit(&Sensor_3);
	TemperatureSens_SetResolution(&Sensor_3);
	TemperatureSens_StartConvertTemperature(&Sensor_3);
	//***********************************************
	//Инициализация I2C Slave для работы по прерываниям.
	I2cWire.i2c 		 = I2C1;
	I2cWire.i2cGpioRemap = I2C_GPIO_NOREMAP;
	I2cWire.slaveAddr	 = 0x05;
	I2cWire.pTxBuf  	 = slaveTxBuf;
	I2cWire.pRxBuf  	 = slaveRxBuf;
	I2C_IT_Slave_Init(&I2cWire);
	//***********************************************
	//Ининциализация SPI для работы с энкодером.





	//***********************************************
	//Инициализация	ШИМ
	//TIM3_InitForPWM();
	//***********************************************
	//Ини-я диспетчера.
	RTOS_Init();
	RTOS_SetTask(Task_Temperature_Read, 0, 1000);
	//RTOS_SetTask(Task_STM32_Slave_Write,0, 500);
	//RTOS_SetTask(Task_STM32_Slave_Read, 0, 500);
	//***********************************************
	__enable_irq();
	//**************************************************************
	while(1)
	{
		RTOS_DispatchLoop();
		//__WFI();//Sleep
	}
	//**************************************************************
}
//*******************************************************************************************
//*******************************************************************************************
//Прерывание каждую милисекунду.
void SysTick_Handler(void){

	RTOS_TimerServiceLoop();
	msDelay_Loop();
	Blink_Loop();
}
//*******************************************************************************************
//*******************************************************************************************















