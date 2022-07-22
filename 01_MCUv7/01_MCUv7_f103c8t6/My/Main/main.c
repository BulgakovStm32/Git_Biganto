/*
 * 	main.c
 *
 *  Created on: 22.07.2022
 *  Autho     : Беляев А.А.
 *
 *	Описание  : MCUv7

 */
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
typedef struct{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
}Time_t;

static Time_t Time;
//---------------------------
DS18B20_t Sensor_1;
DS18B20_t Sensor_2;
DS18B20_t Sensor_3;

//*******************************************************************************************
//*******************************************************************************************
void IncrementOnEachPass(uint32_t *var, uint16_t event){

		   uint16_t riseReg  = 0;
	static uint16_t oldState = 0;
	//-------------------
	riseReg  = (oldState ^ event) & event;
	oldState = event;
	if(riseReg) (*var)++;
}
//************************************************************
void Time_Calculation(uint32_t count){

	Time.hour =  count / 3600;
	Time.min  = (count % 3600) / 60;
	Time.sec  =  count % 60;
}
//************************************************************
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
//*******************************************************************************************
//*******************************************************************************************
//Запросы для отлаживания STM32 I2C в режиме Slave.
#define STM32_SLAVE_I2C		  I2C1
#define STM32_SLAVE_I2C_ADDR (0x05)

static uint8_t txBuf[32] = {0};
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

	txBuf[0] = (uint8_t) Sensor_1.TEMPERATURE_SIGN;
	txBuf[1] = (uint8_t)(Sensor_1.TEMPERATURE >> 8);
	txBuf[2] = (uint8_t) Sensor_1.TEMPERATURE;

	txBuf[3] = (uint8_t) Sensor_2.TEMPERATURE_SIGN;
	txBuf[4] = (uint8_t)(Sensor_2.TEMPERATURE >> 8);
	txBuf[5] = (uint8_t) Sensor_2.TEMPERATURE;

	txBuf[6] = (uint8_t) Sensor_3.TEMPERATURE_SIGN;
	txBuf[7] = (uint8_t)(Sensor_3.TEMPERATURE >> 8);
	txBuf[8] = (uint8_t) Sensor_3.TEMPERATURE;

	txBuf[9] = 0xC9;
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
	//Ини-я DS18B20
	Sensor_1.SENSOR_NUMBER = 1;
	Sensor_1.GPIO_PORT     = GPIOA;
	Sensor_1.GPIO_PIN      = 1;
	Sensor_1.RESOLUTION    = DS18B20_Resolution_12_bit;
	TemperatureSens_GpioInit(&Sensor_1);
	TemperatureSens_SetResolution(&Sensor_1);
	TemperatureSens_StartConvertTemperature(&Sensor_1);

	Sensor_2.SENSOR_NUMBER = 2;
	Sensor_2.GPIO_PORT     = GPIOA;
	Sensor_2.GPIO_PIN      = 2;
	Sensor_2.RESOLUTION    = DS18B20_Resolution_12_bit;
	TemperatureSens_GpioInit(&Sensor_2);
	TemperatureSens_SetResolution(&Sensor_2);
	TemperatureSens_StartConvertTemperature(&Sensor_2);

	Sensor_3.SENSOR_NUMBER = 3;
	Sensor_3.GPIO_PORT     = GPIOA;
	Sensor_3.GPIO_PIN      = 3;
	Sensor_3.RESOLUTION    = DS18B20_Resolution_12_bit;
	TemperatureSens_GpioInit(&Sensor_3);
	TemperatureSens_SetResolution(&Sensor_3);
	TemperatureSens_StartConvertTemperature(&Sensor_3);
	//***********************************************
	//Инициализация	ШИМ
	//TIM3_InitForPWM();
	//***********************************************
	//Отладка I2C по прерываниям.
//	static uint8_t i2cBuf[3] = {1, 2, 3};
//	I2C_IT_Init(I2C1, 0);
//	I2C_IT_StartTx(I2C1, SSD1306_I2C_ADDR, 0x55, i2cBuf, 3);

	//Отладка I2C Slave
	//I2C_Slave_Init(I2C1, 0x05, 0);
	I2C_IT_Slave_Init(STM32_SLAVE_I2C, STM32_SLAVE_I2C_ADDR, 0);
	//I2C_IT_Slave_StartRx(rxBuf, 3);
	I2C_IT_Slave_StartTx(txBuf, 32);
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

	static uint32_t secCounter = 0;
	IncrementOnEachPass(&secCounter, Blink(INTERVAL_500_mS));//Инкримент счетчика секунд.
	Time_Calculation(secCounter);						     //Преобразование времени

	RTOS_TimerServiceLoop();
	msDelay_Loop();
	Blink_Loop();
}
//*******************************************************************************************
//******************************************************************************************
