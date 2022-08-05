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
//DS18B20_t Sensor_3;

I2C_IT_t	I2cWire;
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
	I2cWire.pTxBuf[0] = (uint8_t) Sensor_1.TemperatureSign;
	I2cWire.pTxBuf[1] = (uint8_t)(Sensor_1.Temperature >> 8);
	I2cWire.pTxBuf[2] = (uint8_t) Sensor_1.Temperature;

	TemperatureSens_ReadTemperature(&Sensor_2);
	I2cWire.pTxBuf[3] = (uint8_t) Sensor_2.TemperatureSign;
	I2cWire.pTxBuf[4] = (uint8_t)(Sensor_2.Temperature >> 8);
	I2cWire.pTxBuf[5] = (uint8_t) Sensor_2.Temperature;

//	TemperatureSens_ReadTemperature(&Sensor_3);
//	I2cWire.pTxBuf[6] = (uint8_t) Sensor_3.TemperatureSign;
//	I2cWire.pTxBuf[7] = (uint8_t)(Sensor_3.Temperature >> 8);
//	I2cWire.pTxBuf[8] = (uint8_t) Sensor_3.Temperature;

//	I2cWire.pTxBuf[9] = 0xC9;
//
	I2cWire.txBufSize = 9;

//	LED_ACT_Toggel();
}
//************************************************************
//ф-я вызыввается каждые 100мс.
void Task_PwrButtonPolling(void){

	static uint32_t buttonLongPressCount = 0;
	//-------------------
	if(GPIO_GetPinState(MCU_PWR_BTN_GPIO, MCU_PWR_BTN_PIN) == PIN_LOW)
	{
		buttonLongPressCount++;
		//Отключение через секунду
		if(buttonLongPressCount >= 10)
		{
			buttonLongPressCount = 0;
			MCU_EN_Low();
			FAN_EN_Low();
			LAMP_PWM_Low();
			LIDAR_EN_Low();
			GPS_EN_Low();
		}
	}
	else buttonLongPressCount = 0;
}
//*******************************************************************************************
//*******************************************************************************************
void I2cRxParsing(void){

	static uint32_t spiData = 0;

	switch(I2cWire.pRxBuf[0])
	{
		//-------------------
		case(cmdGetCurrentPosition):
				LED_ACT_Toggel();
				spiData = ENCODER_GetVal();
		break;
		//-------------------
		case(cmdGetCurrentAcceleration):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdGetCurrentVelocity):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdSetTargetPosition):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdSetMaxAcceleration):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdSetMaxVelocity):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdSetMicrostep ):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdResetTMC):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdResetPosition):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdEmergencyStop):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdArduinoMicroTS):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdArduinoMeasurePulseCalibration):
				//LED_ACT_Toggel();
		break;
		//-------------------
		case(cmdArduinoGetPulseCalibration):
				//LED_ACT_Toggel();
		break;
		//-------------------
		default:

		break;
		//-------------------
	}
}
//************************************************************
void I2cTxParsing(void){

//	LED_ACT_Toggel();

}
//*******************************************************************************************
//*******************************************************************************************
int main(void){

	//-----------------------------
	//Drivers.
	Sys_Init();
	GPIO_Init();
	SysTick_Init();
	microDelay_Init();
	//Uart1Init(USART1_BRR);
	//Adc_Init();

	microDelay(100000);//Эта задержка нужна для стабилизации напряжения патания.
					   //Без задержки LCD-дисплей не работает.
	//***********************************************
	//Включение питания платы.
	while(GPIO_GetPinState(MCU_PWR_BTN_GPIO, MCU_PWR_BTN_PIN) == PIN_LOW); //Пока кнопку не отпустя плата не включится.

	MCU_EN_High();
	FAN_EN_High();
	LAMP_PWM_High();
	LIDAR_EN_High();
	GPS_EN_High();
	//***********************************************
	//Инициализация DS18B20.
	Sensor_1.SensorNumber = 1;
	Sensor_1.GPIO_PORT    = GPIOB;
	Sensor_1.GPIO_PIN     = 14;
	Sensor_1.Resolution   = DS18B20_Resolution_12_bit;
	TemperatureSens_Init(&Sensor_1);
	TemperatureSens_StartConvertTemperature(&Sensor_1);

	Sensor_2.SensorNumber = 2;
	Sensor_2.GPIO_PORT    = GPIOB;
	Sensor_2.GPIO_PIN     = 15;
	Sensor_2.Resolution   = DS18B20_Resolution_12_bit;
	TemperatureSens_Init(&Sensor_2);
	TemperatureSens_StartConvertTemperature(&Sensor_2);
	//***********************************************
	//Ининциализация энкодера.
	ENCODER_Init();
	//***********************************************
	//Инициализация I2C Slave для работы по прерываниям.
	I2cWire.i2c 		  = I2C2;
	I2cWire.i2cMode		  = I2C_MODE_SLAVE;
	I2cWire.i2cGpioRemap  = I2C_GPIO_NOREMAP;
	//I2cWire.i2cDmaState  = I2C_DMA_READY;
	I2cWire.slaveAddr	  = 0x05;
	I2cWire.txBufSize     = 16;
	I2cWire.i2cRxCallback = I2cRxParsing;
	I2cWire.i2cTxCallback = I2cTxParsing;
	I2C_IT_Init(&I2cWire);
	//I2C_DMA_Init(&I2cWire);
	//***********************************************
	//Инициализация	драйвера мотора.
	//MOTOR_Init();

	TIM2_Init();
	//***********************************************
	//Ини-я диспетчера.
	RTOS_Init();
	RTOS_SetTask(Task_Temperature_Read, 0, 1000);
	RTOS_SetTask(Task_PwrButtonPolling, 0, 100);

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
	GPIO_CheckLoop();
}
//*******************************************************************************************
//*******************************************************************************************











