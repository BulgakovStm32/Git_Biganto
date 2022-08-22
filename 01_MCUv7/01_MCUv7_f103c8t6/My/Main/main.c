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
Button_t  PwrButton;

DS18B20_t Sensor_1;
DS18B20_t Sensor_2;

I2C_IT_t  I2cWire;
//*******************************************************************************************
//*******************************************************************************************
uint32_t Led_Blink(uint32_t millis, uint32_t period, uint32_t switch_on_time){

	static uint32_t millisOld = 0;
	static uint32_t flag      = 0;
	//-------------------
	if((millis - millisOld) >= (flag ? (period - switch_on_time) : switch_on_time))
	{
		millisOld = millis;
		flag = !flag;
	}
	return flag;
}
//************************************************************

//*******************************************************************************************
//*******************************************************************************************
void Task_ReadTemperature(void){

	TemperatureSens_ReadTemperature(&Sensor_1);
	TemperatureSens_ReadTemperature(&Sensor_2);
}
//************************************************************
//ф-я вызыввается каждые 100мс.
void Task_PwrButtonPolling(void){

//	static uint32_t buttonLongPressCount = 0;
//	//-------------------
//	if(GPIO_GetPinState(MCU_PWR_BTN_GPIO, MCU_PWR_BTN_PIN) == PIN_LOW)
//	{
//		buttonLongPressCount++;
//		//Отключение через секунду
//		if(buttonLongPressCount >= 10)
//		{
//			buttonLongPressCount = 0;
//			MCU_EN_Low();
//			FAN_EN_Low();
//			LAMP_PWM_Low();
//			LIDAR_EN_Low();
//			GPS_EN_Low();
//		}
//	}
//	else buttonLongPressCount = 0;
}
//*******************************************************************************************
//*******************************************************************************************
void I2cRxParsing(void){

	MCU_RequestPack_t  *request  = (MCU_RequestPack_t *) I2C_IT_GetpRxBuf(&I2cWire);
	MCU_ResponsePack_t *response = (MCU_ResponsePack_t *)I2C_IT_GetpTxBuf(&I2cWire);
	//--------------------------
	LED_ACT_Toggel();//Отладка
	//Разбор пришедшего запроса
	switch(request->Pack.CmdCode)
	{
		//-------------------
		case(cmdGetCurrentPosition):
				//LED_ACT_Toggel();
				//spiData = ENCODER_GetVal();
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
		case(cmdGetEncoderPosition):
			response->Pack.Count   = 15; //Размер пакета в байтах (без поля CMD)
			response->Pack.CmdCode = cmdGetEncoderPosition;

			response->Pack.Payload[0] = 0xA0; //u8 SPI status
			response->Pack.Payload[1] = 0xA1; //u8 TMC reg address

			*(uint32_t*)&response->Pack.Payload[6]  = MICRO_DELAY_GetCount(); //u32 beginTS
			*(uint32_t*)&response->Pack.Payload[2]  = ENCODER_GetVal(); 	  //u32 data
			*(uint32_t*)&response->Pack.Payload[10] = MICRO_DELAY_GetCount(); //u32 endTS
		break;
		//-------------------
		case(cmdGetTemperature):
			response->Pack.Count   = 5; 			   // Размер пакета в байтах (без поля CMD)
			response->Pack.CmdCode = cmdGetTemperature;// Команда
			if(request->Pack.Payload[0] == 1) TEMP_SENSE_BuildPack(&Sensor_1, response->Pack.Payload);//Запрос для первого датчика температуры
			else 							  TEMP_SENSE_BuildPack(&Sensor_2, response->Pack.Payload);//Запрос для второго датчика температуры
		break;
		//-------------------
		default:

		break;
		//-------------------
	}
	I2C_IT_SetTxSize(&I2cWire, response->Pack.Count+1);//Кол-во байтов в ответе(с полем CMD).
}
//************************************************************
void I2cTxParsing(void){

}
//*******************************************************************************************
//*******************************************************************************************
//Работа с энкодером AMM3617.Энкодер выдает 17-тибитный код Грея.
#define ENCODER_NUM_STEP 		131072 					         //количество шагов энкодера на один оборот
#define ENCODER_DEGREE_QUANT  	(float)(360.0 / ENCODER_NUM_STEP)//количество градусов в одном наге энкодера.

//#define ENCODER_TIMEOUT 		(30 / 10)			      //деление на 10 так как uSecTick = 10 мкСек.
#define _RPM					60
#define QUANT_FOR_100mS     	((_RPM * (1000/100) * 1000) / 360.0)
#define QUANT_FOR_10mS      	((_RPM * (1000/10)  * 1000) / 360.0)
#define QUANT_FOR_64mS      	((_RPM * (1000/64)  * 1000) / 360.0)
//********************************************************
volatile uint32_t sysTick      		= 0;
volatile uint32_t EncoderTicks 		= 0;

volatile float Angle           		= 0.0;
volatile float OldAngle        		= 0.0;
volatile float DeltaAngle      		= 0.0;
volatile float RPM             		= 0.0;

uint8_t txBuf[64] = {0,};
//*******************************************************************************************
void BinToDecWithoutDot(uint32_t var, uint8_t* buf){

	*(buf+0) = (uint8_t)(var / 100000) + '0';
	var %= 100000;

	*(buf+1) = (uint8_t)(var / 10000) + '0';
	var %= 10000;

	*(buf+2) = (uint8_t)(var / 1000) + '0';
	var %= 1000;

	//*(buf+3) = ',';

	*(buf+3) = (uint8_t)(var / 100) + '0';
	var %= 100;

	*(buf+4) = (uint8_t)(var / 10) + '0';
	*(buf+5) = (uint8_t)(var % 10) + '0';
}
//************************************************************
void BinToDecWithDot(uint32_t var, uint8_t* buf){

	*(buf+0) = (uint8_t)(var / 100000) + '0';
	var %= 100000;

	*(buf+1) = (uint8_t)(var / 10000) + '0';
	var %= 10000;

	*(buf+2) = (uint8_t)(var / 1000) + '0';
	var %= 1000;

	*(buf+3) = ',';

	*(buf+4) = (uint8_t)(var / 100) + '0';
	var %= 100;

	*(buf+5) = (uint8_t)(var / 10) + '0';
	*(buf+6) = (uint8_t)(var % 10) + '0';
}
//************************************************************
void BuildAndSendTextBuf(uint32_t timeStamp, uint32_t encodTicks, uint32_t angle, uint32_t speed){

	BinToDecWithDot(timeStamp, txBuf);
	txBuf[7] = '\t';

	BinToDecWithoutDot(encodTicks, txBuf+8);
	txBuf[14] = '\t';

	BinToDecWithDot(angle, txBuf+15);
	txBuf[22] = '\t';

	BinToDecWithDot(speed, txBuf+23);
	txBuf[30] = '\r';

	DMA1Ch4StartTx(txBuf, 31);
}
//************************************************************
void Task_Motor(void){

	static uint32_t flag = 0;

	if(!flag)
	{
		MOTOR_SpinStart(MOTOR_DECEL);
		flag = 1;
	}
	else
	{
		MOTOR_SpinStart(MOTOR_ACCEL);
		flag = 0;
	}
}
//*******************************************************************************************
//*******************************************************************************************
int main(void){

	//***********************************************
	//Инициализация периферии STM32.
	STM32_Clock_Init();
	GPIO_Init();
	MICRO_DELAY_Init();
	ADC_Init();
	//***********************************************
	//Инициализация кнопки.
	PwrButton.GPIO_PORT = MCU_PWR_BUTTON_GPIO;
	PwrButton.GPIO_PIN  = MCU_PWR_BUTTON_PIN;
	PwrButton.State		= BUTTON_RELEASE;
	BUTTON_Init(&PwrButton);
	//***********************************************
	MICRO_DELAY(100000);	//100mS - Эта задержка нужна для стабилизации напряжения патания.
	//Включение питания платы.
//	while(BUTTON_GetState(&PwrButton) != BUTTON_RELEASE){};//Пока кнопку не отпустя плата не включится.
//	{
//		if(BUTTON_GetState(&PwrButton) == BUTTON_LONG_PRESS) LED_ACT_High();
//	}

	MCU_EN_High();
	FAN_EN_High();
	GPS_EN_High();
	LIDAR_EN_High();
//	LAMP_PWM_High();

	MICRO_DELAY(500000);//Эта задержка нужна для стабилизации напряжения патания.
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
	//Ининциализация оптических дадчитков наличия крышки объетива и наличия АКБ.
	OPT_SENS_Init();
	//***********************************************
	USART_Init(USART1, 57600);
	//***********************************************
	//Инициализация I2C Slave для работы по прерываниям.
	I2cWire.i2c 		  = I2C2;
	I2cWire.i2cMode		  = I2C_MODE_SLAVE;
	I2cWire.i2cGpioRemap  = I2C_GPIO_NOREMAP;;
	I2cWire.slaveAddr	  = 0x05;
	I2cWire.txBufSize     = 16;
	I2cWire.i2cRxCallback = I2cRxParsing;
	I2cWire.i2cTxCallback = I2cTxParsing;
	I2C_IT_Init(&I2cWire);
	//***********************************************
	//Инициализация	драйвера мотора.
//	MOTOR_Init();
//	MOTOR_CalcAccelDecel(0, 300, 1500);
//	MOTOR_SpinStart(MOTOR_ACCEL);

	//MOTOR_SetMaxVelocity(400); //Задание скорости в RPM
	//***********************************************
	//Ини-я диспетчера.
	RTOS_Init();
	RTOS_SetTask(Task_ReadTemperature,  50, 1000);//запуск задачи через 50мс и спериодом повторения 1000мс
	RTOS_SetTask(Task_PwrButtonPolling, 50, 100); //запуск задачи через 50мс и спериодом повторения 100мс

	//RTOS_SetTask(Task_Motor, 5000, 5000);
	//***********************************************
	SysTick_Init();
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
void SysTick_IT_Handler(void){

	RTOS_TimerServiceLoop();
	msDelay_Loop();
	Blink_Loop();
	GPIO_CheckLoop();
	//--------------------------
	OPT_SENS_CheckLoop();		  //Опрос состояния сенсоров наличия крышки объектива и наличия АКБ.
	BUTTON_CheckLoop(&PwrButton); //Опрос кнопки.
	//------------------------------------------
	//Работа с энкодером
	static uint32_t mSecCount = 0;
	//Формирование таймстемпов
	sysTick++;
	if(sysTick > 999999) sysTick = 0;
	//--------------------------
	if(++mSecCount >= 100)
	{
		mSecCount = 0;

		//__disable_irq();
		//LED_ACT_High();
		//Чтение заначения энкодера. ~9,5 мкС
		EncoderTicks = ENCODER_GetVal();
		//LED_ACT_Low();
		//Расчет значений для будущего расчета скрости.
		Angle = ENCODER_DEGREE_QUANT * EncoderTicks;  //расчет угла поворота вала энкодера.
		if(OldAngle > Angle) OldAngle -= 360.0;		  //Это нужно для корректного расчета скорости при переходе от 359 к 0 градусов.
		DeltaAngle = Angle - OldAngle;         		  //приращение угла

		//Расчет скорости вращения.
		RPM = DeltaAngle * QUANT_FOR_100mS;
		OldAngle = Angle;
		//Передаем данные
		BuildAndSendTextBuf(sysTick,
							EncoderTicks,
							(uint32_t)(Angle*1000),
							(uint32_t)(RPM*12)); //умножаем на 120 т.к. передаточное число редуктора 120.
		//__enable_irq();
	}
	//------------------------------------------
//	MOTOR_AccelDecelLoop();	//Отладка работы мотора.
	//------------------------------------------
}
//*******************************************************************************************
//*******************************************************************************************











