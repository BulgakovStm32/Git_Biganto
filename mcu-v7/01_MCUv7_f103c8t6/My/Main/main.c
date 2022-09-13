/*
 * 	main.c
 *
 *  Created on: 22.07.2022
 *  Autho     : Беляев А.А.
 *
 *
 *	Описание  : MCUv7
 *
 *	Последнее редактирование: 24.08.2022
 *
 */
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************

Button_t  PwrButton;


void Task_UartSend(void);
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
void Task_TEMPERATURE_Read(void){

	//Во время вращения запрещено чтение температуры.
	if(MOTOR_GetSpeedRampState() != STOP) return;
	TEMPERATURE_SENSE1_ReadTemperature();
	TEMPERATURE_SENSE2_ReadTemperature();
}
//************************************************************
//ф-я вызыввается каждые 100мс.
void Task_POWER_Check(void){

	static uint32_t count = 0;
	uint32_t supplyVoltage;
	//-------------------
	//Проверка напряжения АКБ.
	supplyVoltage = POWER_SupplyVoltage();

	//Напряженеи НИЖЕ миннимального (10,8В) напряжения АКБ.
	if(supplyVoltage <= BATTERY_VOLTAGE_MIN)POWER_Flags()->f_BatteryLow = FLAG_SET;
	//Напряжение АКБ НИЖЕ 12В
	else if(supplyVoltage <= BATTERY_VOLTAGE_WARNING) POWER_Flags()->f_BatteryWarning = FLAG_SET;
	//Напряжение АКБ выше 12в
	else POWER_Flags()->f_BatteryWarning = FLAG_CLEAR;
	//-------------------
	//Отключение питания по нажатию кнопки или низком напряжении АКБ.
	if(POWER_PwrButton() == PRESS || POWER_Flags()->f_BatteryLow)
	{
		POWER_Flags()->f_PowerOff = FLAG_SET;
		Task_UartSend();//Отладка
		//Отключение питания MCU.
		MOTOR_Disable();
		FAN_EN_Low();
		LAMP_PWM_Low();
		LIDAR_EN_Low();
		GPS_EN_Low();

		count = RTOS_GetTickCount();
		while((RTOS_GetTickCount() - count) <= 3000)
		{
			if(Blink(INTERVAL_50_mS)) PWR_BTN_LED_Low();
			else					   PWR_BTN_LED_High();
		}
		__disable_irq();
		PWR_BTN_LED_Low();
		MCU_POWER_OFF();
		DELAY_milliS(1000);
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Работа с энкодером AMM3617.Энкодер выдает 17-тибитный код Грея.
#define _RPM					60
#define QUANT_FOR_100mS     	((_RPM * (1000/100) * 1000) / 360.0)
#define QUANT_FOR_10mS      	((_RPM * (1000/10)  * 1000) / 360.0)
#define QUANT_FOR_64mS      	((_RPM * (1000/64)  * 1000) / 360.0)
//********************************************************
volatile uint32_t sysTick      	= 0;
volatile int32_t  encoderTicks 	= 0;
volatile static uint32_t encoderOffset	= 0;

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

//	DMA1Ch4StartTx(txBuf, 31);
}
//************************************************************
void Task_UartSend(void){

	//--------------------------------
	Txt_Chr('\f');
	Txt_Print("******************\n");
	Txt_Print("_MCUv7_(+BT)\n");

	Txt_Print("mS_count: ");
	Txt_BinToDec(RTOS_GetTickCount(), 9);//часы
	Txt_Chr('\n');

	//Вывод Напряжения питания
	Txt_Print("SupplyVoltage: ");
	Txt_BinToDec(POWER_SupplyVoltage(), 6);
	Txt_Chr('\n');

	//Вывод данных энкодера.
	Txt_Print("ENCOD_OFFSET  : ");
	Txt_BinToDec(encoderOffset, 6);
	Txt_Chr('\n');

	Txt_Print("ENCOD_CODE  : ");
	Txt_BinToDec(encoderTicks, 6);
	Txt_Chr('\n');

	Txt_Print("ENCOD_ANGLE: ");
	Txt_BinToDec((uint32_t)(Angle * 1000), 6);
	Txt_Chr('\n');

	Txt_Print("ENCOD_RPM   : ");
	Txt_BinToDec((uint32_t)(RPM), 6);
	Txt_Chr('\n');

	//Вывод температуры.
	Txt_Print("TSens1= ");
	if(TEMPERATURE_SENSE_GetSens(1)->TemperatureSign == DS18B20_SIGN_POSITIVE)
	{
		 Txt_Chr('+');
	}
	else Txt_Chr('-');
	Txt_BinToDec(TEMPERATURE_SENSE_GetSens(1)->Temperature/10, 2);
	Txt_Chr('.');
	Txt_BinToDec(TEMPERATURE_SENSE_GetSens(1)->Temperature%10, 1);
	Txt_Print(" C");
	Txt_Chr('\n');

	Txt_Print("TSens2= ");
	if(TEMPERATURE_SENSE_GetSens(2)->TemperatureSign == DS18B20_SIGN_POSITIVE)
	{
		 Txt_Chr('+');
	}
	else Txt_Chr('-');
	Txt_BinToDec(TEMPERATURE_SENSE_GetSens(2)->Temperature/10, 2);
	Txt_Chr('.');
	Txt_BinToDec(TEMPERATURE_SENSE_GetSens(2)->Temperature%10, 1);
	Txt_Print(" C");
	Txt_Chr('\n');

	//Количество нажатий на кнопку.
//	Txt_Print("ButtonPress=");
//	Txt_BinToDec(ButtonPressCount, 4);

		 if(POWER_Flags()->f_BatteryWarning) Txt_Print("BATTERY < 12v!!!\n");
	else if(POWER_Flags()->f_BatteryLow) 	 Txt_Print("BATTERY LOW!!!\n");
	else if(POWER_Flags()->f_PowerOff)   	 Txt_Print("POWER OFF!!!\n");

	Txt_Chr('\n');
	Txt_Print("******************\n");
	//--------------------------------
	DMA1Ch4StartTx(Txt_Buf()->buf, Txt_Buf()->bufIndex);
	Txt_Buf()->bufIndex = 0;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void Task_Motor(void){

	uint32_t RPM            = 38;    	//Скорость оборотов в минуту
	uint32_t accelTime_mSec = 500;		//время ускорения в милисекундах
	uint32_t angle		    = 120*1; 	//Угол на который нужно переместить вал камеры в градусах.
	uint32_t reducerRate    = 6;		//передаточное число редуктора = 6.
	//-----------------------------------------------------
	//Вариант расчета значений когда ускорение задается как время ускорения в секундах
	//uint32_t accel_RPMM  = 4000; //Ускорение оборотов в минуту за минуту
	uint32_t accel_RPMM = (uint32_t)(RPM / ((float)accelTime_mSec / 1000 / 60.0));
	//-----------------------------------------------------
	//Пример расчета:
	//Макс.скорость - 10 RPM, это = 10*2*Pi = 62.8319 рад/мин, или 1,0472 рад/сек.   1,0472/ 0,01 = 104,72 ~ 105 (in 0.01*rad/sec).
	//Ускорение  - 20 RPM^2, это = 20*2*Pi = 125.6637 рад/мин^2, или 0,035 рад/сек^2. 0,035 / 0,01 = 3,49   ~ 3	 (in 0.01*rad/sec^2).
	//Замедление - 20 RPM^2, расчет тотже что и при ускорении.
	//Шаги - мне нужно повернуться на 360 градусов. Это 6400 шагов при microstep = 1/32

	// param step   Number of steps to move (pos - CW, neg - CCW).
	// param accel  Accelration to use, in 0.01*rad/sec^2.
	// param decel  Decelration to use, in 0.01*rad/sec^2.
	// param speed  Max speed, in 0.01*rad/sec.

	uint32_t microSteps = (uint32_t)MOTOR_FULL_STEPS_PER_TURN * MOTOR_GetMicrostep();
	uint32_t steps			   = (uint32_t)(angle      * reducerRate * microSteps / (float)360.0);//кол-во шагов необходимое длч перемещение на угол angle
	uint32_t vel_rad_per_sec   = (uint32_t)(RPM        * reducerRate * _2PI / 60   * 100);		  //скорость рад в сек
	uint32_t accel_rad_per_sec = (uint32_t)(accel_RPMM * reducerRate * _2PI / 3600 * 100);		  //Ускорение рад в секунду за секунд

	MOTOR_SpeedCntrMove(-1*steps, 		   //param step   Number of steps to move
						accel_rad_per_sec, //param accel  Accelration to use, in 0.01*rad/sec^2.
						accel_rad_per_sec, //param decel  Decelration to use, in 0.01*rad/sec^2.
						vel_rad_per_sec);  //param speed  Max speed, in 0.01*rad/sec.
	//-----------------------------------------------------
//	MOTOR_Move(360*6*2, 5, 35*6, 1500);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
int main(void){

	//***********************************************
	//Инициализация периферии STM32.
 	STM32_Clock_Init();
	GPIO_Init();
	DELAY_Init();
	//***********************************************
	//Инициализация АЦП для измерения напряжения АКБ.
	//Инициализация кнопки питания.
	POWER_Init();
	//***********************************************
	while(POWER_PwrButton() != RELEASE){};//Ждем отпускания копки.
	//Включение питания платы.
	MCU_POWER_ON();
	//Смотрим что там с напряжением АКБ.
//	if(POWER_SupplyVoltage() < BATTERY_VOLTAGE_MIN)//Напряженеи НИЖЕ миннимального (10,8В) напряжения АКБ.
//	{
//		SysTick_Init();//Это нужно для мигающей индикации
//		__enable_irq();
//		while(1)//(POWER_SupplyVoltage() <= 10800)
//		{
//			//Мигающая индикация - Низский заряд АКБ.
//			if(Blink(INTERVAL_100_mS)) PWR_BTN_LED_High();
//			else 					   PWR_BTN_LED_Low();
//			//Отключение по нажатию на кнопку.
//			if(!(MCU_PWR_BTN_GPIO->IDR & (1<<MCU_PWR_BTN_PIN))) Task_POWER_Check();
//		}
//	}
	//Включение питания периферии.
	PWR_BTN_LED_High();
	FAN_EN_High();
	GPS_EN_High();
	LIDAR_EN_High();
//	LAMP_PWM_High();
	DELAY_milliS(500);//500mS - Задержка для стабилизации напряжения патания.
	//***********************************************
	TEMPERATURE_SENSE_Init(); //Инициализация датчиков температуры.
	OPT_SENS_Init();		  //Инициализация оптических дадчитков наличия крышки объетива и наличия АКБ.
	ENCODER_Init();			  //Инициализация энкодера.
	MOTOR_Init();			  //Инициализация драйвера мотора.
	PROTOCOL_I2C_Init();	  //Инициализация протокола обмена.

	//	MOTOR_Move(360*6*2, 5, 35*6, 500);//Для отладки.
	USART_Init(USART1, 9600);//57600);	//Для отладки.
	//***********************************************
	//Ини-я диспетчера.
	RTOS_Init();
	RTOS_SetTask(Task_TEMPERATURE_Read, 0, 1000);//запуск задачи через 0мс и с периодом повторения 1000мс
	RTOS_SetTask(Task_POWER_Check,      0, 100); //запуск задачи через 0мс и с периодом повторения 100мс
	RTOS_SetTask(Task_Motor,            2000, 5*1000);
	RTOS_SetTask(Task_UartSend,         1000, 500);
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

	msDelay_Loop();
	Blink_Loop();
	GPIO_CheckLoop();
	RTOS_TimerServiceLoop();
	if(POWER_Flags()->f_PowerOff)return;//Если нажали кнопку питания значит выключаемся и ничего не опрашиваем..
	//--------------------------
	OPT_SENS_CheckLoop(); //Опрос состояния сенсоров наличия крышки объектива и наличия АКБ.
	//MOTOR_TimerITHandler2();//Отладка!!!
	//**************************************************************
	//Работа с энкодером. Для отладки данные предаются по USART на кудахтер.
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
		encoderTicks  = ENCODER_GetCode() - encoderOffset;
		if(encoderTicks < 0) encoderTicks = -encoderTicks;
		//LED_ACT_Low();
		//Расчет значений для расчета скрости.
		Angle = ENCODER_DEGREE_QUANT * encoderTicks;  //расчет угла поворота вала энкодера.
		if(OldAngle > Angle) OldAngle -= 360.0;		  //Это нужно для корректного расчета скорости при переходе от 359 к 0 градусов.
		DeltaAngle = Angle - OldAngle;         		  //приращение угла

		//Расчет скорости вращения.
		RPM = DeltaAngle * QUANT_FOR_100mS;
		OldAngle = Angle;
		//Передаем данные по USART на кудахтер.
		BuildAndSendTextBuf(sysTick,
							encoderTicks,
							(uint32_t)(Angle * 1000),
							(uint32_t)(RPM));
		//__enable_irq();
	}
	//**************************************************************
}
//*******************************************************************************************
//*******************************************************************************************











