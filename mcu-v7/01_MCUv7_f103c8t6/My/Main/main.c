/*
 * 	main.c
 *
 *  Created on: 22.07.2022
 *  Autho     : Беляев А.А.
 *
 *
 *	Описание  : MCUv7
 *
 *	Последнее редактирование: 22.09.2022
 *
 */
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
Button_t PwrButton;

void DBG_SendDebugInfo(void);

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
static void SettingInterruptPriorities(void){

	//Приоритеты прерываний.
	//Таймер управления мотором - самый высокий приоритет = 1.
	NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	//Прерывания I2C для работы протакола - приоритет = 2
	NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	//Системный таймер, прерывание каждую 1мс - самый низкий приоритет = 3
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	//Разрешаем прерывания.
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(I2C2_ER_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void Task_TEMPERATURE_Read(void){

	static uint32_t flag = 0;
	//-------------------
	//Во время вращения запрещено чтение температуры.
	//if(MOTOR_GetSpeedRampState() != STOP) return;

	//Чтение темперартуры с датчика 2 смещенео на 500мс
	//относительно чтение темперартуры датчика 1.
	flag ^= 1;
	if(flag) TEMPERATURE_SENSE1_ReadTemperature();
	else	 TEMPERATURE_SENSE2_ReadTemperature();
}
//************************************************************
//ф-я вызыввается каждые 100мс.
void Task_POWER_Check(void){

	uint32_t count;
	BlinkIntervalEnum_t blinkInterval;
	//-------------------
	//Проверка напряжения АКБ.
	POWER_CheckSupplyVoltage();
	//Отключение питания при:
	if(POWER_PwrButton() == PRESS  ||      //нажатие на кнопку патания,
	   POWER_Flags()->f_BatteryLow ||      //низкое напряжение АКБ,
	   PROTOCOL_I2C_Flags()->f_CmdTurnOff) //пришла команда на отключение.
	{
		PROTOCOL_I2C_SystemCtrlReg()->Flags.f_PwrOff = FLAG_SET;
		POWER_Flags()->f_PowerOff = FLAG_SET;
		DBG_SendDebugInfo();//Отладка
		//-------------------
		//Отключение питания периферии.
		MOTOR_Disable();
		GPS_PowerControl(GPS_POWER_OFF);
		LAMP_PWM_Low();
		LIDAR_EN_Low();
		//Опрос питания БигБорд.
		count = RTOS_GetTickCount();
		while(POWER_GetBigBoardPwr())
		{
			//Мигающая индикация отключения питания.
			POWER_PwrButtonLed(Blink(INTERVAL_500_mS));
			//Длительное нажатие на кнопку (5 сек).
			if(POWER_PwrButton() == PRESS)
			{
				if((RTOS_GetTickCount() - count) >= 5000) goto POWER_OFF;
			}
			else count = RTOS_GetTickCount();
			//Низкое напряжени АКБ.
			if(POWER_Flags()->f_BatteryLow) break;
		}
		//Определение периода мигающей индикации.
		if(POWER_Flags()->f_BatteryLow) blinkInterval = INTERVAL_50_mS;
		else							blinkInterval = INTERVAL_100_mS;
		//Мигающая индикация отключения в течении 2 сек.
		while((RTOS_GetTickCount() - count) <= 2000)
		{
			POWER_PwrButtonLed(Blink(blinkInterval));
		}
		//-------------------
		POWER_OFF:
		PWR_BTN_LED_Low();
		FAN_EN_Low();
		//BB_PWR_BTN_Low();
		__disable_irq();
		MCU_POWER_OFF();
		while(1);
		//-------------------
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void DBG_SendDebugInfo(void){

	static uint32_t flag = 0;

	if(!flag) POWER_SetPWMwidthForLamp(50);
	else      POWER_SetPWMwidthForLamp(0);
	flag ^= 1;

	//--------------------------------
	Txt_Chr('\f');
	Txt_Print("******************\n");
	Txt_Print("_MCUv7_(+BT)\n");

	//Количество зависаний I2C
	Txt_Print("I2C_ReInit: ");
	Txt_BinToDec(PROTOCOL_I2C_GetResetCount(), 6);
	Txt_Chr('\n');

	//Счетчик миллисекунд MCU
	Txt_Print("mS_count: ");
	Txt_BinToDec(RTOS_GetTickCount(), 9);//часы
	Txt_Chr('\n');

	//Вывод Напряжения питания
	Txt_Print("SupplyVoltage = ");
	Txt_BinToDec(POWER_GetSupplyVoltage(), 5);
	Txt_Print(" mV\n");

	//Кол-во шагов сделанные мотором
	Txt_Print("MotorPosition : ");
	Txt_BinToDec(MOTOR_GetMotorPosition(), 10);
	Txt_Chr('\n');

	//код энкодера
	Txt_Print("EncoderCode : ");
	Txt_BinToDec(ENCODER_GetCode(), 6);
	Txt_Chr('\n');

	//угол энкодера
	Txt_Print("EncoderAngle : ");
	Txt_BinToDec(ENCODER_GetAngle(), 7);
	Txt_Chr('\n');

	Txt_Print("EncoderQuant : ");
	Txt_BinToDec(ENCODER_GetAngleQuant(), 7);
	Txt_Chr('\n');


//	//Вывод данных энкодера.
//	Txt_Print("ENCOD_OFFSET  : ");
//	Txt_BinToDec(ENCODER_DBG_Data()->offset, 6);
//	Txt_Chr('\n');
//
//	Txt_Print("ENCOD_CODE  : ");
//	Txt_BinToDec(ENCODER_DBG_Data()->code, 6);
//	Txt_Chr('\n');
//
//	Txt_Print("ENCOD_ANGLE: ");
//	Txt_BinToDec((uint32_t)(ENCODER_DBG_Data()->angle * 1000), 6);
//	Txt_Chr('\n');
//
//	Txt_Print("ENCOD_RPM   : ");
//	Txt_BinToDec((uint32_t)(ENCODER_DBG_Data()->RPM), 6);
//	Txt_Chr('\n');

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

	//Состояние питания MCU.
		 if(POWER_Flags()->f_BatteryWarning) Txt_Print("BATTERY < 12v!!!\n");
	else if(POWER_Flags()->f_BatteryLow) 	 Txt_Print("BATTERY LOW!!!\n");
	else if(POWER_Flags()->f_PowerOff)   	 Txt_Print("POWER OFF!!!\n");
	else									 Txt_Print("BATTERY OK.\n");

	Txt_Chr('\n');
	Txt_Print("******************\n");
	//--------------------------------
	//USART1_TX -> DMA1_Channel4
	//USART2_TX -> DMA1_Channel7
	DMAxChxStartTx(DMA1_Channel7, Txt_Buf()->buf, Txt_Buf()->bufIndex);
	Txt_Buf()->bufIndex = 0;
}
//************************************************************
void DBG_UsartCmdCheck(void){

	static uint8_t rxBuff[RING_BUFF_SIZE] = {0};
	uint32_t charIndex = 0;
	uint32_t param     = 0;
	//-------------------
	if(RING_BUFF_Flags()->f_receivedCR)//Принята строка
	{
		RING_BUFF_Flags()->f_receivedCR = FLAG_CLEAR;//Сброс признака принятой строки.
		RING_BUFF_CopyRxBuff(rxBuff);				 //кипируем принятую строку из колльцевого буфера.
		//-------------------
		//Команда управления моментом удержания ШД.
		charIndex = PARS_EqualWitchRefStr("mcu torque ", (char*)rxBuff);
		if(charIndex)
		{
			param = PARS_EqualWitchRefStr("on", (char*)&rxBuff[charIndex]);
			if(param) MOTOR_TorqueControl(MOTOR_TORQUE_ON);

			param = PARS_EqualWitchRefStr("off", (char*)&rxBuff[charIndex]);
			if(param) MOTOR_TorqueControl(MOTOR_TORQUE_OFF);

			//LED_ACT_Toggel();
			return;
		}
		//-------------------
		//команда вращения на заданный угол
		charIndex = PARS_EqualWitchRefStr("mcu rotate ", (char*)rxBuff);
		if(charIndex)
		{
			//Cледующий символ может быть или знаком "-" или числом от 0 до 9
			char ch = (char)rxBuff[charIndex];
			if((ch < '0' || ch > '9') && ch != '-') return;

			param = PARS_StrToI32((char*)&rxBuff[charIndex]);//параметр команды
			if(MOTOR_GetSpeedRampState() == STOP)
			{
				MOTOR_SetTargetPosition((int32_t)param);
				RTOS_SetTask(MOTOR_StartRotate, 0, 0);//запуск вращения.
			}

			//LED_ACT_Toggel();
			return;
		}
		//-------------------
		//Команда ....

		//-------------------
		//Команда ....

		//-------------------
		//Команда ....

		//-------------------
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void DBG_MotorStop(void){

	MOTOR_SetSpeedRampState(DECEL);
}
//************************************************************
void DBG_Motor(void){

	uint32_t RPM            = MOTOR_GetVelocity();        //38;    //Скорость оборотов в минуту
	uint32_t accelTime_mSec = MOTOR_GetAccelerationTime();//500;   //время ускорения в милисекундах
	uint32_t reducerRate    = MOTOR_GetReducerRate();     //6;     //передаточное число редуктора = 6.
	uint32_t angle		    = MOTOR_GetTargetPosition();  //360*2; //Угол на который нужно переместить вал камеры в градусах.

	//uint32_t accel_RPMM  = 4000; //Ускорение оборотов в минуту за минуту
	//Вариант расчета значений когда ускорение задается как время ускорения в мС.
	uint32_t accel_RPMM = (uint32_t)(RPM / ((float)accelTime_mSec / 1000 / 60.0));
	//-----------------------------------------------------
	//Пример расчета:
	//Макс.скорость - 10 RPM,   это = 10*2*Pi = 62.8319 рад/мин, или 1,0472 рад/сек.   1,0472/ 0,01 = 104,72 ~ 105 (in 0.01*rad/sec).
	//Ускорение     - 20 RPM^2, это = 20*2*Pi = 125.6637 рад/мин^2, или 0,035 рад/сек^2. 0,035 / 0,01 = 3,49   ~ 3	 (in 0.01*rad/sec^2).
	//Замедление    - 20 RPM^2, расчет тотже что и при ускорении.
	//Шаги - мне нужно повернуться на 360 градусов. Это 6400 шагов при microstep = 1/32

	// param step   Number of steps to move (pos - CW, neg - CCW).
	// param accel  Accelration to use, in 0.01*rad/sec^2.
	// param decel  Decelration to use, in 0.01*rad/sec^2.
	// param speed  Max speed, in 0.01*rad/sec.

	uint32_t microSteps 	   = (uint32_t)MOTOR_FULL_STEPS_PER_TURN * MOTOR_GetMicrostepMode();
	uint32_t steps			   = (uint32_t)(angle      * reducerRate * microSteps / (float)360.0);//кол-во шагов необходимое длч перемещение на угол angle
	uint32_t vel_rad_per_sec   = (uint32_t)(RPM        * reducerRate * _2PI / 60   * 100);		  //скорость рад в сек
	uint32_t accel_rad_per_sec = (uint32_t)(accel_RPMM * reducerRate * _2PI / 3600 * 100);		  //Ускорение рад в секунду за секунд

	MOTOR_TorqueControl(MOTOR_TORQUE_ON);  //вкл. момента.
	MOTOR_SpeedCntrMove(-1*steps, 		   //param step   Number of steps to move
						accel_rad_per_sec, //param accel  Accelration to use, in 0.01*rad/sec^2.
						accel_rad_per_sec, //param decel  Decelration to use, in 0.01*rad/sec^2.
						vel_rad_per_sec);  //param speed  Max speed, in 0.01*rad/sec.
	//-----------------------------------------------------
//	MOTOR_Move(360*6*2, 5, 35*6, 1500);
//	RTOS_SetTask(Task_MotorStop, 2000, 0);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void POWER_TurnOnAndSupplyVoltageCheck(void){

	//Ждем отпускания копки.
	//while(POWER_PwrButton() != RELEASE){};

	//Включение питания платы.
	MCU_POWER_ON();
	DELAY_milliS(100); //Задержка для стабилизации напряжения патания.

	//Напряжение НИЖЕ минимального (10,8В) напряжения АКБ.
	if(POWER_GetSupplyVoltage() < BATTERY_VOLTAGE_MIN)
	{
		//Зупуск таймера. Это нужно для мигающей индикации
		SysTick_Init();
		NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));//Системный таймер, прерывание каждую 1мс - самый низкий приоритет прерывания = 3
		NVIC_EnableIRQ(SysTick_IRQn);
		__enable_irq();
		Task_POWER_Check();//Отключение питания.
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
int main(void){

	//--------------------------
	//Инициализация периферии STM32.
 	STM32_Clock_Init();
	GPIO_Init();
	DELAY_Init();
	//--------------------------
	//Инициализация кнопки питания и АЦП для измерения напряжения АКБ.
	POWER_Init();
	//Включение питания платы. Смотрим что там с напряжением АКБ.
	POWER_TurnOnAndSupplyVoltageCheck();
	//Включение питания периферии.
	PWR_BTN_LED_High();
	FAN_EN_High();
	LIDAR_EN_High();
//	LAMP_PWM_High();
	DELAY_milliS(100); //Задержка для стабилизации напряжения патания.
	//--------------------------
	//Инициализация:
	TEMPERATURE_SENSE_Init(); //датчиков температуры.
	OPT_SENS_Init();		  //оптических дадчитков наличия крышки объетива и наличия АКБ.
	ENCODER_Init();			  //энкодера.
	PROTOCOL_I2C_Init();	  //протокола обмена.
	GPS_Init();				  //обмена с модулем GPS.
	MOTOR_Init();			  //драйвера мотора.
	//--------------------------
	//Иницияализация ШИМ для управления LAMP. Используется вывод PB1(TIM3_CH4).
	//TIM3_InitForPWM();

	//--------------------------
	//Ини-я диспетчера.
	RTOS_Init();
	RTOS_SetTask(Task_POWER_Check,   1000, 100);//Опрос кнопки питания и проверка напряжения питания каждые 100мс.
	RTOS_SetTask(Task_TEMPERATURE_Read, 0, 500);//Измерение температуры каждую 1сек.

	RTOS_SetTask(DBG_SendDebugInfo,		0, 500);//Передача отладочной информации.
	RTOS_SetTask(DBG_UsartCmdCheck,		0,  50);//Парсинг команд каждые 50 мс
//	RTOS_SetTask(DBG_Motor,             2000, 2*1000);
	//--------------------------
	SysTick_Init();
	SettingInterruptPriorities();//Приоритеты прерываний
	__enable_irq();				 //Глобальное разрешение прерываний
	//**************************************************************
	while(1)
	{
		RTOS_DispatchLoop();
		//__WFI(); //Sleep
	}
	//**************************************************************
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Прерывание каждую милисекунду.
void SysTick_IT_Handler(void){

	Blink_Loop();
	GPIO_CheckLoop();
	RTOS_TimerServiceLoop();//Служба таймеров.
	//Если нажали кнопку питания значит выключаемся и ничего не опрашиваем.
	if(POWER_Flags()->f_PowerOff)return;
	//--------------------------
	PROTOCOL_I2C_IncTimeoutAndResetI2c();//Таймаут переинициализации I2C в случае зависания.
	OPT_SENS_CheckLoop(); 	   			 //Опрос состояния сенсоров наличия крышки объектива и наличия АКБ.
	//--------------------------
	//Отладка!!!
	//MOTOR_TimerITHandler2();		  //Работа с мотром.
	ENCODER_DBG_CalcAngleAndSpeed();//Работа с энкодером. Для отладки данные предаются по USART.
	//--------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************











