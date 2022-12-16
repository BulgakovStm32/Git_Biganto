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
		MOTOR_DriverDisable();
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
//************************************************************
void Task_CheckRotate(void){

//	uint32_t encodPosition = ENCODER_GetAngle();
//	uint32_t motorPosition = MOTOR_GetMotorPosition();
//	//-------------------
//	if((encodPosition - motorPosition) > 100)
//	{
//		MOTOR_Disable();
//	}
//	else
//	{
//		MOTOR_Enable();
//	}
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



	//Передаточное число редуктора
	Txt_Print("ReducerRate   : ");
	Txt_BinToDec(MOTOR_GetReducerRate(), 4);
	Txt_Chr('\n');

	//Установленная скорость мотора
	Txt_Print("MotorSpeed    : ");
	Txt_BinToDec(MOTOR_GetVelocity(), 4);
	Txt_Print(" rpm\n");

	//Время ускорения/замедления
	Txt_Print("MotorAccel     : ");
	Txt_BinToDec(MOTOR_GetAccelerationTime(), 4);
	Txt_Print(" mS\n");

	//Целевой угол
	int32_t targetAngle = MOTOR_GetTargetPosition();
	Txt_Print("MotorTarget    : ");
	if(targetAngle < 0)
	{
		Txt_Chr('-');
		targetAngle = -targetAngle;
	}
	else Txt_Chr('+');
	Txt_BinToDec(targetAngle/1000, 5);
	Txt_Chr('.');
	Txt_BinToDec(targetAngle%1000, 3);
	Txt_Chr('\n');

	//Угол на котором началось замедление
	uint32_t motorPausa = MOTOR_GetPausaPosition();
	Txt_Print("MotorPausa    : ");
	Txt_BinToDec(motorPausa/1000, 5);
	Txt_Chr('.');
	Txt_BinToDec(motorPausa%1000, 3);
	Txt_Chr('\n');

	//Электрический угол мотора
	uint32_t motorPosition = MOTOR_GetMotorPosition();
	Txt_Print("MotorPosition : ");
	Txt_BinToDec(motorPosition/1000, 5);
	Txt_Chr('.');
	Txt_BinToDec(motorPosition%1000, 3);
	Txt_Chr('\n');



	//угол энкодера
	uint32_t encoderAngle = ENCODER_GetAngle();
	Txt_Print("EncoderAngle  : ");
	Txt_BinToDec(encoderAngle/1000, 5);
	Txt_Chr('.');
	Txt_BinToDec(encoderAngle%1000, 3);
	Txt_Chr('\n');

	//код энкодера
	Txt_Print("EncoderCode   : ");

	Txt_BinToDec(ENCODER_GetCode(), 6);
	Txt_Chr('\n');

	//кол-во граусов в одном шаге энкодера *1000000
	//выводится как угол в градусах на один шаг энкодер.
	Txt_Print("EncoderQuant  : 000.");
	Txt_BinToDec(ENCODER_GetAngleQuant(), 6);
	Txt_Chr('\n');

	//скорсть энкодера.
//	Txt_Print("EncoderRPM   : ");
//	Txt_BinToDec((uint32_t)(ENCODER_DBG_Data()->RPM), 6);
//	Txt_Chr('\n');


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
//	Txt_BinToDec((uint32_t)(ENCODER_DBG_Data()->RPM * 1000), 6);
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
				RTOS_SetTask(MOTOR_CalcAndStartRotation, 0, 0);//запуск вращения.
			}
			return;
		}
		//-------------------
		//Команда "Задать скорость вращения"
		charIndex = PARS_EqualWitchRefStr("mcu speed ", (char*)rxBuff);
		if(charIndex)
		{
			//Cледующий символ может быть числом от 0 до 9
			char ch = (char)rxBuff[charIndex];
			if(ch < '0' || ch > '9') return;

			param = PARS_StrToU32((char*)&rxBuff[charIndex]);//параметр команды
			if(MOTOR_GetSpeedRampState() == STOP)
			{
				MOTOR_SetVelocity((uint32_t)param);
			}
			return;
		}
		//-------------------
		//Команда "Задать время ускорения/замедления"
		charIndex = PARS_EqualWitchRefStr("mcu accel time ", (char*)rxBuff);
		if(charIndex)
		{
			//Cледующий символ может быть числом от 0 до 9
			char ch = (char)rxBuff[charIndex];
			if(ch < '0' || ch > '9') return;

			param = PARS_StrToU32((char*)&rxBuff[charIndex]);//параметр команды
			if(MOTOR_GetSpeedRampState() == STOP)
			{
				MOTOR_SetAccelerationTime((uint32_t)param);
			}
			return;
		}
		//-------------------
		//Команда "Пауза"
		charIndex = PARS_EqualWitchRefStr("mcu pausa", (char*)rxBuff);
		if(charIndex)
		{
			MOTOR_PausaRotation();
			return;
		}
		//-------------------
		//Команда "Снятие с паузы"
		charIndex = PARS_EqualWitchRefStr("mcu start", (char*)rxBuff);
		if(charIndex)
		{
			MOTOR_StartRotation();
			return;
		}
		//-------------------
		//Команда "Отключение питания"
		charIndex = PARS_EqualWitchRefStr("mcu turn off", (char*)rxBuff);
		if(charIndex)
		{
			POWER_TurnOff();
			return;
		}
		//-------------------
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void DBG_MotorStop(void){

}
//************************************************************
void DBG_Motor(void){

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
	RTOS_SetTask(Task_CheckRotate,      0, 50); //Проверка вращения.

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
		//Отключен запрет прерываний в диспетчере. Так стабильнее работает мотор.
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
	//ENCODER_DBG_CalcAngleAndSpeed();//Работа с энкодером. Для отладки данные предаются по USART.
	//--------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************











