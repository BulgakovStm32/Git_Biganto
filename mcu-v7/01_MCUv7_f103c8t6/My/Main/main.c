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
static void STM32_Init(void){

 	STM32_Clock_Init();
 	STM32_GPIO_Init();
 	DELAY_Init();
}
//**********************************************************
static void SettingInterruptPriorities(void){

	//Приоритеты прерываний.
	//Таймер управления мотором - самый высокий приоритет = 1.
	NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	//Прерывания I2C для работы протакола - приоритет = 2
	NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	//Системный таймер, прерывание каждую 1мс - самый низкий приоритет = 3
	//NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	//Разрешаем прерывания.
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(I2C2_ER_IRQn);
	//NVIC_EnableIRQ(SysTick_IRQn);
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
	if(flag) TEMPERATURE_SENSE_ReadData(1); //TEMPERATURE_SENSE1_ReadTemperature();
	else	 TEMPERATURE_SENSE_ReadData(2); //TEMPERATURE_SENSE2_ReadTemperature();
}
//************************************************************
//ф-я вызыввается каждые 100мс.
void Task_POWER_Check(void){

	uint32_t count;
	BlinkIntervalEnum_t blinkInterval = INTERVAL_100_mS;
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
		//Остановка если было вращение
		MOTOR_PausaRotation();
		while(MOTOR_GetSpeedRampState() != STOP);//Ждем полной остановки мотора

		//-------------------
		//Отключение питания периферии.
		MOTOR_DriverDisable();
		GPS_PowerControl(GPS_POWER_OFF);
		LAMP_PWM_Low();
		LIDAR_EN_Low();

		//Аппаратное откл. питания БигБорд через линию "BTN"
		BB_PWR_BTN_High();
		DELAY_mS(50);
		BB_PWR_BTN_Low();

		//Опрос питания БигБорд.
		count = RTOS_GetTickCount();
		while(POWER_GetBigBoardPwr())
		{
			//Мигающая индикация отключения питания.
			POWER_PwrButtonLed(Blink(INTERVAL_500_mS));

			//Нажатие на кнопку в процессе отключения камеры.
			if(POWER_PwrButton() == PRESS)
			{
				//Аппаратное откл. питания БигБорд через линию "BTN"
				//Это нужно т.к. можно нажать кнопку отключения питания
				//еще до того как Хавьер загрузился.
				BB_PWR_BTN_High();
				DELAY_mS(50);
				BB_PWR_BTN_Low();

				//Длительное нажатие на кнопку (5 сек).
				if((RTOS_GetTickCount() - count) >= 5000) goto POWER_OFF;
			}
			else count = RTOS_GetTickCount();

			//Проверка напряжения АКБ.
			POWER_CheckSupplyVoltage();
			if(POWER_Flags()->f_BatteryLow)//Низкое напряжениe АКБ.
			{
				blinkInterval = INTERVAL_50_mS;
				break;
			}
		}
		//Частая мигающая индикация отключения в течении 2 сек.
		while((RTOS_GetTickCount() - count) <= 2000)
		{
			POWER_PwrButtonLed(Blink(blinkInterval));
		}
		//-------------------
		POWER_OFF:
		BB_PWR_EN_Low();
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

		   uint32_t currPos;
	static uint32_t oldPos 	 = 0;
		    int32_t delta  	 = 0;
		   uint32_t motorPos = 0;
	//-------------------
	motorPos = (MOTOR_GetVelocity() * MOTOR_GetReducerRate() * 64 + 30) / 60;

	if(MOTOR_GetSpeedRampState() != STOP &&  //Началось вращение
	   MOTOR_GetMotorPosition()  >= motorPos)//и мотор сдела несколько шагов
	{
		//Позиция энкодера
		currPos = ENCODER_GetAngle();

		//Опередлин направления вращения
		if(MOTOR_GetTargetPosition() < 0) delta = (int32_t)(currPos - oldPos); //По часовой
		else							  delta = (int32_t)(oldPos - currPos); //Против часовой

		//Уберем знак и отнимем 360 градусов
		if(delta < 0)
		{
			//delta = -delta;
			//delta = 360000 - delta;
			delta = delta + 360000;
		}

		//сохраним положение
		oldPos = currPos;

		//Проверим приращение угла.
		if(delta <= 10)
		{
			MOTOR_StopRotation();
			oldPos = 0;
		}
	}
	//Пока нет вращения сохраняем угол энкодера
//	else
//	{
//		oldPos = ENCODER_GetAngle();
//	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void DBG_SendDebugInfo(void){

	FAN_EN_High();


	static uint32_t flag = 0;

	if(!flag) POWER_SetPWMwidthForLamp(5);
	else      POWER_SetPWMwidthForLamp(1);
	flag ^= 1;

	//--------------------------------
	//Txt_Chr('\f');
	Txt_Print("\f//********************************\n");
	Txt_Print("_MCUv7_BT_rev8_\n");

	//Количество зависаний I2C
	Txt_Print("i2c_reInit: ");
	Txt_BinToDec(PROTOCOL_I2C_GetResetCount(), 6);
	Txt_Chr('\n');

	//Счетчик миллисекунд MCU
	Txt_Print("time_mS  : ");
	Txt_BinToDec(RTOS_GetTickCount(), 9);
	Txt_Chr('\n');

	//Вывод Напряжения питания
	uint32_t v = POWER_GetSupplyVoltage();
	Txt_Print("Vin: ");
	Txt_BinToDec(v/1000, 2);
	Txt_Chr('.');
	Txt_BinToDec(v%1000, 3);
	Txt_Print(" V\n");

	//Состояние питания MCU.
		 if(POWER_Flags()->f_BatteryWarning) Txt_Print("BATTERY < 12v!!!\n");
	else if(POWER_Flags()->f_BatteryLow) 	 Txt_Print("BATTERY LOW!!!\n");
	else if(POWER_Flags()->f_PowerOff)   	 Txt_Print("POWER OFF!!!\n");
	else									 Txt_Print("BATTERY OK\n");

	//Вывод температуры.
	int16_t tSense = TEMPERATURE_SENSE_GetTemp(1);	//TEMPERATURE_SENSE_GetSens(1)->temperature;
	Txt_Print("ds1: ");
	if(tSense != DS18B20_NO_SENSE)
	{
		if(tSense < 0)
		{
			tSense = -tSense;
			Txt_Chr('-');
		}
		else Txt_Chr('+');

		Txt_BinToDec(tSense/10, 2);
		Txt_Chr('.');
		Txt_BinToDec(tSense%10, 1);
		Txt_Print(" C");
	}
	else Txt_Print("err");
	Txt_Chr('\n');

	tSense = TEMPERATURE_SENSE_GetTemp(2);//TEMPERATURE_SENSE_GetSens(2)->temperature;
	Txt_Print("ds2: ");
	if(tSense != DS18B20_NO_SENSE)
	{
		if(tSense < 0)
		{
			tSense = -tSense;
			Txt_Chr('-');
		}
		else Txt_Chr('+');

		Txt_BinToDec(tSense/10, 2);
		Txt_Chr('.');
		Txt_BinToDec(tSense%10, 1);
		Txt_Print(" C");
	}
	else Txt_Print("err");
	Txt_Print("\n\n");
	//--------------------------------
	//Передаточное число редуктора
	Txt_Print("ReducerRate  : ");
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

	//Состояние рампы разгона торможения мотора
	MotorState_t mState = MOTOR_GetSpeedRampState();
	Txt_Print("MotorState     : ");
	switch(mState){
		case(STOP) : Txt_Print("STOP") ; break;
		case(ACCEL): Txt_Print("ACCEL"); break;
		case(RUN)  : Txt_Print("RUN")  ; break;
		case(DECEL): Txt_Print("DECEL"); break;
		default:	 Txt_Print("undef..");
	}
	Txt_Chr('\n');

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
	Txt_Print("MotorPosition  : ");
	Txt_BinToDec(motorPosition/1000, 5);
	Txt_Chr('.');
	Txt_BinToDec(motorPosition%1000, 3);
	Txt_Print("\n\n");
	//--------------------------------
	//Данные энкодера
	Txt_Print("Encoder :\n");

	//угол энкодера
	uint32_t encoderAngle = ENCODER_GetAngle();
	Txt_Print("Angle  : ");
	Txt_BinToDec(encoderAngle/1000, 3);
	Txt_Chr('.');
	Txt_BinToDec(encoderAngle%1000, 3);
	Txt_Chr('\n');

	//код энкодера
	Txt_Print("Code   : ");
	Txt_BinToDec(ENCODER_GetCode(), 5);
	Txt_Chr('\n');

	//кол-во граусов в одном шаге энкодера *1000000
	//выводится как угол в градусах на один шаг энкодер.
	Txt_Print("Quant  : 0.");
	Txt_BinToDec(ENCODER_GetAngleQuant(), 6);
	Txt_Chr('\n');

	Txt_Print("//********************************\n");
	//--------------------------------
	//USART1_TX -> DMA1_Channel4
	//USART2_TX -> DMA1_Channel7
	DMAxChxStartTx(DMA1_Channel7, Txt_Buf()->buf, Txt_Buf()->bufIndex);
	Txt_Buf()->bufIndex = 0;



	FAN_EN_Low();
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
				//RTOS_SetTask(MOTOR_CalcAndStartRotation, 0, 0);//запуск вращения.
				MOTOR_CalcAndStartRotation();
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
			MOTOR_SetVelocity((uint32_t)param);
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
			MOTOR_SetAccelerationTime((uint32_t)param);
			return;
		}
		//-------------------
		//Команда "Задать передаточное число редуктора"
		charIndex = PARS_EqualWitchRefStr("mcu reducer rate ", (char*)rxBuff);
		if(charIndex)
		{
			//Cледующий символ может быть числом от 0 до 9
			char ch = (char)rxBuff[charIndex];
			if(ch < '0' || ch > '9') return;

			param = PARS_StrToU32((char*)&rxBuff[charIndex]);//параметр команды
			MOTOR_SetReducerRate((uint32_t)param);
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
		//Команда "Ресет"
		charIndex = PARS_EqualWitchRefStr("mcu reset", (char*)rxBuff);
		if(charIndex)
		{
			MOTOR_PausaRotation();					 //если вращались, то плавно остановимся
			while(MOTOR_GetSpeedRampState() != STOP);//ждем останоку мотора
			NVIC_SystemReset();                      //сброс микроконтроллера.
		}
		//-------------------
		//Команда "Переход в загрузчик"
		charIndex = PARS_EqualWitchRefStr("mcu boot", (char*)rxBuff);
		if(charIndex)
		{
			SharedMemory_SetFlag(DFU_REQUESTED_FLAG);
			MOTOR_PausaRotation();					 //если вращались, то плавно остановимся
			while(MOTOR_GetSpeedRampState() != STOP);//ждем останоку мотора
			NVIC_SystemReset();                      //сброс микроконтроллера.
		}
		//-------------------
	}
}
//************************************************************
void DBG_SendEncoderData(void){

	if(MOTOR_GetSpeedRampState() != STOP)
	{
		//угол энкодера
		uint32_t encoderAngle = ENCODER_GetAngle();
		//Txt_Print("EncoderAngle  : ");
		Txt_BinToDec(encoderAngle/1000, 3);
		Txt_Chr(',');
		Txt_BinToDec(encoderAngle%1000, 3);
		Txt_Chr('\n');
		//--------------------------------
		//USART1_TX -> DMA1_Channel4
		//USART2_TX -> DMA1_Channel7
		DMAxChxStartTx(DMA1_Channel7, Txt_Buf()->buf, Txt_Buf()->bufIndex);
		Txt_Buf()->bufIndex = 0;
	}
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
	DELAY_mS(100); //Задержка для стабилизации напряжения патания.

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
//Метаданные приложения
appMetadata_t appMetadata __attribute__((section(".app_metadata"))) = {

	.metadataVersion  = 0x1234,
	.appMagic		  = METADATA_MAGIC,
	.appType 		  = 0x56,
	.appVectorAddr 	  = 0x08002800,   //вектор сброса приложения
	.appVersion_major = 0x07,
	.appVersion_minor = 0x08,
	.appVersion_patch = 0x09,

	// GIT_SHA is generated by Makefile
	.git_sha = "12345678",

	// populated as part of a post compilation step
	.appSize = 0x12345678,
	.appCrc  = 0xabcdefde,
};
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
int main(void){

	//--------------------------
	//Это необходимо для работы с загрузчиком.
	//BOOT_Init();
	//--------------------------
	STM32_Init();	 //Инициализация STM32.
 	GPIO_MCU_Init(); //Инициализация портов MCU.
	//--------------------------
	//Инициализация кнопки питания и АЦП для измерения напряжения АКБ.
	POWER_Init();
	//Включение питания платы. Смотрим что там с напряжением АКБ.
	POWER_TurnOnAndSupplyVoltageCheck();
	//Включение питания периферии.
	PWR_BTN_LED_High();
	FAN_EN_High();
	LIDAR_EN_High();

	DELAY_mS(100); //Задержка для стабилизации напряжения патания.
	//--------------------------
	//Инициализация:
	TEMPERATURE_SENSE_Init();	//датчиков температуры.
	OPT_SENS_Init();		  	//оптических дадчитков наличия крышки объетива и наличия АКБ.
	ENCODER_Init();			  	//энкодера.
	PROTOCOL_I2C_Init();	  	//протокола обмена.
	GPS_Init();				  	//обмена с модулем GPS.
	MOTOR_Init();			  	//драйвера мотора.
	//--------------------------
	//Ини-я диспетчера. Так же инициализируется системный таймер на прерывания каждую 1 мс.
	RTOS_Init();

	RTOS_SetTask(Task_POWER_Check,   1000, 100); //Опрос кнопки питания и проверка напряжения питания каждые 100мс.
	RTOS_SetTask(Task_TEMPERATURE_Read, 0, 500); //Измерение температуры каждую 1сек.
	//RTOS_SetTask(Task_CheckRotate,      0, 10);  //Проверка вращения. Не отлажена!!!

	//Отладочные задачи.
	RTOS_SetTask(DBG_SendDebugInfo,	 0, 500);	//Передача отладочной информации.
	RTOS_SetTask(DBG_UsartCmdCheck,	50, 50);	//Парсинг команд каждые 50 мс
	//RTOS_SetTask(DBG_SendEncoderData, 50, 10);//Передача данных энкодера
	//--------------------------
	SettingInterruptPriorities();				//Приоритеты прерываний
	__enable_irq();				 				//Глобальное разрешение прерываний
	//**************************************************************
	while(1)
	{
		//Отключен запрет прерываний в диспетчере.
		//Так стабильнее работает мотор.
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
void RTOS_SysTickCallback(void){

	Blink_Loop();
	STM32_GPIO_CheckLoop();
	//Если нажали кнопку питания значит выключаемся и ничего не опрашиваем.
	if(POWER_Flags()->f_PowerOff)return;
	//--------------------------
	PROTOCOL_I2C_CheckTimeout();	//Таймаут переинициализации I2C в случае зависания.
	OPT_SENS_CheckLoop(); 	   		//Опрос состояния сенсоров наличия крышки объектива и наличия АКБ.
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











