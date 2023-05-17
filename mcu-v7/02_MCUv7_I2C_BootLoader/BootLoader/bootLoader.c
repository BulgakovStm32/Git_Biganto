/*
 * bootLoader.c
 *
 *  Created on: 10 февр. 2023 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "bootLoader.h"

//*******************************************************************************************
//*******************************************************************************************
static uint8_t 	bootBuf[1024+16] = {0};	//Рабочий буфер загрузчика
static I2C_IT_t	I2c;					//Рабочий I2C

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
// Function    : _waitEndRx
// Description : Ожидание приема данных от Хоста.
//				(Ждем от Мастера STOP - признак завершения записи байтов.)
// Parameters  :
// RetVal      :
//*****************************************
static void _waitEndRx(void){

	uint32_t micros = DELAY_microSecCount();
	//------------------
	I2C_IT_InterruptEnable(&I2c);			//включаем прерывания
	while(I2c.state != I2C_IT_STATE_STOP)	//Ждем завершения чтения данных
	{
		//Сброс сторожевого таймера
		//IWDG_Reset();
		//WWDG_Reset();
		//Мигаем...
		if((DELAY_microSecCount() - micros) >= 500000)
		{
			micros = DELAY_microSecCount();
			BOOT_INDICATOR_Toggle();
		}

		//Отключение питания MCUv7 по нажатию на кнопку
		if(GPIO_MCU_GetPwrButton() == 0)
		{
			//ждем отпускания кнопки
			while(GPIO_MCU_GetPwrButton() == 0);
			//Отключение питания
			MCU_POWER_OFF();
			while(1);
		}
	}
	I2c.state = I2C_IT_STATE_READY;	//Сброс состояния I2C
}
//*******************************************************************************************
// Function    : _sendBuf
// Description : отправить ответ мастеру
// Parameters  : size - кол-во байтов ответа
// RetVal      :
//*****************************************
static void _sendBuf(uint32_t size){

	I2C_IT_SetDataSize(&I2c, size);			//кол-во байт на передачу
	I2C_IT_InterruptEnable(&I2c);			//включаем прерывания

	while(I2c.state != I2C_IT_STATE_NAC);	//Ждем завершения передачи кадра данных
	I2c.state = I2C_IT_STATE_READY;			//Сброс состояния I2C
}
//*******************************************************************************************
// Function    : _send_Byte
// Description : отправить ответ мастеру
// Parameters  : byte - байт ответа
// RetVal      :
//*****************************************
static void _sendByte(uint8_t byte){

	bootBuf[0] = byte;						//
	I2C_IT_SetDataSize(&I2c, 1);			//1 байт на передачу
	I2C_IT_InterruptEnable(&I2c);			//включаем прерывания

	while(I2c.state != I2C_IT_STATE_NAC);	//Ждем завершения передачи кадра данных
	I2c.state = I2C_IT_STATE_READY;			//Сброс состояния I2C
}
//*******************************************************************************************
// Function    : _getChecksum()
// Description : расчет контрольной суммы. Полученные блоки байтов данных подвергаются операции XOR.
//			     Байт, содержащий вычисленное XOR всех предыдущих байтов, добавляется в конец каждого
//				 сообщения (байт контрольной суммы). При выполнении XOR всех полученных байтов данных и
//				 контрольной суммы результат в конце пакета должен быть 0x00.
// Parameters  : buf  - указатель на буфер
//				 size - кол-во байтов для расчета, от 1 байта
// RetVal      : XOR байтов
//*****************************************
static uint8_t _getChecksum(uint8_t *buf, uint32_t size){

	uint8_t xor = buf[0];
	//---------------------
	//Checksum: XOR (byte 0, byte 1, byte 2, byte 3)
	//bootBuf[4] = bootBuf[0] ^ bootBuf[1] ^ bootBuf[2] ^ bootBuf[3];
	for(uint32_t i=1; i < size; i++)
	{
		xor ^= buf[i];
	}
	return xor;
}
//*******************************************************************************************
// Function    : _getStartAddress()
// Description : собирает 4-хбайтовый адрес из принятых байтов
// Parameters  : buf - указатель на младший байт принятого адреса
// RetVal      : 4-хбайтовый адрес
//*****************************************
static uint32_t _getStartAddress(uint8_t *buf){

	return (uint32_t)((buf[0]<<24) |
			  	  	  (buf[1]<<16) |
					  (buf[2]<< 8) |
					  (buf[3]<< 0));
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
// Function    : _cmd_BOOT_Get()
// Description : Команда Get позволяет узнать версию bootloader и поддерживаемые команды.
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_Get(void){

	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	//Сборка кадра данных
	bootBuf[0]	= 11; //(11 - 1);			//N = Number of bytes to follow - 1, except current and ACKs
	bootBuf[1]	= BOOT_I2C_VERSION;	//bootloader version
	//Поддерживаемые команды
	bootBuf[2] 	= CMD_BOOT_Get;
	bootBuf[3] 	= CMD_BOOT_GetVersion;
	bootBuf[4] 	= CMD_BOOT_GetID;
	bootBuf[5] 	= CMD_BOOT_RM;
	bootBuf[6] 	= CMD_BOOT_GO;
	bootBuf[7] 	= CMD_BOOT_WM;
	bootBuf[8] 	= CMD_BOOT_ERASE;
	bootBuf[9]  = CMD_BOOT_WP;	//Нереализована!!!

	//!!! Опасные команды !!!
	bootBuf[10]	= CMD_BOOT_RP;
	bootBuf[11]	= CMD_BOOT_RUP;

	bootBuf[12] = CMD_BOOT_NS_GetCheckSum;

	_sendBuf(13); //кол-во байт на передачу
	//-----------------------
	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
// Function    : _cmd_CMD_BOOT_GetVersion()
// Description : Команда используется для получения версии загрузчика.
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_GetVersion(void){

	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	//Передача версии загрузчика
	_sendByte(BOOT_I2C_VERSION);
	//-----------------------
	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
// Function    : _cmd_BOOT_GetID()
// Description : Команда Get ID используется для получения идентификатора чипа.
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_GetID(void){

	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	//Передача кадра данных - идентификатор чипа

	//DBGMCU_IDCODE - регистр кода микроконтроллера - Регистры отладки DBGMCU_IDCODE и DBGMCU_CR
	//доступны только в режиме отладки (недоступны для программного обеспечения пользователя).
	//Младшие 12 бит содержат код микроконтроллера.
	//volatile uint32_t pid = (uint32_t)(DBGMCU->IDCODE & 0x00000FFF);

	bootBuf[0] = 1;		//N = the number of bytes-1 (for STM32, N = 1)
	bootBuf[1] = (uint8_t)(BOOT_I2C_STM32_PID >> 8);	//product ID MSB
	bootBuf[2] = (uint8_t)(BOOT_I2C_STM32_PID >> 0);  	//product ID LSB
	_sendBuf(3);		//3 байта на передачу
	//-----------------------
	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
// Function    : _cmd_BOOT_RM()
// Description : Команда Read Memory используется для чтения данных из любого допустимого
//				 адреса (см. даташит на используемый STM32 и апноут AN2606 [2]) в RAM,
//				 памяти Flash и информационного блока (system memory или область байта опций).
// Parameters  : None
// RetVal      :1 - команда выполнена; 0 - команда не выполнена.
//*****************************************
uint8_t _cmd_RM(void){

	uint32_t startAddr;
	uint32_t size;
	//-----------------------
	//Проверка ROP (Readout Protect) и передача ACK/NACK
	if(STM32_Flash_GetReadOutProtectionStatus() == 1)//ROP активна
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//ROP отключена
	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Receive data frame: start address (4 bytes) with checksum
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	//Проверка Сhecksum
	if(_getChecksum(bootBuf, 5) != 0)//Контрольная сумма не совпала
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//Контрольная сумма OK
	startAddr = _getStartAddress(bootBuf);//Соберам принятый адрес. Bytes 3-6: Start address (byte 3: MSB, byte 6: LSB)
	_sendByte(CMD_ACK);					  //Передача ACK
	//-----------------------
	//Receive data frame: number of bytes to be read (1 byte) and a checksum (1 byte)
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	//Send data frame: number of bytes to be read (1 byte) and a checksum (1 byte)
	//bootBuf[0] = (uint8_t)(size - 1);		  //The number of bytes to be read-1
	//bootBuf[1] = (uint8_t)(size - 1) ^ 0xFF;//Checksum

	//Проверка Сhecksum
	if(_getChecksum(bootBuf, 2) != 0xFF)//Контрольная сумма не совпала
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//Контрольная сумма OK
	size = bootBuf[0] + 1;	//Сохраняем количество передаваемых данных
	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Send data frame: requested data to the host

	//Чтение данных из флэш-памяти.
	STM32_Flash_ReadBufU8((void*)startAddr, (void*)bootBuf, size);//копируем нужное кол-во байтов в буфер передачи

	//TODO ... Чтение из любого допустимого адреса
	//(см. даташит на используемый STM32 и апноут AN2606 [2])
	//в RAM, памяти Flash и информационного блока (system memory или область байта опций).

	_sendBuf(size);//кол-во байт на передачу
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
// Function    : _cmd_BOOT_Go()
// Description : Команда Go используется для запуска загруженного или любого другого кода
//				 путем безусловного перехода по указанному адресу.
// Parameters  : None
// RetVal      :1 - команда выполнена; 0 - команда не выполнена.
//*****************************************
uint8_t _cmd_GO(void){

	uint32_t appAddr;
	//-----------------------
	//Проверка ROP (Readout Protect) и передача ACK/NACK
	if(STM32_Flash_GetReadOutProtectionStatus() == 1)//ROP активна
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//ROP отключена
	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Receive data frame: start address (4 bytes) and checksum
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	//Проверка Сhecksum
	if(_getChecksum(bootBuf, 5) != 0)//Контрольная сумма не совпала
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//Контрольная сумма OK. Соберам принятый адрес.
	appAddr = _getStartAddress(bootBuf);
	//-----------------------
	//TODO ... запуск любого другого кода путем безусловного перехода по указанному адресу

	//У хоста есть команда -R Reset device at exit.
	//При этом хост посылает адрес перехода 0x2000 0200
	if(appAddr == 0x20000200)
	{
		_sendByte(CMD_ACK);	//Передача ACK
		BOOT_DeinitAll();	//Сброс всей периферии
		NVIC_SystemReset();	//Cистемный сброс!!!
	}

	//Запуск загруженного кода приложения.
	if(appAddr == APP_PROGRAM_START_ADDR)
	{
		//Проверка наличия приложения.
		if(BOOT_AppAvailableCheck(appAddr))
		{
			_sendByte(CMD_ACK);	//Передача ACK
			//---------------
			//BOOT_ErasePageAppState();				//Сбросим признак запуска основного приложения
			//BOOT_SetAppLaunch(APP_OK_AND_START);	//Установим признак запуска приложения после ресета
			//BOOT_CalcAndWriteAppCrc();			//Расчет и запись во флеш CRC приложения
			BOOT_GoToApp(APP_PROGRAM_START_ADDR);	//Переход в приложение
		}
	}
	//-----------------------
	_sendByte(CMD_NACK);	//Передача NACK
	return 0; 				//команда не выполнена.
}
//*******************************************************************************************
// Function    : _cmd_BOOT_WM(void)()
// Description : Команда WM (Write Memory) используется для записи данных во Flash.
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_WM(void){

	uint32_t startAddr;
	uint32_t nBytesMinusOne;
	//-----------------------
	//Проверка ROP (Readout Protect) и передача ACK/NACK
	if(STM32_Flash_GetReadOutProtectionStatus() == 1)//ROP активна
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//ROP отключена
	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Receive data frame: start address (4 bytes) with checksum
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	//Проверка Сhecksum
	if(_getChecksum(bootBuf, 5) != 0)//Контрольная сумма не совпала
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//Контрольная сумма OK. Соберам принятый адрес.
	//Bytes 3-6: Start address (byte 3: MSB, byte 6: LSB)
	startAddr = _getStartAddress(bootBuf);
	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Receive data frame:
	//- number of bytes to be written
	//- data to be written
	//- checksum
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	nBytesMinusOne = bootBuf[0]; //N - количество записываемых байт минус 1 (количество принимаемых байтов = N+1),
	//Проверка Сhecksum
	//+3 т.к. +байт_N +(N+1)байтов_данных и +байт Сhecksum
	if(_getChecksum(bootBuf, (nBytesMinusOne+1+1+1)) != 0)//Контрольная сумма не совпала
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//Контрольная сумма OK. Проверка принятого адреса на валидность.
	//Адреса могут быть:
	// - Main memory   			- 0x0800 0000 - 0x0801 FFFF (128 KB)
	// - System memory 			- 0x1FFF F000 - 0x1FFF F7FF (2 KB)
	// - Option Bytes  			- 0x1FFF F800 - 0x1FFF F80F (16 bytes)
	// - Flash memory registers - 0x4002 2000 - 0x4002 2023 (36 bytes)
	// - Peripheral				- 0x4000 0000 - 0x4FFF FFFF

	//Пишем фо FLASH - Main memory.
	if(startAddr >= APP_METADATA_ADDR)
	{
		//Записываем полученные данные во флэш-память
		STM32_Flash_Unlock();
		STM32_Flash_WriteBuf((void*)&bootBuf[1], (void*)startAddr, nBytesMinusOne+1);
		STM32_Flash_Lock();
	}
	//TODO ... Data written in Option bytes ?
//	else if(startAddr >= 0x1FFFF800 && startAddr < 0x1FFFF80F)
//	{
//		//Если команда Write Memory производит запись в область байтов
//		//управления (the option byte area), то все байты в этой области
//		//будут стерты перед записью новых значений, и по завершению
//		//команды загрузчик генерирует системный сброс, чтобы установки
//		//новых значений байт опций вступили в силу.
//
//		//_sendByte(CMD_ACK);	//Передача ACK
//		//BOOT_DeinitAll();		//Деинициализация всей периферии
//		//NVIC_SystemReset();	//Cистемный сброс!!!
//	}
	//Неверный адрес
	else
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
// Function    : _cmd_RP() --- Не отлажена!!!
// Description : Команда Readout Protect используется для разрешения защиты от чтения памяти Flash (RDP).
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_RP(void){

	//Проверка ROP (Readout Protect) и передача ACK/NACK
	if(STM32_Flash_GetReadOutProtectionStatus() == 1)//ROP активна
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//ROP отключена
	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Активация защиты от чтения
	STM32_Flash_Unlock();
	STM32_Flash_ReadOutProtection(FLASH_RDO_ENABLE);
	STM32_Flash_Lock();

	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Деинициализация всей периферии
	BOOT_DeinitAll();

	//Генерация сброса микроконтроллера.
	NVIC_SystemReset();
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
// Function    : _cmd_RUP() --- Не отлажена!!!
// Description : Команда Readout Unprotect используется для запрета (отмены) защиты от чтения памяти Flash (RDP).
//				 Когда загрузчик получит команду Readout Unprotect command, он передаст хосту байт ACK.
//				 После передачи байта ACK загрузчик сотрет всю память Flash и деактивирует защиту от чтения
//				 памяти Flash. Если операция стирания была успешной, загрузчик деактивирует RDP.
//				 !!!эта команда запустит процесс стирание ВСЕЙ флэш памяти!!!
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_RUP(void){

	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	//Отключение защиты от чтения - это запустит процесс стирание ВСЕЙ флэш памяти!!!
	STM32_Flash_Unlock();
	STM32_Flash_ReadOutProtection(FLASH_RDO_DISABLE);
	STM32_Flash_Lock();

	_sendByte(CMD_ACK);		//Передача ACK
	//-----------------------
	//Очистка всей RAM-памяти

	//-----------------------
	//Деинициализация всей периферии
	BOOT_DeinitAll();

	//Генерация сброса микроконтроллера.
	NVIC_SystemReset();
	//-----------------------
	return 1; //команда выполнена.
}









//*******************************************************************************************
// Function    : _cmd_ERASE() --- Отлажена!!!
// Description : Стирает от одной до всех страниц или секторов флэш-памяти,
//				 используя режим двухбайтовой адресации.
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_ERASE(void){

	uint32_t pageAddr;
	uint32_t nPages;
	//-----------------------
	//Проверка ROP (Readout Protect) и передача ACK/NACK
	if(STM32_Flash_GetReadOutProtectionStatus() == 1)//ROP активна
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//ROP отключена. Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	//Receive data frame:
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	//Байты 0 и 1: количество стираемых страниц минус 1
	nPages = (uint32_t)((bootBuf[0]<<8) | bootBuf[1])+1;//кол-во страниц

	//Проверка Сhecksum
	if(_getChecksum(bootBuf, (nPages*2 + 2 + 1)) != 0)
	{
		//Контрольная сумма не совпала
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//Контрольная сумма OK.Стираем страницы памяти.
	STM32_Flash_Unlock();
	for(uint32_t i=0; i < nPages; i++)
	{
		//начиная с 2го байта (bootBuf[2]) идут номера страниц
		pageAddr = (uint32_t)((bootBuf[i*2+2]<<8) | bootBuf[i*2+2+1]);//Номер стираемой страницы
		pageAddr = pageAddr & 0x0000FFFF;							//Маска
		pageAddr = pageAddr * 1024;									//смещение страницы относительно базового адреса
		pageAddr = pageAddr + FLASH_BASE;							//адрес стираемой страницы
		//Запрещено стирание страниц с 0 по 8, в них записан загрузчик.
		if(pageAddr < APP_METADATA_ADDR)
		{
			STM32_Flash_Lock();
			_sendByte(CMD_NACK);//Передача NACK
			return 0; 			//команда не выполнена.
		}
		//Стирание одной страницы flash
		STM32_Flash_ErasePage(pageAddr);
	}
	STM32_Flash_Lock();
	//Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	return 1; //команда выполнена.




//	//Проверка Сhecksum
//	if(_getChecksum(bootBuf, 3) != 0)//Контрольная сумма не совпала
//	{
//		_sendByte(CMD_NACK);//Передача NACK
//		return 0; 			//команда не выполнена.
//	}
//	//Контрольная сумма OK. Сохраним кол-во страниц и передадим ACK
//	nPages = (uint32_t)((bootBuf[0]<<8) | bootBuf[1])+1;//кол-во страниц
//	_sendByte(CMD_ACK);
	//-----------------------
	//- в случае стирания N страниц или секторов (это наш случай) загрузчик получает (2 x N) байтов,
	//каждое полуслово которых содержит номер страницы или сектора, закодированный двумя байтами,
	//причем старший бит идет первым. Затем передается байт контрольгой суммы. Если контрольная сумма верна,
	//загрузчик стирает память и отправляет хосту байт ACK, в противном случае он отправляет байт NACK хосту и команда прерывается.
	//Example of I2C frame:
	//– erase page 1: 0x44 0xBB Wait ACK 0x00 0x00 0x00 Wait ACK 0x00 0x01 0x01 Wait ACK
	//– erase page 1 and page 2:
	//0x44 0xBB Wait ACK 0x00 0x01 0x01 Wait ACK 0x00 0x01 0x00 0x02 0x03 Wait ACK

//	//Принимаем номера страниц и Сhecksum.
//	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.
//
//	//Проверка Сhecksum
//	if(_getChecksum(bootBuf, nPages*2+1) != 0)//Контрольная сумма не совпала
//	{
//		_sendByte(CMD_NACK);//Передача NACK
//		return 0; 			//команда не выполнена.
//	}
//	//Контрольная сумма OK. Стираем страницы памяти.
//	STM32_Flash_Unlock();
//	for(uint32_t i=0; i < nPages; i++)
//	{
//		pageAddr = (uint32_t)((bootBuf[i*2]<<8) | bootBuf[i*2+1]);	//Номер стираемой страницы
//		pageAddr = pageAddr & 0x000000FF;							//Маска
//		pageAddr = pageAddr * 1024;									//смещение страницы относительно базового адреса
//		pageAddr = pageAddr + FLASH_BASE;							//адрес стираемой страницы
//
//		//Запрещено стирание страниц с 0 по 8,
//		//в них записан загрузчик.
//		if(pageAddr < APP_STATE_ADDR)
//		{
//			STM32_Flash_Lock();
//			_sendByte(CMD_NACK);//Передача NACK
//			return 0; 			//команда не выполнена.
//		}
//
//		STM32_Flash_ErasePage(pageAddr);	//Стирание одной страницы flash
//	}
//	STM32_Flash_Lock();
//	//Передача ACK
//	_sendByte(CMD_ACK);
//	//-----------------------
//	return 1; //команда выполнена.



}
//*******************************************************************************************
// Function    : _cmd_NS_GetMemCs --- Отлажена!!!
// Description : Получает значение контрольной суммы CRC для области памяти.
// Parameters  : None
// RetVal      : 1 - команда выполнена; 0 - команда выполняется.
//*****************************************
uint8_t _cmd_NS_GetCheckSum(void){

	uint32_t addr;
	uint32_t size;
	uint32_t crc;
	//-----------------------
	//Проверка ROP (Readout Protect) и передача ACK/NACK
	if(STM32_Flash_GetReadOutProtectionStatus() == 1)//ROP активна
	{
		_sendByte(CMD_NACK);//Передача NACK
		return 0; 			//команда не выполнена.
	}
	//ROP отключена. Передача ACK
	_sendByte(CMD_ACK);
	//-----------------------
	//Receive address (4 bytes, MSB first) and its XOR
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	addr = _getStartAddress(bootBuf);	//Соберем адрес.

	//Address valid, within the memory, and XOR OK?
	if(_getChecksum(bootBuf, 5) != 0 ||	//Контрольная сумма не совпала
	   addr < FLASH_BASE)				//Неверный адрес
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//Контрольная сумма OK и верный адрес
	_sendByte(CMD_ACK);				  	//Передача ACK
	//-----------------------
	//Receive memory size to check (4 bytes, MSB first) and its XOR
	_waitEndRx();	//Ждем завершения приема фрейма данных от хоста.

	size = _getStartAddress(bootBuf);	//Соберем размер.

	//XOR OK, Size != 0 and multiple of 4, (Address + Size) valid ?
	if(_getChecksum(bootBuf, 5) != 0 ||	//Контрольная сумма не совпала
	    size == 0					 || //размер = 0
	   (size % 4) != 0				 || //некратно 4м
	   (addr + size) > FLASH_BANK1_END) //Address+Size больше размера флеш памяти
	{
		_sendByte(CMD_NACK);	//Передача NACK
		return 0; 				//команда не выполнена.
	}
	//Все ОК!
	_sendByte(CMD_ACK);	//Передача ACK
	//-----------------------
	//Calculate checksum. Operation completed? Передача BUSY при длительном расчете CRC
	bootBuf[0] = CMD_BUSY;		//при запросе состояния от Хоста передадим ему состояния BUSY
	I2C_IT_SetDataSize(&I2c, 1);//
	//Сброс счетчика CRC
	CRC->CR |= CRC_CR_RESET;
	//Считаем CRC
	for(uint32_t i=0; i < size; i+=4)
	{
		CRC->DR = *(uint32_t*)(addr + i);
		//----------
		//Пока считаем CRC пердаем хосту состояние BUSY
		if(I2c.state == I2C_IT_STATE_NAC)//Хост вычитал ответ, сформируем новый
		{
			I2c.state = I2C_IT_STATE_READY;	//Сброс состояния I2C
			bootBuf[0] = CMD_BUSY;			//передаем BUSY
			I2C_IT_SetDataSize(&I2c, 1);	//1 байт на передачу
		}
		//----------
	}
	//Сохраним CRC и передадим ACK
	crc = CRC->DR;
	_sendByte(CMD_ACK);
	//-----------------------
	//Send calculated memory checksum (4 bytes, MSB first) + XOR byte
	bootBuf[0] = (uint8_t)(crc >> 24);
	bootBuf[1] = (uint8_t)(crc >> 16);
	bootBuf[2] = (uint8_t)(crc >> 8);
	bootBuf[3] = (uint8_t)(crc >> 0);
	bootBuf[4] = _getChecksum(bootBuf, 4);
	_sendBuf(5);	//кол-во байтов на передачу
	//-----------------------
	return 1; //команда выполнена.
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void I2C_IT_SlaveRxCpltCallback(I2C_IT_t *i2cIt){

	BOOT_INDICATOR_Toggle();	//мигнем светодиодом
}
//************************************************************
//void I2C_IT_SlaveTxCpltCallback(I2C_IT_t *i2cIt){
//
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
// Function   : BOOT_Init()
// Description: инициализация загрузчика
// Parameters : None
// RetVal     : None
//*****************************************
void BOOT_Init(void){

	//Инициализация I2C Slave для работы по прерываниям.
	I2c.i2c		  = BOOT_I2C;			//используемый порт I2C
	I2c.i2cMode	  = I2C_MODE_SLAVE;		//режим SLAVE
	I2c.gpioRemap = I2C_GPIO_NOREMAP;	//нет ремапа ножек порта I2C
	I2c.i2cSpeed  = BOOT_I2C_SPEED;		//скорость работы порта I2C
	I2c.slaveAddr = BOOT_I2C_ADDR;		//адрес устройства на нине I2C
	I2C_IT_Init(&I2c);
	I2C_IT_SetpBuf(&I2c, bootBuf);		//Прикрепим буфер загрузчика к обработчику прерываний I2C

	//Включаем модуль CRC
	CRC_Init();

	//Период сторожевого таймера 3 сек.
	//IWDG_Init(3000);

	//Индикатор активации загрузчика
	GPIO_InitForOutputPushPull(BOOT_INDICATOR_GPIO, BOOT_INDICATOR_PIN);
	BOOT_INDICATOR_Off();
}
//**********************************************************
// Function   : BOOT_Indication()
// Description: Индикация активации загрузчика
// Parameters : None
// RetVal     : None
//*****************************************
void BOOT_Indication(void){

	//Мигнем три раза - индикация запуска загрузчика.
	for(uint32_t i = 0; i < 3; i++)
	{
		DELAY_milliS(50);
		BOOT_INDICATOR_On();	//LED_PC13_On();
		DELAY_milliS(50);
		BOOT_INDICATOR_Off();	//LED_PC13_Off();
	}
}
//**********************************************************
// Function   : BOOT_DeinitAll()
// Description: Деинициализация всей периферии
// Parameters : None
// RetVal     : None
//*****************************************
void BOOT_DeinitAll(void){

	//Выключение тактирования периферии
	RCC->APB1ENR  = 0x0; 		//Выключаем всю периферию на APB1
	RCC->APB2ENR  = 0x0; 		//Выключаем всю периферию на APB2
	RCC->AHBENR   = 0x0; 		//Выключаем всю периферию на AHB

	//Сброс периферии
	RCC->APB1RSTR = 0xFFFFFFFF;	//Сбрасываем всю периферию на APB1
	RCC->APB1RSTR = 0x0;

	RCC->APB2RSTR = 0xFFFFFFFF;	//Сбрасываем всю периферию на APB2
	RCC->APB2RSTR = 0x0;

	//Reset the RCC clock configuration to the default reset state(for debug purpose).
	RCC->CR	  |= (uint32_t)0x00000001;	/* Set HSION bit */
	RCC->CFGR &= (uint32_t)0xF0FF0000;	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
	RCC->CR   &= (uint32_t)0xFEF6FFFF;	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR   &= (uint32_t)0xFFFBFFFF;	/* Reset HSEBYP bit */
	RCC->CFGR &= (uint32_t)0xFF80FFFF;	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
	RCC->CIR   = 0x009F0000;  			/* Disable all interrupts and clear pending bits  */
}
//**********************************************************
// Function   : AppAvailableCheck()
// Description: Проверка наличия приложения. По стартовому адресу
//				приложения должно лежать значение вершины стека
//				приложения, т.е. значение больше 0x2000 0000.
//				Если это не так, значит	приложение отсутсвует
// Parameters : None
// RetVal     : 1 - приложение есть; 0 - приложения нет.
//*****************************************
uint32_t BOOT_AppAvailableCheck(uint32_t appAddr){

	if((STM32_Flash_ReadWord(appAddr) & 0x2FFC0000) != 0x20000000) return 0;
	return 1;
}
//**********************************************************
// Function   : BOOT_GetAppSize()
// Description: подсчет размера приложения, в байтах
// Parameters : None
// RetVal     : размер приложения в байтах
//*****************************************
uint32_t BOOT_GetAppSize(void){

	//Проверка наличия приложения.
	if(!BOOT_AppAvailableCheck(APP_PROGRAM_START_ADDR)) return 0;	//Приложение нет.

	//Подсчитаем сколько байт занимает приложение.
	uint32_t appSize_Bytes = 0;
	while(STM32_Flash_ReadWord(APP_PROGRAM_START_ADDR + appSize_Bytes) != 0xFFFFFFFF)
	{
		appSize_Bytes += 4; //шагаем по 4 байта
	}

	//в appSize_Bytes лежит размер приложения в байтах.
	//Округлим в большую сторону и сделаем кратным 4м
	//while((appSize_Bytes % 4) != 0) { appSize_Bytes++; }
	//appSize_Bytes = appSize_Bytes % 4; //проверка

	return appSize_Bytes;
}
//**********************************************************
// Function    : BOOT_GetStateI2C
// Description : возвращает состояние шины I2C
// Parameters  : None
// RetVal      : смотреть I2C_IT_State_t
//*****************************************
I2C_IT_State_t BOOT_GetStateI2C(void){

	return I2c.state;
}
//**********************************************************
// Function    : BOOT_GoToApp
// Description : используется для запуска загруженного или
//				 любого другого кода путем безусловного перехода
//				 по указанному адресу.
// Parameters  : appAddr - адрес перехода (кратен 4м)
// RetVal      : None
//*****************************************
void BOOT_GoToApp(volatile uint32_t appAddr){

	//Проверка наличия приложения.
	if(!BOOT_AppAvailableCheck(appAddr)) return;

	//Останавливаем сторожевой таймер
	//IWDG_Stop();
	//WWDG_Stop();

	//Деинициализация всей периферии
	BOOT_DeinitAll();

	//Глобальное отключение прерываний.
	__disable_irq();

	//Переход на приложение
	uint32_t resetHandler = *(__IO uint32_t*)(appAddr + 4);	//Адрес функции Reset_Handler приложения
	void(*app)(void) = (void(*)(void))resetHandler; 		//Указатель на функцию Reset_Handler приложения
	SCB->VTOR = appAddr; 									//Адрес таблицы прерываний
	__set_MSP(*(__IO uint32_t*)appAddr);            		//Устанавливаем указатель стека SP приложения
	app();													//Переход на функцию Reset_Handler приложения
}
//**********************************************************
// Function      BOOTLOADER_SetAppLaunch()
// Description   установить условие запуска приложения
// Parameters    launch - Условие запуска приложения
//				 APP_NO					0xFFFFFFFF	//приложение отсутствует
//				 APP_OK_AND_START 		0xAAAA0001	//CRC в норме, можно запускать
//				 APP_REWRITE_ME			0xAAAA0002	//была команда на переход в загрузчик после ресета
//				 APP_CRC_ERR 			0xAAAA001F	//ошибка CRC
// RetVal        None
//*****************************************
//void BOOT_SetAppLaunch(uint32_t launch){
//
////	STM32_Flash_Unlock();
////	STM32_Flash_WriteWord(APP_LAUNCH_CONDITIONS_ADDR, launch);
////	STM32_Flash_Lock();
//}
//**********************************************************
// Function      BOOTLOADER_GetAppLaunch()
// Description   получение условие запуска приложения
// Parameters    None
// RetVal        None
//*****************************************
//uint32_t BOOT_GetAppLaunch(void){
//
////	return STM32_Flash_ReadWord(APP_LAUNCH_CONDITIONS_ADDR);
//}
//**********************************************************
// Function      BOOTLOADER_CalcAppCrc()
// Description   расчет контрольной суммы записанного приложения
// Parameters	 addr - адрес (кратен 4м) области, с которого начинаем считать CRC
//				 size - размер (кратен 4м) области, для которой считаем CRC
// RetVal        crc области данных
//*****************************************
uint32_t BOOT_CalcCrc(uint32_t *addr, uint32_t size){

	return CRC_CalcCrc(addr, size);
}
//**********************************************************
// Function      BOOTLOADER_ReadAppCrc()
// Description	 чтение из флэша записанной ране контрольной
//				 суммы приложения
// Parameters	 None
// RetVal        crc приложения
//*****************************************
//uint32_t BOOT_ReadAppCrc(void){
//
////	return STM32_Flash_ReadWord(APP_CRC_ADDR);
//}
//**********************************************************
// Function      BOOT_CalcAndWriteAppCrc()
// Description	 расчет и запись во флеш контрольной суммы приложения
// Parameters	 None
// RetVal		 None
//*****************************************
void BOOT_CalcAndWriteAppCrc(void){

//	uint32_t appSize = BOOT_GetAppSize();
//	uint32_t appCrc  = BOOT_CalcCrc((uint32_t*)APP_PROGRAM_START_ADDR ,appSize);

	//запись во флеш контрольной суммы приложения
//	STM32_Flash_Unlock();
//	STM32_Flash_WriteWord(APP_CRC_ADDR, appCrc);
//	STM32_Flash_Lock();
}
//**********************************************************
// Function      BOOTLOADER_ErasePageAppState()
// Description   Очистка страницы памяти (1024байта) со структурой
//				 состояния приложения
// Parameters    None
// RetVal        None
//*****************************************
//void BOOT_ErasePageAppState(){
//
////	STM32_Flash_Unlock();
////	STM32_Flash_ErasePage(APP_STATE_ADDR);
////	STM32_Flash_Lock();
//}
//**********************************************************
// Function		BOOT_Loop()
// Descriptio	Основной цикл работы загрузчика. В нем производится обработка
//				команд.
// Parameters   None
// RetVal       None
//*****************************************
void BOOT_Loop(void){

	//Ждем кадра команды (Command frame) от хоста.
	_waitEndRx();	//Блокирующая ф-ия

	//Принят Кадр команды (Command frame) от хоста.
	//Кадр команды состояит из кода команды и инверсии кода команды.
	//После чего хост ждет ACK или NACK.
	//0-й байт - код команды. 1-й байт - инверсия кода команды
	uint8_t inv = ~bootBuf[0];
	if(inv != bootBuf[1])
	{
		_sendByte(CMD_NACK);	//передаем NACK
		bootBuf[0] = 0;			//Сброс приемного буфера
		bootBuf[1] = 0;
		return;
	}
	//Команда верна! Выполнение принятой команды
	switch(bootBuf[0])
	{
		//-------------------
	 	//Команда Get позволяет узнать версию bootloader и поддерживаемые команды.
		case(CMD_BOOT_Get):
			_cmd_Get();
		break;
		//-------------------
		//Команда используется для получения версии загрузчика.
		case(CMD_BOOT_GetVersion):
			_cmd_GetVersion();
		break;
		//-------------------
		//Команда Get ID используется для получения идентификатора чипа
		case(CMD_BOOT_GetID):
			_cmd_GetID();
		break;
		//-------------------
		//Команда Read Memory используется для чтения данных из любого допустимого адреса
		case(CMD_BOOT_RM):
			_cmd_RM();
		break;
		//-------------------
		//Команда Go используется для запуска загруженного или любого другого
		//кода путем безусловного перехода по указанному адресу
		case(CMD_BOOT_GO):
			_cmd_GO();
		break;
		//-------------------
		//Команда WM (Write Memory) используется для записи данных во Flash.
		case(CMD_BOOT_WM):
			_cmd_WM();
		break;
		//-------------------
		//Стирает от одной до всех страниц или секторов флэш-памяти,
		//используя режим двухбайтовой адресации.
		case(CMD_BOOT_ERASE):
			_cmd_ERASE();
		break;
		//-------------------
		//Команда Readout Protect используется для разрешения защиты от чтения памяти Flash (RDP).
		//case(CMD_BOOT_RP):
		//	_cmd_RP();
		//break;
		//-------------------
		//Команда Readout Unprotect используется для запрета (отмены) защиты от чтения памяти Flash (RDP).
		//Когда загрузчик получит команду Readout Unprotect command, он передаст хосту байт ACK.
		//После передачи байта ACK загрузчик сотрет всю память Flash и деактивирует защиту от чтения
		//памяти Flash. Если операция стирания была успешной, загрузчик деактивирует RDP.
		//case(CMD_BOOT_RUP):
		//	_cmd_RUP();
		//break;
		//-------------------
		//Вычисление CRC заданного диапазона флэш-памяти
		case(CMD_BOOT_NS_GetCheckSum):
			_cmd_NS_GetCheckSum();
		break;
		//----------------------------------------
		//Передаем NACK на незнакомую команда
		default:
			_sendByte(CMD_NACK);
		break;
		//----------------------------------------
	}
	//----------------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************








