/*
 * ds18b20.c
 *
 *  Created on: 20 дек. 2020 г.
 *      Author: Zver
 */
//*******************************************************************************************
//*******************************************************************************************

#include "DS18B20.h"

//*******************************************************************************************
//*******************************************************************************************

//static uint32_t DELAY_WAIT_CONVERT = DELAY_T_CONVERT;

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Функция задержки в микросекундах.
__STATIC_INLINE void _oneWire_usDelay(uint32_t us){

	DELAY_uS(us);

//	us *= 4;  //Эти цифры подобраны эмпирическим путем для Fclk=72MHz.
//	us += 4;
//	while(us--);
}
//**********************************************************
//Процедура инициализации: импульсы сброса и присутствия
static uint32_t _oneWire_ResetAndPresencePulse(GPIO_TypeDef *const port, uint32_t pin){

	uint32_t presence = 0;
	pin = (1 << pin);
	//---------------------
	if(!port) return 1;	//Проверка. Не опрделен порт - значит нет датчика.
						//Выходим с признаком отсутствия устройства на шине.
	//---------------------
	//Формирование импульса сброса
	port->ODR &= ~pin;	  //низкий уровень
	_oneWire_usDelay(500);//задержка минимум 480 мкС
	port->ODR |= pin;	  //отпускаем линию.
	_oneWire_usDelay(60); //задержка 15-60 мкС

	//Проверяем сигнал присутствия.
	_oneWire_usDelay(20);		 //подождем что бы точно попасть на импульс присутствия.
	presence = (port->IDR & pin);//фиксируем уровень
	_oneWire_usDelay(500);		 //подождём, так как могут быть неточности в задержке.
	return presence;
}
//**********************************************************
//Запись бита на шину
static void _oneWire_SendBit(GPIO_TypeDef *const port, uint32_t pin, uint32_t bit){

	pin = (1 << pin);
	//---------------------
	//низкий уровень
	port->ODR &= ~pin;
	if(bit) _oneWire_usDelay(DELAY_WRITE_1);
	else    _oneWire_usDelay(DELAY_WRITE_0);
	//высокий уровень
	port->ODR |= pin;
	if(bit) _oneWire_usDelay(DELAY_WRITE_1_PAUSE);
	else    _oneWire_usDelay(DELAY_WRITE_0_PAUSE);
}
//**********************************************************
//Запись байта на шину
static void _oneWire_SendByte(GPIO_TypeDef *const port, uint32_t pin, uint8_t data){

	for(uint32_t i = 0; i < 8; i++)
	{
		_oneWire_SendBit(port, pin, ((data >> i) & 0x01));
	}
}
//**********************************************************
//Чтение бита.
static uint8_t _oneWire_ReadBit(GPIO_TypeDef *const port, uint32_t pin){

	uint8_t bit = 0;
	pin = (1 << pin);
	//---------------------
	//шину к земле.
	port->ODR &= ~pin;
	_oneWire_usDelay(DELAY_READ_SLOT);
	//отпустили шину.
	port->ODR |= pin;
	_oneWire_usDelay(DELAY_BUS_RELAX);
	//Чтение состояния линии
	if(port->IDR & pin) bit = 1;
	_oneWire_usDelay(DELAY_READ_PAUSE);
	return bit;
}
//**********************************************************
//Чтение нужного количества бит
static uint32_t _oneWire_ReadData(GPIO_TypeDef *const port, uint32_t pin, uint32_t numBit){

	uint32_t data = 0;
	//---------------------
	for(uint32_t i = 0; i < numBit; i++)
	{
		data |= _oneWire_ReadBit(port, pin) << i;
	}
	return data;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//static void _ds18b20_GpioInit(ds18b20_t *sensor){
//
//	if(!sensor->gpioPort) return;	//Проверка. Не опрделен порт - значит нет датчика. Выходим.
//	STM32_GPIO_InitForOutputOpenDrain(sensor->gpioPort, sensor->gpioPin);
//}
////**********************************************************
//static void _ds18b20_SendByte(ds18b20_t *sensor, uint8_t byte){
//
//	_oneWire_SendByte(sensor->gpioPort, sensor->gpioPin, byte);
//}
////**********************************************************
//static void _ds18b20_SetResolution(ds18b20_t *sensor){
//
//	GPIO_TypeDef *port = sensor->gpioPort;
//	uint32_t      pin  = sensor->gpioPin;
//	//---------------------
//	if(_oneWire_ResetAndPresencePulse(port, pin)) return;//Нет сигнала присутсвия на шине - выходим.
//	_ds18b20_SendByte(sensor, SKIP_ROM);
//	_ds18b20_SendByte(sensor, WRITE_SCRATCHPAD);
//	_ds18b20_SendByte(sensor, TH_REGISTER);
//	_ds18b20_SendByte(sensor, TL_REGISTER);
//	_ds18b20_SendByte(sensor, sensor->resolution);
//}
////**********************************************************
//static void _ds18b20_Init(ds18b20_t *sensor){
//
//	_ds18b20_GpioInit(sensor);
//	_ds18b20_SetResolution(sensor);
//	sensor->temperature = DS18B20_NO_SENSE;
//}
////**********************************************************
//static void _ds18b20_ReadScratchPad(ds18b20_t *sensor){
//
//	int16_t  tempReg;
//	uint16_t scale;
//	uint8_t  crcCalc;
//	//---------------------
//	//Чтение блокнота (9 байт) из DS18B20
//	for(uint32_t i = 0; i < 9; i++)
//	{
//		sensor->scratchPad.Blk[i]= _oneWire_ReadData(sensor->gpioPort, sensor->gpioPin, 8);
//	}
//	//Проверка CRC
//	crcCalc = CRC8_TableOneWire(sensor->scratchPad.Blk, 8);
//	if(sensor->scratchPad.Str.crc != crcCalc) return;
//	//Собираем температуру.
//	tempReg = (sensor->scratchPad.Str.temperatureMsb << 8) |
//			   sensor->scratchPad.Str.temperatureLsb;
//	//Смотрим какое разрешение
//	switch(sensor->resolution){
//		case(DS18B20_REOLUTION_9_BIT) : scale = DS18B20_SCALE_9_bit ; break;
//		case(DS18B20_REOLUTION_10_BIT): scale = DS18B20_SCALE_10_bit; break;
//		case(DS18B20_REOLUTION_11_BIT): scale = DS18B20_SCALE_11_bit; break;
//		case(DS18B20_REOLUTION_12_BIT): scale = DS18B20_SCALE_12_bit; break;
//		default: 						scale = DS18B20_SCALE_9_bit;
//	}
//	//Расчет температуры. Один знак после запятой.
//	sensor->temperature = (tempReg * scale + 500) / 1000;
//}
////**********************************************************
//static void _ds18b20_StartConvertTemperature(ds18b20_t *sensor){
//
//	GPIO_TypeDef *port = sensor->gpioPort;
//	uint32_t      pin  = sensor->gpioPin;
//	//---------------------
//	if(_oneWire_ResetAndPresencePulse(port, pin)) return;//Нет сигнала присутсвия на шине - выходим.
//	_ds18b20_SendByte(sensor, SKIP_ROM);
//	_ds18b20_SendByte(sensor, CONVERT_T);
//}
////**********************************************************
//static void _ds18b20_ReadTemperature(ds18b20_t *sensor){
//
//	GPIO_TypeDef *port = sensor->gpioPort;
//	uint32_t      pin  = sensor->gpioPin;
//	//---------------------
//	if(_oneWire_ResetAndPresencePulse(port, pin)) //Нет сигнала присутсвия на шине - выходим.
//	{
//		sensor->temperature = DS18B20_NO_SENSE;
//		return;
//	}
//	_ds18b20_SendByte(sensor, SKIP_ROM);
//	_ds18b20_SendByte(sensor, READ_SCRATCHPAD);
//	_ds18b20_ReadScratchPad(sensor);
//	_ds18b20_StartConvertTemperature(sensor);
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Работа с несколькими устройствами на шине 1-Wire.
//static 	GPIO_TypeDef *owPort;		//
//static 	uint32_t	  owPin;		//
//static 	uint32_t	  owDevicesNum;	//кол-во устройств на шине.

//*******************************************************************************************
//*******************************************************************************************
void ONE_WIRE_Init(GPIO_TypeDef *port, uint32_t pin){

	//Запомним нужный порт и пин
	//owPort = port;
	//owPin  = pin;
	//Порт микроконтроллера, используемый для шины 1-Wire.
	STM32_GPIO_InitForOutputOpenDrain(port, pin);
}
//**********************************************************
//Назначение:	Получение id одного 1-Wire устройства. Посылает команду SEARCH ROM,
//				получает id одного 1-Wire устройства и сохраняет его в массиве.
//Принимает:	wire  - структура для работы с шиной 1-Wire
//				idBuf - указатель на массив хранения идентификатора 1-Wire устройства
//				lastDeviation – номер бита, на котором возникла неоднозначность
//Возвращает: 	номер бита, на котором возникла неоднозначность

uint8_t ONE_WIRE_SearchRom(uint8_t *idBuf, uint8_t lastDeviation){

	uint8_t currentBit 	 = 1;
	uint8_t newDeviation = 0;
	uint8_t bitMask 	 = 1;
	uint8_t bitA;
	uint8_t bitB;
	//---------------------
    //Передача команды SEARCH ROM.
	_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, SEARCH_ROM);

	//Проходим по всем 64-м битам.
	while(currentBit <= 64)
	{
		//Читаем два бита с шины.
		bitA = _oneWire_ReadBit(ONE_WIRE_PORT, ONE_WIRE_PIN); //OWI_ReadBit(pin);
		bitB = _oneWire_ReadBit(ONE_WIRE_PORT, ONE_WIRE_PIN); //OWI_ReadBit(pin);

		//Оба бита = 1 (Ошибка)
		if(bitA && bitB)
		{
			//newDeviation = OWI_ROM_SEARCH_FAILED; //не ясно зачем это тут
			return SEARCH_ERROR;
		}
		//Биты разные
		else if(bitA ^ bitB)
		{
			// Bits A and B are different. All devices have the same bit here.
			// Set the bit in bitPattern to this value.

			//if(bitA) (*idBuf) |=  bitMask;//
			//else	 (*idBuf) &= ~bitMask;	//

			(*idBuf) &= ~bitMask;
			if(bitA) (*idBuf) |= bitMask;
		}
		//Оба бита = 0
		else
		{
			// If this is where a choice was made the last time,
			// a '1' bit is selected this time.
			if(currentBit == lastDeviation)
			{
				(*idBuf) |= bitMask;
			}
			// For the rest of the id, '0' bits are selected when
			// discrepancies occur.
			else if (currentBit > lastDeviation)
			{
				(*idBuf) &= ~bitMask;
				newDeviation = currentBit;
			}
			// If current bit in bit pattern = 0, then this is
			// out new deviation.
			else if (!(*idBuf & bitMask))
			{
				newDeviation = currentBit;
			}
			// IF the bit is already 1, do nothing.
			else
			{

			}
		}

		// Send the selected bit to the bus.
		uint32_t bit = 0;
		if((*idBuf) & bitMask) bit = 1;
		_oneWire_SendBit(ONE_WIRE_PORT, ONE_WIRE_PIN, bit);

		// Increment current bit.
		currentBit++;

		// Adjust bitMask and bitPattern pointer.
		bitMask <<= 1;
		if(!bitMask)
		{
			bitMask = 0x01;
			idBuf++;
		}
	}
	return newDeviation;
}
//**********************************************************
//Назначение:	Получение адресов 1-Wire устройств. Посылает команду SEARCH ROM,
//				получает адреса нескольких 1-Wire устройств, сохраняет их и
//				проверяет контрольную сумму каждого адреса.
//Принимает:	wire - указатель на структура для работы с шиной 1-Wire.
//				devices – указатель на массив структур, в который будет сохраняться id
//				Структура определена в файле OWIHighLevelFunctions.h
//
//				typedef struct
//				{
//					unsigned char id[8];    //!< The 64 bit identifier.
//				} OWI_device;
//
//				numDevices – количество 1-Wire устройств, подключенных к шине
//				numCount – указатель на переменную, в которой сохраняется количество найденных 1-Wire устройств прошедших проверку
//
//Возвращает:	код ошибки.
//				SEARCH_SUCCESSFUL (0x00) - ошибок нет
//				SEARCH_CRC_ERROR  (0x01) - ошибка CRC в одном из адресов

uint8_t ONE_WIRE_SearchDevices(ds18b20_t *devices, uint8_t numDevices, uint8_t *numCount){

	//uint8_t *newID;
	//uint8_t *currentID;
	uint8_t lastDeviation	= 0;		//номер бита, на котором возникла неоднозначность
	uint8_t numFoundDevices = 0;		//кол-во найденых устройств на шине
	uint8_t flag = SEARCH_SUCCESSFUL;	//
	//---------------------
    //сбрасываем id 1-Wire устройств
    for(uint32_t i = 0; i < numDevices; i++)
    {
        for(uint32_t j = 0; j < 8; j++)
        {
            devices[i].id[j] = 0x00;
        }
    }
    //---------------------
    //Вычитываем id устройств, находящихся на шине
//    newID = devices[0].id;
//    currentID = newID;
//	do
//	{
//		//Копируем данные из currentID в newID
//		memcpy(newID, currentID, 8);
//
//		//Чтение id устройства
//		if(_oneWire_ResetAndPresencePulse(owPort, owPin)) return SEARCH_ERROR;
//		lastDeviation = _oneWire_SearchRom(newID, lastDeviation);
//
//		currentID = newID;
//		numFoundDevices++;
//		newID = devices[numFoundDevices].id;
//
//	}while(lastDeviation != OWI_ROM_SEARCH_FINISHED);


	//Мой вариант
	do{
		//Чтение id устройства
		if(_oneWire_ResetAndPresencePulse(ONE_WIRE_PORT, ONE_WIRE_PIN)) return SEARCH_ERROR;
		lastDeviation = ONE_WIRE_SearchRom(devices[numFoundDevices].id, lastDeviation);
		numFoundDevices++;
	}
	while(lastDeviation != OWI_ROM_SEARCH_FINISHED);
	//---------------------
	//Проверка crc вычитаных id
    for(uint32_t i = 0; i < numFoundDevices; i++)
    {
    	//Если какой-либо идентификатор имеет ошибку crc, вернем ошибку.
        uint8_t crc = CRC8_OneWire(devices[i].id, 7);
        if(crc != devices[i].id[7]) flag = SEARCH_CRC_ERROR;
        //crc совпало
        else
        {
        	devices[i].number = i + 1;
        	(*numCount)++;
        }
    }
    return flag;
}
//**********************************************************
//Назначение:	адресация 1-Wire устройства. Посылает команду MATCH ROM и выдает на шину id устройства.
//Принимает:	idArray – указатель на массив в котором храниться адрес 1-Wire устройства
//Возвращает: 	нет

void ONE_WIRE_MatchRom(uint8_t *idArray){

	uint8_t bytesLeft = 8;	//8 байтов id устройства.
	//---------------------
	//Передача команды "MATCH ROM".
	_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, MATCH_ROM);

	//Передача 8-ми байтов id устройства.
	while(bytesLeft > 0)
	{
		_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, *idArray++);
		bytesLeft--;
	}
}
//**********************************************************
uint8_t ONE_WIRE_SendCmd(uint8_t *idArray, DS18B20_Cmd_Enum cmd){

	if(_oneWire_ResetAndPresencePulse(ONE_WIRE_PORT, ONE_WIRE_PIN)) return 0;
	ONE_WIRE_MatchRom(idArray);
	_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, cmd);
	return 1;
}
//**********************************************************
void ONE_WIRE_ReadDeviceData(ds18b20_t *device){

	//---------------------
	//команда чтения блокнота
	if(!ONE_WIRE_SendCmd(device->id, READ_SCRATCHPAD))
	{
		device->temperature = DS18B20_NO_SENSE;
		return;
	}
	//---------------------
	//Чтение блокнота (9 байт) из DS18B20
	for(uint32_t i = 0; i < 9; i++)
	{
		device->scratchPad.Blk[i]= _oneWire_ReadData(ONE_WIRE_PORT, ONE_WIRE_PIN, 8);
	}
	//Проверка CRC
	uint8_t crcCalc = CRC8_TableOneWire(device->scratchPad.Blk, 8);
	if(device->scratchPad.Str.crc != crcCalc) return;
	//Собираем температуру.
	int16_t tempReg = (device->scratchPad.Str.temperatureMsb << 8) |
			   	   	   device->scratchPad.Str.temperatureLsb;
	//Смотрим какое разрешение
	uint16_t scale;
	switch(device->resolution){
		case(DS18B20_REOLUTION_9_BIT) : scale = DS18B20_SCALE_9_bit ; break;
		case(DS18B20_REOLUTION_10_BIT): scale = DS18B20_SCALE_10_bit; break;
		case(DS18B20_REOLUTION_11_BIT): scale = DS18B20_SCALE_11_bit; break;
		case(DS18B20_REOLUTION_12_BIT): scale = DS18B20_SCALE_12_bit; break;
		default: 						scale = DS18B20_SCALE_9_bit;
	}
	//Расчет температуры. Один знак после запятой.
	device->temperature = (tempReg * scale + 500) / 1000;
	//---------------------
	//Команда запуска преобразования температуры
	if(!ONE_WIRE_SendCmd(device->id, CONVERT_T))
	{
		device->temperature = DS18B20_NO_SENSE;
		return;
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
static uint8_t	 numberDevices;	//кол-во устройств на шине.
static ds18b20_t devices[2];	//2 датчика на шине

//*******************************************************************************************
//*******************************************************************************************
void TEMPERATURE_SENSE_Init(void){

	uint8_t crcFlag;		//
	//---------------------
	//Порт микроконтроллера, используемый для шины 1-Wire.
	ONE_WIRE_Init(ONE_WIRE_PORT, ONE_WIRE_PIN);

	//Чтение id и определение кол-ва устройств на шине 1-Wire.
	crcFlag = ONE_WIRE_SearchDevices(devices, 2, &numberDevices);

	//Установим точность измерения и запустим преобразование подключенных датчиков
	while(numberDevices > 0)
	{
		//Установка точности измерения
		TEMPERATURE_SENSE_SetResolution(numberDevices, DS18B20_REOLUTION_12_BIT);

		//Запуск преобразования температуры
		ONE_WIRE_SendCmd(devices[numberDevices-1].id, CONVERT_T);

		numberDevices--;
	}
}
//**********************************************************
//Установка точности измерения
void TEMPERATURE_SENSE_SetResolution(uint8_t sensorNumber, ds18b20_resolution_t resolution){

	//Установка точности измерения
	devices[sensorNumber-1].resolution = resolution;
	ONE_WIRE_SearchRom(devices[sensorNumber-1].id, WRITE_SCRATCHPAD);
	_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, TH_REGISTER);
	_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, TL_REGISTER);
	_oneWire_SendByte(ONE_WIRE_PORT, ONE_WIRE_PIN, resolution);
}
//**********************************************************
void TEMPERATURE_SENSE_ReadData(uint8_t sensorNumber){

	if(sensorNumber > numberDevices) sensorNumber = numberDevices;
	ONE_WIRE_ReadDeviceData(&devices[sensorNumber-1]);
}
//**********************************************************
int16_t TEMPERATURE_SENSE_GetTemp(uint8_t sensorNumber){

	if(sensorNumber > numberDevices) sensorNumber = numberDevices;
	return devices[sensorNumber-1].temperature;
}
//**********************************************************
void TEMPERATURE_SENSE_BuildPack(uint8_t sensorNumber, uint8_t *buf){

	ds18b20_t *sensor;
	//---------------------
	if(sensorNumber > numberDevices) return;
	sensor = &devices[sensorNumber-1];

 	buf[0] = (uint8_t) 0;
 	buf[1] = (uint8_t)(sensor->temperature >> 8);
 	buf[2] = (uint8_t) sensor->temperature;
 	buf[3] = (uint8_t) sensor->number;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************











