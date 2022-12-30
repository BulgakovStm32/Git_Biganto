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

	DELAY_microS(us);

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
static void _oneWire_WriteBit(GPIO_TypeDef *const port, uint32_t pin, uint32_t bit){

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
static void _oneWire_WriteByte(GPIO_TypeDef *const port, uint32_t pin, uint32_t data){

	for(uint32_t i = 0; i < 8; i++)
	{
		_oneWire_WriteBit(port, pin, ((data >> i) & 0x00000001));
	}
}
//**********************************************************
//Чтение бита.
static uint32_t _oneWire_ReadBit(GPIO_TypeDef *const port, uint32_t pin){

	uint32_t bit = 0;
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
//**********************************************************
/*! \brief  Sends the SEARCH ROM command and returns 1 id found on the
 *          1-Wire(R) bus.
 *
 *  \param  bitPattern      A pointer to an 8 byte char array where the
 *                          discovered identifier will be placed. When
 *                          searching for several slaves, a copy of the
 *                          last found identifier should be supplied in
 *                          the array, or the search will fail.
 *
 *  \param  lastDeviation   The bit position where the algorithm made a
 *                          choice the last time it was run. This argument
 *                          should be 0 when a search is initiated. Supplying
 *                          the return argument of this function when calling
 *                          repeatedly will go through the complete slave
 *                          search.
 *
 *  \param  pin             A bit-mask of the bus to perform a ROM search on.
 *
 *  \return The last bit position where there was a discrepancy between slave addresses the last time this function was run. Returns OWI_ROM_SEARCH_FAILED if an error was detected (e.g. a device was connected to the bus during the search), or OWI_ROM_SEARCH_FINISHED when there are no more devices to be discovered.
 *
 *  \note   See main.c for an example of how to utilize this function.
 */
//unsigned char OWI_SearchRom(uint8_t *bitPattern, uint8_t lastDeviation, uint8_t pin){
//
//	uint8_t currentBit = 1;
//	uint8_t newDeviation = 0;
//	uint8_t bitMask = 0x01;
//	uint8_t bitA;
//	uint8_t bitB;
//
//	// Send SEARCH ROM command on the bus.
//	OWI_SendByte(OWI_ROM_SEARCH, pin);
//
//	// Walk through all 64 bits.
//	while(currentBit <= 64)
//	{
//		// Read bit from bus twice.
//		bitA = OWI_ReadBit(pin);
//		bitB = OWI_ReadBit(pin);
//
//		if (bitA && bitB)
//		{
//			// Both bits 1 (Error).
//			newDeviation = OWI_ROM_SEARCH_FAILED;
//			return SEARCH_ERROR;
//		}
//		else if (bitA ^ bitB)
//		{
//			// Bits A and B are different. All devices have the same bit here.
//			// Set the bit in bitPattern to this value.
//			if (bitA)
//			{
//				(*bitPattern) |= bitMask;
//			}
//			else
//			{
//				(*bitPattern) &= ~bitMask;
//			}
//		}
//		else // Both bits 0
//		{
//			// If this is where a choice was made the last time,
//			// a '1' bit is selected this time.
//			if (currentBit == lastDeviation)
//			{
//				(*bitPattern) |= bitMask;
//			}
//			// For the rest of the id, '0' bits are selected when
//			// discrepancies occur.
//			else if (currentBit > lastDeviation)
//			{
//				(*bitPattern) &= ~bitMask;
//				newDeviation = currentBit;
//			}
//			// If current bit in bit pattern = 0, then this is
//			// out new deviation.
//			else if ( !(*bitPattern & bitMask))
//			{
//				newDeviation = currentBit;
//			}
//			// IF the bit is already 1, do nothing.
//			else
//			{
//
//			}
//		}
//
//
//		// Send the selected bit to the bus.
//		if ((*bitPattern) & bitMask)
//		{
//			OWI_WriteBit1(pin);
//		}
//		else
//		{
//			OWI_WriteBit0(pin);
//		}
//
//		// Increment current bit.
//		currentBit++;
//
//		// Adjust bitMask and bitPattern pointer.
//		bitMask <<= 1;
//		if (!bitMask)
//		{
//			bitMask = 0x01;
//			bitPattern++;
//		}
//	}
//	return newDeviation;
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
static void _ds18b20_GpioInit(DS18B20_t *sensor){

	if(!sensor->GPIO_PORT) return;	//Проверка. Не опрделен порт - значит нет датчика. Выходим.
	GPIO_InitForOutputOpenDrain(sensor->GPIO_PORT, sensor->GPIO_PIN);
}
//**********************************************************
static void _ds18b20_SetResolution(DS18B20_t *sensor){

	uint32_t      pin  = sensor->GPIO_PIN;
	GPIO_TypeDef *port = sensor->GPIO_PORT;
	//---------------------
	if(_oneWire_ResetAndPresencePulse(port, pin)) return;//Нет сигнала присутсвия на шине - выходим.
	_oneWire_WriteByte(port, pin, SKIP_ROM);
	_oneWire_WriteByte(port, pin, WRITE_SCRATCHPAD);
	_oneWire_WriteByte(port, pin, TH_REGISTER);
	_oneWire_WriteByte(port, pin, TL_REGISTER);
	_oneWire_WriteByte(port, pin, sensor->Resolution);
}
//**********************************************************
static void _ds18b20_Init(DS18B20_t *sensor){

	_ds18b20_GpioInit(sensor);
	_ds18b20_SetResolution(sensor);
	sensor->Temperature = 0xFFFFFFFF;
}
//**********************************************************
static void _ds18b20_ReadScratchPad(DS18B20_t *sensor){

	int16_t temperature;
	uint8_t crcCalc = 0;
	//---------------------
	//Чтение блокнота (9 байт) из DS18B20
	for(uint32_t i = 0; i < 9; i++)
	{
		sensor->ScratchPad.Blk[i]= _oneWire_ReadData(sensor->GPIO_PORT, sensor->GPIO_PIN, 8);
	}
	//Проверка CRC
	crcCalc = CRC8_TableOneWire(sensor->ScratchPad.Blk, 8);
	if(sensor->ScratchPad.Str.Crc != crcCalc) return;
	//Собираем температуру.
	temperature = (sensor->ScratchPad.Str.Temperature_MSB << 8) |
				   sensor->ScratchPad.Str.Temperature_LSB;
	//Знак температуры.
	if(temperature < 0)
	{
		sensor->TemperatureSign = DS18B20_SIGN_NEGATIVE;
		temperature  = (temperature ^ 0xFFFF) + 1;
		temperature &= 0x0FFF;//Маска для выделения 12 бит.???Возможно не нужна.
	}
	else sensor->TemperatureSign = DS18B20_SIGN_POSITIVE;
	//Расчет температуры
	sensor->Temperature = (uint32_t)(((temperature * 625) + 500) / 1000);
}
//**********************************************************
static void _ds18b20_StartConvertTemperature(DS18B20_t *sensor){

	uint32_t      pin  = sensor->GPIO_PIN;
	GPIO_TypeDef *port = sensor->GPIO_PORT;
	//---------------------
	if(_oneWire_ResetAndPresencePulse(port, pin)) return;//Нет сигнала присутсвия на шине - выходим.
	_oneWire_WriteByte(port, pin, SKIP_ROM);
	_oneWire_WriteByte(port, pin, CONVERT_T);
}
//**********************************************************
static void _ds18b20_ReadTemperature(DS18B20_t *sensor){

	uint32_t      pin  = sensor->GPIO_PIN;
	GPIO_TypeDef *port = sensor->GPIO_PORT;
	//---------------------
	if(_oneWire_ResetAndPresencePulse(port, pin)) //Нет сигнала присутсвия на шине - выходим.
	{
		sensor->Temperature = 0xFFFFFFFF;
		return;
	}
	_oneWire_WriteByte(port, pin, SKIP_ROM);
	_oneWire_WriteByte(port, pin, READ_SCRATCHPAD);
	_ds18b20_ReadScratchPad(sensor);
	_ds18b20_StartConvertTemperature(sensor);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

static DS18B20_t TemperatureSensor_1;
static DS18B20_t TemperatureSensor_2;

//*******************************************************************************************
//*******************************************************************************************
void TEMPERATURE_SENSE_Init(void){

	TemperatureSensor_1.SensorNumber = 1;
	TemperatureSensor_1.GPIO_PORT    = GPIOB;
	TemperatureSensor_1.GPIO_PIN     = 14;
	TemperatureSensor_1.Resolution   = DS18B20_Resolution_12_bit;
	_ds18b20_Init(&TemperatureSensor_1);
	_ds18b20_StartConvertTemperature(&TemperatureSensor_1);

	TemperatureSensor_2.SensorNumber = 2;
	TemperatureSensor_2.GPIO_PORT    = GPIOB;
	TemperatureSensor_2.GPIO_PIN     = 15;
	TemperatureSensor_2.Resolution   = DS18B20_Resolution_12_bit;
	_ds18b20_Init(&TemperatureSensor_2);
	_ds18b20_StartConvertTemperature(&TemperatureSensor_2);
}
//**********************************************************
void TEMPERATURE_SENSE1_ReadTemperature(void){

	_ds18b20_ReadTemperature(&TemperatureSensor_1);
}
//**********************************************************
void TEMPERATURE_SENSE2_ReadTemperature(void){

	_ds18b20_ReadTemperature(&TemperatureSensor_2);
}
//**********************************************************
DS18B20_t* TEMPERATURE_SENSE_GetSens(uint8_t numSensor){

		 if(numSensor == 1) return &TemperatureSensor_1;
	else if(numSensor == 2) return &TemperatureSensor_2;
	else return 0;
}
//**********************************************************
void TEMPERATURE_SENSE_BuildPack(uint8_t numSensor, uint8_t *buf){

	DS18B20_t *sensor;
	//---------------------
		 if(numSensor == 1) sensor = &TemperatureSensor_1;
	else if(numSensor == 2) sensor = &TemperatureSensor_2;
	else return;

	buf[0] = (uint8_t) sensor->TemperatureSign;
	buf[1] = (uint8_t)(sensor->Temperature >> 8);
	buf[2] = (uint8_t) sensor->Temperature;
	buf[3] = (uint8_t) sensor->SensorNumber; // sensorsNum
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************



















