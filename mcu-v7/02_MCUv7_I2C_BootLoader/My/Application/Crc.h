/*
 * Crc.h
 *
 *  Created on: 29 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef CRC_H
#define CRC_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************


//*******************************************************************************************
//*******************************************************************************************
uint16_t CRC16_FastCalculate(uint8_t *pBlock, uint32_t len);
uint8_t  CRC8_Calculate(uint8_t *pBlock, uint32_t len);
uint8_t  CRC8_FastCalculate(uint8_t *pBlock, uint32_t len);
uint8_t  CRC8_ForEachByte(uint8_t byte, uint8_t oldCrc);

uint8_t  CRC8_OneWire(uint8_t *pBlock, uint32_t len);
uint8_t  CRC8_TableOneWire(uint8_t *pBlock, uint32_t len);

uint8_t  CRC_Calculate(uint8_t *pBlock, uint32_t len);

//*******************************************************************************************
//*******************************************************************************************
#endif /*CRC_H*/
