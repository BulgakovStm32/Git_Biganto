/*
 * crc_ST.h
 *
 *  Created on: 9 мар. 2023 г.
 *      Author: belyaev
 */

#ifndef MY_STM32_PERIF_CRC_ST_H_
#define MY_STM32_PERIF_CRC_ST_H_
//*******************************************************************************************
//*******************************************************************************************

#include "stm32f103xb.h"

//*******************************************************************************************
//*******************************************************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void 	 CRC_Init(void);
uint32_t CRC_CalcCrc(uint32_t *data, uint32_t size);

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
#endif /* MY_STM32_PERIF_CRC_ST_H_ */














