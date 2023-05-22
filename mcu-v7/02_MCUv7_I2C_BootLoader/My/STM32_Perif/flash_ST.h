/*
 * flash_ST.h
 *
 *  Created on: 9 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef FLASH_ST_H
#define FLASH_ST_H
//*******************************************************************************************
//*******************************************************************************************

#include "stm32f103xb.h"

//*******************************************************************************************
//*******************************************************************************************
//Как же определить свободную страницу? Это зависит от размера прошивки.
//Обычно используют последнюю страницу в памяти.
//Её адрес можно вычислить как [начальный адрес] + [номер страницы - 1] * [размер страницы].
//Выберем 16 страницу: 0x0800 0000 + 15 * 1024 = 0x0800 3C00

//#define FLASH_PAGES_NUM 			64		//Кол-во страниц флеш-памяти для микроконтроллера с  64kB FLASH.
#define FLASH_PAGES_NUM				128 	//Кол-во страниц флеш-памяти для микроконтроллера с 128kB FLASH.
#define FLASH_PAGE_SIZE				1024	//Размер страницы флеш-памяти в байтах для STM32F103xx.
#define FLASH_PAGE_ADDR(pageNum)	(uint32_t)(FLASH_BASE + FLASH_PAGE_SIZE * pageNum)

//Адреса страниц флеш-памяти
//#define FLASH_PAGE_127 (FLASH_BASE + (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM - 0)))
//#define FLASH_PAGE_126 (FLASH_BASE + (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM	- 1)))
//
//#define FLASH_PAGE127_END_ADDR 	(FLASH_PAGE_127 + FLASH_PAGE_SIZE)
//#define FLASH_PAGE126_END_ADDR 	(FLASH_PAGE_126 + FLASH_PAGE_SIZE)
//**********************************
#define FLASH_RDO_ENABLE	1
#define FLASH_RDO_DISABLE	0
//**********************************
//Состояния выполнения операция
typedef enum{
	FLASH_BUSY = 1,
	FLASH_ERROR_PG,
	FLASH_ERROR_WRP,
	FLASH_COMPLETE,
	FLASH_TIMEOUT
}FlashStatus_t;
//**********************************
//typedef enum{
//
//}FlashState_t;
//*******************************************************************************************
//*******************************************************************************************
void STM32_Flash_Lock     (void);
void STM32_Flash_Unlock   (void);

void STM32_Flash_ErasePage(uint32_t pageAddress);

void STM32_Flash_WriteWord(uint32_t address, uint32_t data);
void STM32_Flash_WriteHalfWord(uint32_t address, uint16_t data);
void STM32_Flash_WriteBuf(void* Src, void* Dst, uint32_t size);

uint32_t STM32_Flash_ReadWord(uint32_t address);
void 	 STM32_Flash_ReadBufU8(void* Src, void* Dst, uint32_t size);
void 	 STM32_Flash_ReadBufU32(void* Src, void* Dst, uint32_t size);

uint32_t 	  STM32_Flash_GetReadOutProtectionStatus(void);
FlashStatus_t STM32_Flash_ReadOutProtection(uint32_t state);

uint32_t 	  STM32_Flash_GetWriteProtectionOptionByte(void);
FlashStatus_t STM32_Flash_EnableWriteProtection(uint32_t flashPages);
//*******************************************************************************************
//*******************************************************************************************
#endif /* FLASH_ST_H */




