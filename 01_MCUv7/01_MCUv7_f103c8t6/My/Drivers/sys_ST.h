
#ifndef _sys_ST_H
#define _sys_ST_H
//-----------------------------------------------------------------------------

//#include "stm32f10x.h"
//#include  "stm32f103xb.h"
#include "main.h"

//-----------------------------------------------------------------------------
//Как же определить свободную страницу? Это зависит от размера прошивки. 
//Обычно используют последнюю страницу в памяти. 
//Её адрес можно вычислить как [начальный адрес] + [номер страницы - 1] * [размер страницы].
//Выберем 16 страницу: 0x0800 0000 + 15 * 0x0000 0400 = 0x0800 3C00

#define FLASH_SIZE			128
#define FLASH_PAGE_SIZE		1024

#define FLASH_PAGE_127 (FLASH_BASE + (FLASH_PAGE_SIZE * (FLASH_SIZE	 - 1)))   
#define FLASH_PAGE_126 (FLASH_BASE + (FLASH_PAGE_SIZE * (FLASH_SIZE	 - 2)))   

#define FLASH_PAGE127_END_ADDR 	(FLASH_PAGE_127 + FLASH_PAGE_SIZE)
#define FLASH_PAGE126_END_ADDR 	(FLASH_PAGE_126 + FLASH_PAGE_SIZE)
//-----------------------------------------------------------------------------
void     Sys_Init(void);

void     Stm32Flash_Lock     (void);
void     Stm32Flash_Unlock   (void);
void     Stm32Flash_ErasePage(uint32_t pageAddress);
void     Stm32Flash_WriteWord(uint32_t word, uint32_t address);
uint32_t Stm32Flash_ReadWord (uint32_t address);
//-----------------------------------------------------------------------------
#endif /*_sys_ST_H*/



