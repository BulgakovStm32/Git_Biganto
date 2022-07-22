
#include "sys_ST.h"

//**********************************************************************************************
//**********************************************************************************************
//Настройка тактирования 72МГц. Внешний кварц 8 МГц.
static uint32_t Init_RCC(void){
	
	uint32_t StartUpCounter = 0, HSEStatus = 0;
	//--------------------
	//SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/        
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);//Enable HSE.
	/* Wait till HSE is ready and if Time out is reached exit */
	do{
			HSEStatus = RCC->CR & RCC_CR_HSERDY;
			StartUpCounter++;  
	  }
	while((HSEStatus == 0) && (StartUpCounter != 0xFFFF));
	//--------------------
	if ((RCC->CR & RCC_CR_HSERDY) != 0) HSEStatus = (uint32_t)0x01;
	else                                HSEStatus = (uint32_t)0x00;
	//--------------------
	if(HSEStatus == (uint32_t)0x01)
		{
			/* Enable Prefetch Buffer */
			FLASH->ACR |= FLASH_ACR_PRFTBE;
			/* Flash 2 wait state */
			FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
			FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;  
			//--------------------
			//Настройка тактирования шин AHB, APB1 и APB2.
			RCC->CFGR |= (uint32_t)(RCC_CFGR_HPRE_DIV1  | // AHB prescaler. SYSCLK not divided
									RCC_CFGR_PPRE2_DIV1 | // APB2 = AHB. APB2 Fmax = 72MHz.
									RCC_CFGR_PPRE1_DIV2); // APB1 = (AHB / 2). APB1 Fmax = 36MHz.

			//--------------------
			//PLL configuration: = HSE * 9 = 72 MHz */
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
			RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_Msk        |  // HSE oscillator clock selected as PLL input clock
															//RCC_CFGR_PLLXTPRE_HSE_Div2 |  // HSE clock divided by 2                                
															RCC_CFGR_PLLMULL9);
			/* Enable PLL */
			RCC->CR |= RCC_CR_PLLON;
			/* Wait till PLL is ready */
			while((RCC->CR & RCC_CR_PLLRDY) == 0) {}
			/* Select PLL as system clock source */
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
			/* Wait till PLL is used as system clock source */
			while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)  {}        
			RCC->CR |= RCC_CR_CSSON; //Включаем защиту HSE от сбоев - CSS.
		}
	else
		{
				/* If HSE fails to start-up, the application will have wrong clock 
				 configuration. User can add here some code to deal with this error */
		} 
	return HSEStatus;
}
//*****************************************************************************
void Sys_Init(void){
	
  //Reset the RCC clock configuration to the default reset state(for debug purpose).
  RCC->CR 	|= (uint32_t)0x00000001;	/* Set HSION bit */
  RCC->CFGR &= (uint32_t)0xF0FF0000;	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CR 	&= (uint32_t)0xFEF6FFFF;  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR 	&= (uint32_t)0xFFFBFFFF;  /* Reset HSEBYP bit */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CIR 	 = 0x009F0000;  				  /* Disable all interrupts and clear pending bits  */
  //RCC->CFGR2 = 0x00000000;      			/* Reset CFGR2 register */  
	
  Init_RCC();//Настройка тактирования микроконтроллера. Внешний кварц 8 МГц.
}
//**********************************************************************************************
//**********************************************************************************************
//Блокировка flash.
void Stm32Flash_Lock(void){

  FLASH->CR |= FLASH_CR_LOCK;
}
//*****************************************************************************
//Разблокировка работы с flash.
void Stm32Flash_Unlock(void){

  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}
//*****************************************************************************
//Стирание страницы flash.
//Перед записью необходимо стереть данные по нужным адресам, 
//это особенность флеша.
//pageAddress - любой адрес, принадлежащий стираемой странице
void Stm32Flash_ErasePage(uint32_t pageAddress){

	while(FLASH->SR & FLASH_SR_BSY){};                         //Ждем окончания работы с памятю.
	   if(FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;//Сбрасывается бит EOP записью в него единицы.
	FLASH->CR |= FLASH_CR_PER; //стирания страницы.
	FLASH->AR  = pageAddress;  //адрес стираемой страницы.
	FLASH->CR |= FLASH_CR_STRT;//Запуск выбранной операции
	while (!(FLASH->SR & FLASH_SR_EOP));
	FLASH->SR  =  FLASH_SR_EOP;
	FLASH->CR &= ~FLASH_CR_PER;
}
//*****************************************************************************
void Stm32Flash_WriteWord(uint32_t word, uint32_t address){

	while(FLASH->SR & FLASH_SR_BSY){};                         //Ждем окончания работы с памятю.
	   if(FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;//Сбрасывается бит EOP записью в него единицы.
	FLASH->CR |= FLASH_CR_PG;                                //разрешение зиписи во флеш.
	
	*(volatile unsigned short*)address = (uint16_t)word;
	while(!(FLASH->SR & FLASH_SR_EOP)); 
	FLASH->SR = FLASH_SR_EOP;
		
	*(volatile unsigned short*)(address + 2) = (uint16_t)(word >> 16);
	while(!(FLASH->SR & FLASH_SR_EOP)){}; 
  FLASH->SR = FLASH_SR_EOP;
		
  FLASH->CR &= ~(FLASH_CR_PG);
}
//*****************************************************************************
//Чтения 32-хбитного слова из FLASH.
uint32_t Stm32Flash_ReadWord(uint32_t address){

	return (*(__IO uint32_t*)address);
}
//**********************************************************************************************
//**********************************************************************************************
//Прерываени блока CSS. Возникает при сбое тактирования HSE.
void NMI_Handler( void){

  //RCC->CIR |= RCC_CIR_CSSC;//Очищаем флаг прерывания CSS иначе прерывание будет генерироваться бесконечно.
	//NVIC_SystemReset();
  //LC1SostRedLedOn;
}
//*****************************************************************************
//*****************************************************************************
void HardFault_Handler( void){

	Lcd_SetCursor(1, 1);
	Lcd_Print("HardFault");
	Lcd_Update();
	Lcd_ClearVideoBuffer();
}
//*****************************************************************************
//*****************************************************************************
void MemManage_Handler( void){

//  LC2SostRedLedOn();
}
//*****************************************************************************
//*****************************************************************************
void BusFault_Handler( void){

//  LC2SostGreenLedOn();
}
//*****************************************************************************
//*****************************************************************************
void UsageFault_Handler( void){

  //LC2SostRedLedOn;
}
//*****************************************************************************
//*****************************************************************************
void SVC_Handler( void){

  //LC2SostRedLedOn;
}
//**********************************************************************************************
//**********************************************************************************************








