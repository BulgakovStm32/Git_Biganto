//*******************************************************************************************
//*******************************************************************************************

#include "sys_ST.h"

//**********************************************************************************************
//**********************************************************************************************
//Настройка тактирования 72МГц. Внешний кварц 8 МГц.
static uint32_t STM32_RCC_Init(void){
	
	uint32_t StartUpCounter = 0;
	uint32_t HSEStatus = 0;
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
		// Enable Prefetch Buffer
		FLASH->ACR |= FLASH_ACR_PRFTBE;

		//Latency
		//These bits represent the ratio of the SYSCLK (system clock)
		//period to the Flash access time.
		// 000 - Zero wait state, if  0 MHz < SYSCLK ≤ 24 MHz
		// 001 - One wait state,  if 24 MHz < SYSCLK ≤ 48 MHz
		// 010 - Two wait states, if 48 MHz < SYSCLK ≤ 72 MHz
		FLASH->ACR &= ~FLASH_ACR_LATENCY;
		FLASH->ACR |= (2 << FLASH_ACR_LATENCY_Pos);// Flash 2 wait state
		//--------------------
		// Настройка тактирования шин AHB, APB1 и APB2.
		RCC->CFGR &= ~(RCC_CFGR_HPRE  |		// AHB prescaler. SYSCLK not divided
					   RCC_CFGR_PPRE1 |		// APB1 = AHB.
					   RCC_CFGR_PPRE2);		// APB2 = AHB. APB2 Fmax = 72MHz.
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;	// APB1 = (AHB / 2). APB1 Fmax = 36MHz.
		//--------------------
		//PLL configuration: = HSE * 9 = 72 MHz */
		//Работает!!!
		RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
		RCC->CFGR |= (RCC_CFGR_PLLSRC | 				//HSE oscillator clock selected as PLL input clock
					//RCC_CFGR_PLLXTPRE_HSE_Div2 |  	//HSE clock divided by 2
					  RCC_CFGR_PLLMULL9);				//PLL input clock*9
		// Enable PLL
		RCC->CR |= RCC_CR_PLLON;
		// Wait till PLL is ready
		while((RCC->CR & RCC_CR_PLLRDY) == 0) {}
		// Select PLL as system clock source
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
		// Wait till PLL is used as system clock source
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08){}
		// Включаем защиту HSE от сбоев - CSS.
		RCC->CR |= RCC_CR_CSSON;
	}
	else
	{
			/* If HSE fails to start-up, the application will have wrong clock
			 configuration. User can add here some code to deal with this error */
	}
	return HSEStatus;
}
//*****************************************************************************
void STM32_Clock_Init(void){
	
  //Reset the RCC clock configuration to the default reset state(for debug purpose).
  RCC->CR 	|= (uint32_t)0x00000001;	/* Set HSION bit */
  RCC->CFGR &= (uint32_t)0xF0FF0000;	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CR 	&= (uint32_t)0xFEF6FFFF;	/* Reset HSEON, CSSON and PLLON bits */
  RCC->CR 	&= (uint32_t)0xFFFBFFFF;	/* Reset HSEBYP bit */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CIR 	 = 0x009F0000;  			/* Disable all interrupts and clear pending bits  */
  //RCC->CFGR2 = 0x00000000;      		/* Reset CFGR2 register */
	
  STM32_RCC_Init();//Настройка тактирования микроконтроллера. Внешний кварц 8 МГц.
}
//**********************************************************************************************
//**********************************************************************************************






