
#include "gpio_ST.h"

//*******************************************************************************************
//*******************************************************************************************
static volatile uint16_t GpioAState = 0; //
static volatile uint16_t GpioBState = 0; //
static volatile uint16_t GpioCState = 0; //
//*******************************************************************************************
//*******************************************************************************************



//**********************************************************
//Инициализация переферии.
void GPIO_Init(void){
  
	//Включаем тактирование порта A, B, C, D и модуля альтернативных функций.
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN |
				   RCC_APB2ENR_IOPBEN |
				   RCC_APB2ENR_IOPCEN |
				   RCC_APB2ENR_IOPDEN |
				   RCC_APB2ENR_AFIOEN);
	//Отключение JTAG-D от порта PA15, отладка через SWD активна.
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	//--------------------
	//Для отладки. Настройка вывода MCO(PA8) для вывода на нее системной частоты SYSCLK.
//	GPIOA->CRH &= ~GPIO_CRH_CNF8;
//	GPIOA->CRH |= GPIO_CRH_CNF8_1;//PA8 -выход, альтернативный режим push-pull.
//	GPIOA->CRH |= GPIO_CRH_MODE8; //PA8 -выход, тактирование 50МГц.
//
//	RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;//Подключение к выводу PA8 системную частоту.
//	RCC->CFGR |= RCC_CFGR_MCO_HSI;   //Подключение к выводу PA8 частоту HSI.
//	RCC->CFGR |= RCC_CFGR_MCO_HSE;      //Подключение к выводу PA8 частоту HSE.
//	RCC->CFGR |= RCC_CFGR_MCO_PLL;   //Подключение к выводу PA8 частоту PLL/2.
	//--------------------
	//LAMP_PWM - PB1
	//MCU_EN   - PB2
	//LIDAR_EN - PB5
	//SENS_LED - PB6
	GPIOB->CRL &= ~(GPIO_CRL_CNF1  | GPIO_CRL_CNF2  | GPIO_CRL_CNF5  | GPIO_CRL_CNF6); //выход, режим - push-pull.
	GPIOB->CRL |=  (GPIO_CRL_MODE1 | GPIO_CRL_MODE2 | GPIO_CRL_MODE5 | GPIO_CRL_MODE6);//тактирование 50МГц.
	//--------------------
	//BB_PWR_BTN - PA8
	//LED_AC     - PA15
	GPIOA->CRH &= ~(GPIO_CRH_CNF8  | GPIO_CRH_CNF15); //выход, режим - push-pull.
	GPIOA->CRH |=  (GPIO_CRH_MODE8 | GPIO_CRH_MODE15);//тактирование 50МГц.
	//--------------------
	//PWR_BTN_LED - PB9
	//FAN_EN 	  - PB12
	//GPS_EN      - PB13
	GPIOB->CRH &= ~(GPIO_CRH_CNF9  | GPIO_CRH_CNF12  | GPIO_CRH_CNF13); //выход, режим - push-pull.
	GPIOB->CRH |=  (GPIO_CRH_MODE9 | GPIO_CRH_MODE12 | GPIO_CRH_MODE13);//тактирование 50МГц.
	//-----------------------------------------
	//Выводы управления драйвером мотора.
	// /DRV_EN    - PA1
	//  DRV_DIR   - PA4
	// /DRV_RESET - PA7
	GPIOA->CRL &= ~(GPIO_CRL_CNF1  | GPIO_CRL_CNF4  | GPIO_CRL_CNF7); //выход, режим - push-pull.
	GPIOA->CRL |=  (GPIO_CRL_MODE1 | GPIO_CRL_MODE4 | GPIO_CRL_MODE7);//тактирование 50МГц.
	// DRV_MODE2 - PC13
	// DRV_MODE1 - PC14
	// DRV_MODE0 - PC15
	GPIOC->CRH &= ~(GPIO_CRH_CNF13  | GPIO_CRH_CNF14  | GPIO_CRH_CNF15); //выход, режим - push-pull.
	GPIOC->CRH |=  (GPIO_CRH_MODE13 | GPIO_CRH_MODE14 | GPIO_CRH_MODE15);//тактирование 50МГц.
	//-----------------------------------------
  //--------------------
  //PC13 - Led.
//  GPIOC->CRH &= ~GPIO_CRH_CNF13;//выход, режим - push-pull.
//  GPIOC->CRH |= GPIO_CRH_MODE13;//тактирование 50МГц.
//
//  //--------------------
//  //PA6 - Led.
//  //PA7 - Led.
//  GPIOA->CRL &= ~(GPIO_CRL_CNF6  | GPIO_CRL_CNF7); //выход, режим - push-pull.
//  GPIOA->CRL |=  (GPIO_CRL_MODE6 | GPIO_CRL_MODE7); //PA7(LC2_SOST_Red) - тактирование 50МГц.


  //JQ6500
  // K1   - PC9 - выход.
  // K2   - PA8 - выход.
  // BUSY - PC9 - вход. - 
/*	GPIOC->CRH &= ~GPIO_CRH_CNF9; //PC9 - выход, режим - push-pull.
  GPIOC->CRH |=  GPIO_CRH_MODE9;//PC9 - тактирование 50МГц.  

  GPIOA->CRH &= ~GPIO_CRH_CNF8; //PA8 - выход, режим - push-pull.
  GPIOA->CRH |=  GPIO_CRH_MODE8;//PA8 - тактирование 50МГц. 
  //PC9 после сброса настроен как вход.
  JqK1Hight;
  JqK2Hight;
  //--------------------
	//Зуммер - PA11.
	//GPIOA->CRH &= ~GPIO_CRH_CNF11; //PA11 - выход, режим - push-pull.
  //GPIOA->CRH |=  GPIO_CRH_MODE11;//PA11 - тактирование 50МГц.  
  //--------------------
	//Двухцветный светодио LC1_SOST.
	//PB1 - LC1_SOST_Red.
	//PB2 - LC1_SOST_Green.
	GPIOB->CRL &= ~GPIO_CRL_CNF1; //PB1(LC1_SOST_Red) - выход, режим - push-pull.
  GPIOB->CRL |= GPIO_CRL_MODE1; //PB1(LC1_SOST_Red) - тактирование 50МГц. 
	GPIOB->CRL &= ~GPIO_CRL_CNF2; //PB2(LC1_SOST_Green) - выход, режим - push-pull.
  GPIOB->CRL |= GPIO_CRL_MODE2; //PB2(LC1_SOST_Green) - тактирование 50МГц. 	
  //--------------------
	//Двухцветный светодио LC2_SOST.
	//PA7 - LC2_SOST_Red.
	//PB0 - LC2_SOST_Green.
	GPIOA->CRL &= ~GPIO_CRL_CNF7; //PA7(LC2_SOST_Red) - выход, режим - push-pull.
  GPIOA->CRL |= GPIO_CRL_MODE7; //PA7(LC2_SOST_Red) - тактирование 50МГц. 
	GPIOB->CRL &= ~GPIO_CRL_CNF0; //PB0(LC2_SOST_Green) - выход, режим - push-pull.
  GPIOB->CRL |= GPIO_CRL_MODE0; //PB0(LC2_SOST_Green) - тактирование 50МГц. 
	//--------------------
  //Оптореле LC. 
  //PC10 - OptSP2Att.
  //PC11 - OptSP2Line.
  //PC12 - OptSP1Line.
  //PD2  - OptSP1Att.
 	GPIOC->CRH &= ~(GPIO_CRH_CNF10 | 
                  GPIO_CRH_CNF11 |
                  GPIO_CRH_CNF12 );//выход, режим - push-pull.
  GPIOD->CRL &= ~(GPIO_CRL_CNF2  ); 
                   
  GPIOC->CRH |= ( GPIO_CRH_MODE10 |
                  GPIO_CRH_MODE11 |
                  GPIO_CRH_MODE12 );//тактирование 50МГц.
  GPIOD->CRL |= ( GPIO_CRL_MODE2  );
	//--------------------
	 *
	 */
}
//**********************************************************
void GPIO_CheckLoop(void){

 	static uint32_t msCount         = 0;
 	static uint32_t cycle           = 0;
 	static uint32_t GpioAIDRtemp[3] = {0};
 	static uint32_t GpioBIDRtemp[3] = {0};
 	static uint32_t GpioCIDRtemp[3] = {0};
  //-------------------------
  if(++msCount >= GPIO_POLLING_DELAY)
    { 
	  msCount = 0;
      //-------------------------
      if(cycle < 3)
        {
          GpioAIDRtemp[cycle] = GPIOA->IDR;//Считывание выводов.
          GpioBIDRtemp[cycle] = GPIOB->IDR;//Считывание выводов.
          GpioCIDRtemp[cycle] = GPIOC->IDR;//Считывание выводов.
          cycle++;
        } 
      //-------------------------
      else
        {
          cycle = 0;
          //Мажоритарное определение состояния выводов.
          GpioAState = ((GpioAIDRtemp[0] & GpioAIDRtemp[1]) |
                        (GpioAIDRtemp[1] & GpioAIDRtemp[2]) |
                        (GpioAIDRtemp[0] & GpioAIDRtemp[2]));

          GpioBState = ((GpioBIDRtemp[0] & GpioBIDRtemp[1]) |
                        (GpioBIDRtemp[1] & GpioBIDRtemp[2]) |
                        (GpioBIDRtemp[0] & GpioBIDRtemp[2]));

          GpioCState = ((GpioCIDRtemp[0] & GpioCIDRtemp[1]) |
                        (GpioCIDRtemp[1] & GpioCIDRtemp[2]) |
                        (GpioCIDRtemp[0] & GpioCIDRtemp[2]));
        }
      //-------------------------
    }
}
//**********************************************************
uint32_t GPIO_GetState(GPIO_TypeDef *port){

	     if(port == GPIOA) return GpioAState;
	else if(port == GPIOB) return GpioBState;
	else if(port == GPIOC) return GpioCState;
	return 0;
}
//**********************************************************
uint32_t GPIO_GetPinState(GPIO_TypeDef *port, uint32_t pin){

		 if(port == GPIOA) return (GpioAState & pin);
	else if(port == GPIOB) return (GpioBState & pin);
	else if(port == GPIOC) return (GpioCState & pin);
	return 0;
}
//*******************************************************************************************
//*******************************************************************************************





