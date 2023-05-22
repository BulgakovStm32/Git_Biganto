
#include "gpio_ST.h"

//*******************************************************************************************
//*******************************************************************************************
static volatile uint16_t GpioAState = 0; //
static volatile uint16_t GpioBState = 0; //
static volatile uint16_t GpioCState = 0; //
//*******************************************************************************************
//*******************************************************************************************
static uint32_t _gpio_PortXClockEnable(GPIO_TypeDef *port){

	//Включение тактирования портов.
		 if(port == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(port == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(port == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	else return 0;
		 return 1;
}
//*******************************************************************************************
//*******************************************************************************************
void GPIO_InitForOutputPushPull(GPIO_TypeDef *port, uint32_t pin){

	//Включение тактирования портов.
	if(!_gpio_PortXClockEnable(port)) return;
	//Конфигурация выводы в режим 50MHz output push-pull
	if(pin <= 7)
	{
		pin = pin * 4;
		port->CRL |=  (0X03 << pin);    //MODEy[1:0] - 11: Output mode, max speed 50 MHz
		port->CRL &= ~(0x03 << (pin+2));//CNFy[1:0]  - 00: General purpose output push-pull
	}
	else
	{
		pin = (pin - 8) * 4;
		port->CRH |=  (0X03 << pin);    //MODEy[1:0] - 11: Output mode, max speed 50 MHz
		port->CRH &= ~(0x03 << (pin+2));//CNFy[1:0]  - 00: General purpose output push-pull
	}
}
//**********************************************************
void GPIO_InitForOutputOpenDrain(GPIO_TypeDef *port, uint32_t pin){

	//Включение тактирования портов.
	if(!_gpio_PortXClockEnable(port)) return;
	//Конфигурация выводы в режим 50MHz output open-drain.
	if(pin <= 7)
	{
		pin = pin * 4;
		port->CRL |=  (0X03 << pin) |   //MODEy[1:0] - 11: Output mode, max speed 50 MHz
					  (0X01 << (pin+2));//CNFy[1:0]  - 01: General purpose output Open-drain
		port->CRL &= ~(0x02 << (pin+2));//
	}
	else
	{
		pin = (pin - 8) * 4;
		port->CRH |=  (0x03 << pin) |   //MODEy[1:0] - 11: Output mode, max speed 50 MHz
					  (0x01 << (pin+2));//CNFy[1:0]  - 01: General purpose output Open-drain
		port->CRH &= ~(0x02 << (pin+2));//
	}
}
//**********************************************************
void GPIO_InitForInputPullUp(GPIO_TypeDef *port, uint32_t pin){

	//Включение тактирования портов.
	if(!_gpio_PortXClockEnable(port)) return;
	//Конфигурация выводов: Input with pull-up.
	if(pin <= 7)
	{
		pin = pin * 4;
		port->CRL &= ~(0x03 << pin);//GPIO_CRL_MODEx - 00:Input mode (reset state)
		//CNFy[1:0]: 10 - Input with pull-up / pull-down.
		port->CRL &= ~(0b11 << (pin + 2));
		port->CRL |=  (0b10 << (pin + 2));
	}
	else
	{
		pin = (pin - 8) * 4;
		port->CRH &= ~(0x03 << pin);//GPIO_CRL_MODEx - 00:Input mode (reset state)
		//CNFy[1:0]: 10 - Input with pull-up / pull-down.
		port->CRH &= ~(0b11 << (pin + 2));
		port->CRH |=  (0b10 << (pin + 2));
	}
	port->ODR |= (1 << pin); //pull-up.
}
//**********************************************************
void GPIO_InitForInputPullDown(GPIO_TypeDef *port, uint32_t pin){

	//Включение тактирования портов.
	if(!_gpio_PortXClockEnable(port)) return;
	//Конфигурация выводов: Input with pull-up.
	if(pin <= 7)
	{
		pin = pin * 4;
		port->CRL &= ~(0x03 << pin);//GPIO_CRL_MODEx - 00:Input mode (reset state)
		//CNFy[1:0]: 10 - Input with pull-up / pull-down.
		port->CRL &= ~(0b11 << (pin + 2));
		port->CRL |=  (0b10 << (pin + 2));
	}
	else
	{
		pin = (pin - 8) * 4;
		port->CRH &= ~(0x03 << pin);//GPIO_CRL_MODEx - 00:Input mode (reset state)
		//CNFy[1:0]: 10 - Input with pull-up / pull-down.
		port->CRH &= ~(0b11 << (pin + 2));
		port->CRH |=  (0b10 << (pin + 2));
	}
	port->ODR &= ~(1 << pin); //pull-down.
}
//**********************************************************
void GPIO_InitForFloatingInput(GPIO_TypeDef *port, uint32_t pin){

	//Включение тактирования портов.
	if(!_gpio_PortXClockEnable(port)) return;
	//Конфигурация выводов: Input with pull-up.
	if(pin <= 7)
	{
		pin = pin * 4;
		port->CRL &= ~(0x03 << pin);//GPIO_CRL_MODEx - 00:Input mode (reset state)
		//CNFy[1:0]: 01: Floating input (reset state)
		port->CRL &= ~(0b11 << (pin + 2));
		port->CRL |=  (0b01 << (pin + 2));
	}
	else
	{
		pin = (pin - 8) * 4;
		port->CRH &= ~(0x03 << pin);//GPIO_CRL_MODEx - 00:Input mode (reset state)
		//CNFy[1:0]: 01: Floating input (reset state)
		port->CRH &= ~(0b11 << (pin + 2));
		port->CRH |=  (0b01 << (pin + 2));
	}
}
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
	// LED_PC13 - PC13
	//GPIO_InitForOutputPushPull(LED_PC13_GPIO, LED_PC13_PIN);
	//LED_PC13_Off();
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
uint32_t GPIO_GetPortState(GPIO_TypeDef *port){

	     if(port == GPIOA) return GpioAState;
	else if(port == GPIOB) return GpioBState;
	else if(port == GPIOC) return GpioCState;
	return 0;
}
//**********************************************************
uint32_t GPIO_GetPinState(GPIO_TypeDef *port, uint32_t pin){

		 if(port == GPIOA) return (GpioAState & (1<<pin));
	else if(port == GPIOB) return (GpioBState & (1<<pin));
	else if(port == GPIOC) return (GpioCState & (1<<pin));
	return 0;
}
//*******************************************************************************************
//*******************************************************************************************





