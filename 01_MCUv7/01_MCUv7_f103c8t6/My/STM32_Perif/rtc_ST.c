
#include "rtc_ST.h"

//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
//function инициализация RTC                                                                
//argument none 
//result   1 - инициализация выполнена; 0 - часы уже были инициализированы 
void RtcInit(void){
  
 //разрешить тактирование модулей управления питанием и управлением резервной областью
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  //разрешить доступ к области резервных данных
  PWR->CR |= PWR_CR_DBP;
  //--------------------
  //если часы выключены - инициализировать их
  if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
    {
      //выполнить сброс области резервных данных
      RCC->BDCR |=  RCC_BDCR_BDRST;
      RCC->BDCR &= ~RCC_BDCR_BDRST;
   
      //выбрать источником тактовых импульсов внешний кварц 32768 и подать тактирование
      RCC->BDCR |=  RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_HSE; //RCC_BDCR_RTCSEL_LSE;
   
      RTC->CRL  |=  RTC_CRL_CNF;
      RTC->PRLL  = 0x7FFF;         //регистр деления на 32768
      RTC->CRL  &=  ~RTC_CRL_CNF;
   
      //установить бит разрешения работы и дождаться установки бита готовности
      RCC->BDCR |= RCC_BDCR_LSEON;
      while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON){}
   
      RTC->CRL &= (uint16_t)~RTC_CRL_RSF;
      while((RTC->CRL & RTC_CRL_RSF) != RTC_CRL_RSF){}
    }
  //--------------------
  RTC->CRL |=  RTC_CRL_CNF;  //разрешить конфигурирование регистров RTC
  RTC->CRH  =  RTC_CRH_SECIE;//разрешить прерывание от секундных импульсов
  //RTC->CRH  =  RTC_CRH_ALRIE;//разрешить прерывание при совпадении счетного и сигнального регистра
  //RTC->CRH  =  RTC_CRH_OWIE; //разрешить прерывание при переполнении счетного регистра
  RTC->CRL &= ~RTC_CRL_CNF;  //выйти из режима конфигурирования 
  NVIC_EnableIRQ (RTC_IRQn); //вызвать функцию, которая разрешит прерывание от модуля RTC    
  //--------------------  
}
//-----------------------------------------------------------------------------
//function  читает счетчик RTC
//result    текущее значение счетного регистра
uint32_t GetCounterRTC(void){
  
//  volatile uint32_t temp;

//  __disable_irq();
//  temp = (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);
//  __enable_irq();

//  return  temp;
    return (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);
}
//-----------------------------------------------------------------------------
//function  записывает новое значение в счетчик RTC
//argument  новое значение счетчика               
void SetCounterRTC(uint32_t value){
  
  RTC->CRL |= RTC_CRL_CNF;    //включить режим конфигурирования
  RTC->CNTH = value>>16;      //записать новое значение счетного регистра
  RTC->CNTL = value;          //
  RTC->CRL &= ~RTC_CRL_CNF;   //выйти из режима конфигурирования
}
//-----------------------------------------------------------------------------
void RTC_IRQHandler(void){
  
  //причина прерывания - переполнение входного делителя (новая секунда)
  if(RTC->CRL & RTC_CRL_SECF)
    {
       RTC->CRL &= ~RTC_CRL_SECF;    //сбросить флаг (обязательно!!!)
       //выполняем какие-то действия
       //LC1SostRedLedToggel;
    }
  //причина прерывания - совпадение счетного и сигнального регистра
  if(RTC->CRL & RTC_CRL_ALRF)
    {
       RTC->CRL &= ~RTC_CRL_ALRF;    //сбросить флаг (обязательно!!!)
       //выполняем какие-то действия
    }
  //причина прерывания - переполнение счетного регистра
  if(RTC->CRL & RTC_CRL_OWF)
    {
       RTC->CRL &= ~RTC_CRL_OWF;     //сбросить флаг (обязательно!!!)
       //выполняем какие-то действия
       //LED_ACT_Toggel();
    }
}
//-----------------------------------------------------------------------------
