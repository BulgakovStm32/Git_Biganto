//*******************************************************************************************
//*******************************************************************************************

#include "adc_ST.h"

//*******************************************************************************************
//*******************************************************************************************
//Настройка АЦП. 
void ADC_Init(void){

	RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN | //Разрешить тактирование АЦП1.
				     RCC_APB2ENR_IOPBEN );//Разрешить тактирование порта PORTB
	//--------------------
	//настраиваем вывод для работы АЦП в режим аналогового входа
	//CNFy[1:0]  = 00 - Analog mode.
	//MODEy[1:0] = 00 - Input mode (reset state).
	GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); //PB0 - аналоговый вход.
	//--------------------	
	//тактовая частота АЦП не должна превышать 14MHz
	RCC->CFGR &= ~RCC_CFGR_ADCPRE;	   //
	RCC->CFGR |=  RCC_CFGR_ADCPRE_DIV8;//предделитель 8 (72МГц/8 = 9МГц).

	//Время выборки канала. При тактировании АЦП 9МГц и при 28,5 циклов преобразование вемя преобразование получится ~3,16мкС.
	//ADC1->SMPR2 |= (0b001 << ADC_SMPR2_SMP8_Pos); // время выборки канала 8 = 28,5 циклов

	//предочистка регистра.
	ADC1->CR1  = 0;
	ADC1->SQR1 = 0;
	ADC1->SQR3 = 0;
	//Настройка работы АЦП.
	ADC1->CR2  =  ADC_CR2_EXTSEL;   //выбрать источником запуска разряд SWSTART
	ADC1->CR2 |= (ADC_CR2_EXTTRIG | //разр. внешний запуск регулярного канала
				  ADC_CR2_ADON);    //включить АЦП
	//Рабочий вариант самокалибровки АЦП. Вычитать значение самокалибровки ненужно, АЦП это делает сам.
	ADC1->CR2 |= ADC_CR2_RSTCAL;         //Сброс калибровки
	while (ADC1->CR2 & ADC_CR2_RSTCAL){};//ждем окончания сброса
	ADC1->CR2 |= ADC_CR2_CAL;            //запуск калибровки
	while (ADC1->CR2 & ADC_CR2_CAL){};   //ждем окончания калибровки
}
//**********************************************************
void ADC_Loop(void){


}
//**********************************************************
//Одно измерение АЦП в мВ.
uint16_t ADC_GetMeas_mV(uint32_t adcCh){
  
	ADC1->SQR3 = adcCh;      	 	  //задать номер канала.
	ADC1->CR2 |= ADC_CR2_SWSTART;     //запуск преобразования в регулярном канале.
	while(!(ADC1->SR & ADC_SR_EOC)){};//дождаться окончания преобразования
	//Вычитать значение самокалибровки ненужно, АЦП это делает сам.
	return (uint16_t)((ADC1->DR * ADC_QUANT_uV + ADC_SCALE/2) / ADC_SCALE);
	//return ADC1->DR;
}
//**********************************************************
uint32_t ADC_GetRegDR(ADC_TypeDef *adc){

    return adc->DR;
}
//**********************************************************
uint32_t ADC_GetRegJDRx(ADC_TypeDef *adc, uint32_t ch){

	if(ch == 1)return adc->JDR1;
	if(ch == 2)return adc->JDR2;
	if(ch == 3)return adc->JDR3;
	if(ch == 4)return adc->JDR4;
	return 0;
}
//*******************************************************************************************
//*******************************************************************************************



