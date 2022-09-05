Внедрение прошивки

1) Во-первых, вычисляем начальную константу задержки C0, как указано в уравнении 7 статьи [1]:

	C0 = clock_frequency * sqrt(2 * motor_step_angle / angular_accel) 

где:
angular_accel    = 4(рад/сек^2)    - ускорение; ???? неясно, как его выбирать.
motor_step_angle = 0,0019625 (рад) - угол шага мотора;
clock_frequency  = 1200000 Гц.     - часто тактирования таймера.


void Calculate_C0(void){
	
 /*C0 equation: C0=frequency*sqrt(2*motor_step_angle/angular_accel)*/
	step  = 0;
	temp0 = 2 * motor_step_angle;
	temp0 = temp0 / angular_accel;
	temp0 = fastsqrt(temp0);
	temp0 = temp0 * frequency; //тут С0
}


2) Затем мы вычисляем постоянную времени C1, используя уравнение 12 из [1]. 
Эта постоянная времени будет использоваться для создания задержки первого импульса.

	Cn = (Cn-1)-(2*Cn-1/(4*step+1))

	/*C1 = (C0)-(2*C0/(4*step+1))*/
	step++;
	denom = (step  << 2) + 1;	  	// 4*step+1
	temp1 = (temp0 << 1) / denom; 	// 2*C0/(4*step+1)
	temp0 =  temp0 - temp1;			// (C0)-(2*C0/(4*step+1))

	/* normalization so that delays are obtained in Microseconds */
	temp3 = ceil(temp0 / 12);
	delay_constant = temp3;


	На втором шаге мы настраиваем прерывание по таймеру для расчета временной задержки между импульсами. 
Здесь следует упомянуть, что скорость двигателя увеличивается по мере того, как время задержки между 
импульсами становится короче, а также скорость замедляется по мере увеличения времени задержки между 
импульсами. Прерывание по таймеру настроено, как показано ниже.

	Как только вычислена первая постоянная времени задержки, она загружается в регистр таймера T0MR0. 
Вычисление следующей выдержки времени выполняется, пока таймер ведет счет до значения предыдущей 
выдержки времени.

void SetupTimerInterrupt(void ){

	/* setup the timer interrupt */ 
	T0MCR 		 = 3;                              /* Interrupt and Reset on MR0 */ 
	VICVectAddr4 = (unsigned long ) T0_IRQHandler; /* Set Interrupt Vector */ 
	VICVectCntl4 = 15                   		   /* use it for Timer0 Interrupt */
	VICIntEnable = (1 << 4);
	T0TCR = 1;                          			/* Timer0 Enable */
	T0MR0 = delay_constant;
}

Мы определяем наше прерывание по таймеру, как показано ниже:

__irq void T0_IRQHandler (void){
	
	int i;
	j++;    				   //keep track of number of steps.
	for(i=0; i<120; i++) { ; } //this controls the width of the pulse - ??? Для чего это нужно??

	if(j == 4100)      		   //total number of steps ??? Откуда такое число??
	{
		T0TCR 	   = 0;        /* Timer0 Enable */
		next_state = 0x4;      //Exit state - Для чего это нужно?
	}
	else
	{
		T0MR0 = delay_constant;
	}
	
	T0IR        = 1; /* Clear interrupt flag */
	VICVectAddr = 0; 
	flag         =1; /* this flag signals to main loop to calculate the subsequent time delay */
}

	Общее количество шагов, которые должен пройти двигатель, составляет 4200. - почему именно 4200???
Чтобы выполнить эти шаги и удовлетворить требованиям времени в пути, двигатель проходит три этапа: 
	ускорение, 
	круиз и 
	замедление. 
Двигатель должен ускориться, чтобы достичь желаемой скорости, и как только он достигает такой скорости, 
он остается на этой скорости, прежде чем начнет замедляться. 
Приведенные ниже уравнения используются для определения точек, в которых начинается замедление.



//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
[1] https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
	русский вариант http://avrdoc.narod.ru/index/0-7