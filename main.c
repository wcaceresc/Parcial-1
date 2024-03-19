
#include <stdint.h>
#include "stm32l053xx.h"


/*#if !defined(_SOFT_FP) && defined(_ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
*/

void timer2();
void timer6();
void timer21();
void print_bcd(uint8_t val);
void TIM2_IRQHandler();
void TIM21_IRQHandler();
void inicializar();

uint32_t delay=0x00;

volatile uint8_t clock_base_counter = 0x00;
volatile uint8_t seconds_unit = 0x00;
volatile uint8_t seconds_decimal = 0x00;
volatile uint8_t minutes_unit = 0x00;
volatile uint8_t minutes_decimal = 0x00;
volatile uint8_t hours_unit = 0x00;
volatile uint8_t hours_decimal = 0x00;

int main(void){
	inicializar();

	//while(1)
//	{
//	}

}
void inicializar(){
	//ES UNA FUNCION DE ARM, POR ESO SE AGREGO LAS LIBRERIAS CMSIS
	__disable_irq(); //DESABILITA TODAS LAS INTERRUPCIONES HASTA QUE SE LE INDIQUE

	//ENABLE HSI 16MHZ
		//HSI ON
	RCC->CR |=(1<<0);
		//HSI16 AS SYSCLK
	RCC->CFGR |=(1<<0);

	//ENABLE GPIOA CLK
	RCC->IOPENR |=(1<<0);
	//ENABLE GPIOB CLK
	RCC->IOPENR |=(1<<1);
	//ENABLE GPIOC CLK
	RCC->IOPENR |=(1<<2);

	//ENABLE SYSCFG CLK
	RCC->APB2ENR |=(1<<0);

   // PUERTOS  SALIDA SEGMENTOS
	    GPIOB->MODER &= ~(1<<1); // Segment A
	 	GPIOB->MODER &= ~(1<<3); // Segment B
	 	GPIOB->MODER &= ~(1<<5); // Segment C
	 	GPIOB->MODER &= ~(1<<7); // Segment D
	 	GPIOB->MODER &= ~(1<<9); // Segment E
	 	GPIOB->MODER &= ~(1<<11); // Segment F
	 	GPIOB->MODER &= ~(1<<13); // Segment G
	 	GPIOB->MODER &= ~(1<<15); // Segment DP

// PUERTOS SALIDA DISPLAY

 	GPIOC->MODER &= ~(1<<9); // Common D0
 	GPIOC->MODER &= ~(1<<11); // Common D1
 	GPIOC->MODER &= ~(1<<13); // Common D2
 	GPIOC->MODER &= ~(1<<15); // Common D3
 	GPIOC->MODER &= ~(1<<17); // Common D4
 	GPIOC->MODER &= ~(1<<19); // Common D5

		//PA5 AS OUTPUT
	GPIOA->MODER &=~(1<<11);
		//PC13 AS INPUT
	GPIOC->MODER &=~(1<<26);
	GPIOC->MODER &=~(1<<27);

	GPIOC->MODER &=~(1<<21);  // pc 10
	GPIOC->MODER &=~(1<<23);  //pc11
	GPIOC->MODER &=~(1<<25);   // pc12

	//EXTI13 CONFIG
	//HABILITA EL MULTIPLEXOR EXTI PUERTO C LINEA 13
	SYSCFG->EXTICR[3] |=(1<<5);
	EXTI->IMR |=(1<<13);		//NO IGNORAR EVENTOS DE LA LINEA EXTI 13
	EXTI->FTSR |=(1<<13);		//HABILITO GENERAR EL EVENTO DE INTERRUPCION MEDIANTE UN FLANCO DE BAJADA

	timer2();
	timer6();
	timer21();



	NVIC_EnableIRQ(EXTI4_15_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM21_IRQn);





	__enable_irq();		// ACTIVAR INTERRPTOR GLOBAL

}
void TIM2_IRQHandler(){

	if((TIM2->SR & 0x00000001)){ //CHECK UIF == 1 a 16MHZ
				TIM2->SR &=~1;			//CLEAR UIF SI NO LA LIMPIO NUNCA SE VA A LIMPIAR
				GPIOA->ODR ^=(1<<5);
				clock_base_counter=clock_base_counter + 1;
				if(clock_base_counter == 10){
					seconds_unit = seconds_unit + 1;
					clock_base_counter = 0;
					GPIOC->ODR ^=(1<<10);    // base del contador que cuando sume 10 secuenciencias del TIM2 sea 1 segundo.

				}
				if(seconds_unit == 10){
					seconds_decimal = seconds_decimal + 1;  // base del decimal segundo al contar 10 segundos me da 1 valor decimal
					seconds_unit = 0;
					GPIOC->ODR ^=(1<<11);   // pc11
					if (GPIOC->IDR ^=(1<<13));
					{
							seconds_unit++;}
				}

				if(seconds_decimal == 6){
					minutes_unit++;                              // base de unidad minutos al contar 6 veces los decimales segundos me suma 1 minuto
					seconds_decimal = 0;
					GPIOC->ODR ^=(1<<12); // pc12
				}
				if(minutes_unit == 10){
					minutes_decimal++;          // base de  miuntos decimal al contar 10 minutos unidad me cuenta 1 minuto decimal
					minutes_unit = 0;
				}
				if(minutes_decimal == 6){
					hours_unit++;                                // base de unidad horas al sumar 6 veces decimales minutos me agrega 1 hora unidad.
					minutes_decimal = 0;
				}
				if(hours_unit == 10){
					hours_decimal++;                // base del decimal hora al sumar 10 unidades hora agrega un valor decimal hora
					hours_unit = 0;
				}
				if(hours_decimal == 2){                // condicion de que el valor decimal hora no puede exceder el valor 2, al llegar reinica la secuencia a 00:00:00.
					hours_decimal = 0;
				}
	}
}

void timer6(){

	//TIMER 6 CONFIG 250MS
		//ENABLE TIMER2
	RCC->APB1ENR |=(1<<4);	//AQUI ES DONDE VIVE EL TIMER6

		//LOAD PRESCALER
	TIM6->PSC = 5000-1;	//ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS

		//LOAD ARR
	TIM6->ARR = 210-1;	//SE RESTA UNO PARA QUE AVANCE LOS CICLOS

		//DIRECTION [0: (DEFAULT) UP COUNTER, 1: DOWN COUNTER]
	//TIM2->CR1 |=(1<<0);
		//COUNT TO 0
	TIM6->CNT = 0;
		//COUNTER ENABLE
	TIM6->CR1 |=(1<<0);
}

void timer21(){

	//TIMER 21 CONFIG 1S
		//ENABLE TIMER21
	RCC->APB2ENR |=(1<<2);	//AQUI ES DONDE VIVE EL TIMER 21

		//LOAD PRESCALER
	TIM21->PSC = 1600-1;

		//LOAD ARR
	TIM21->ARR = 100-1;
		//COUNT TO 0
	TIM21->CNT = 0;
		//COUNTER ENABLE
	TIM21->CR1 |=(1<<0);

}

void timer2(){

	//TIMER 2 CONFIG 500MS
		//ENABLE TIMER2
	RCC->APB1ENR |=(1<<0);	//AQUI ES DONDE VIVE EL TIMER2

		//LOAD PRESCALER
	TIM2->PSC = 1600-1;	//ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS

		//LOAD ARR
	TIM2->ARR = 1000-1;	//SE RESTA UNO PARA QUE AVANCE LOS CICLOS

		//DIRECTION [0: (DEFAULT) UP COUNTER, 1: DOWN COUNTER]
	//TIM2->CR1 |=(1<<0);
		//COUNT TO 0
	TIM2->CNT = 0;
		//COUNTER ENABLE
	TIM2->CR1 |=(1<<0);

}

void TIM21_IRQHandler(){
	if((TIM21->SR & 0x00000001)){
		TIM21->SR &=~1;
		delay=delay+1;
	}
}
void EXTI4_15_IRQHandler(){

	TIM2_IRQHandler();
	GPIOB->BSRR = (0xFF<<16);   // RESET BITS SEGMENTS
	print_bcd(seconds_unit);
	GPIOC->ODR |=(1<<9);
	//delay;
	TIM21_IRQHandler(delay);
	GPIOC->BSRR=(0X3F0<<16);  // RESET BITS DE COMUN DISPLAY

	//GPIOB->BSRR = (0x3DC<<16);
	GPIOB->BSRR = (0xFF<<16);   // RESET BITS SEGMENTS
	print_bcd(seconds_decimal);
	//GPIOA->ODR |=(1<<1);
	GPIOC->ODR |=(1<<8);
	TIM21_IRQHandler(delay);
	//delay;
	//GPIOA->BSRR=(0XF03<<16);
	GPIOC->BSRR=(0X3F0<<16);  // RESET BITS DE COMUN DISPLAY

	//GPIOB->BSRR = (0x3DC<<16);
	GPIOB->BSRR = (0xFF<<16);   // RESET BITS SEGMENTS
	print_bcd(minutes_unit);
	//GPIOA->ODR |=(1<<8);
	GPIOC->ODR |=(1<<7);
	TIM21_IRQHandler(delay);
	//delay;
	//GPIOA->BSRR=(0XF03<<16);
	GPIOC->BSRR=(0X3F0<<16);  // RESET BITS DE COMUN DISPLAY

	//GPIOB->BSRR = (0x3DC<<16);
	GPIOB->BSRR = (0xFF<<16);   // RESET BITS SEGMENTS
	print_bcd(minutes_decimal);
	//GPIOA->ODR |=(1<<10);
	GPIOC->ODR |=(1<<6);
	TIM21_IRQHandler(delay);
	//delay;
	//GPIOA->BSRR=(0XF03<<16);
	GPIOC->BSRR=(0X3F0<<16);  // RESET BITS DE COMUN DISPLAY

	//GPIOB->BSRR = (0x3DC<<16);
	GPIOB->BSRR = (0xFF<<16);   // RESET BITS SEGMENTS
	print_bcd(hours_unit);
	//GPIOA->ODR |=(1<<9);
	GPIOC->ODR |=(1<<5);
	TIM21_IRQHandler(delay);
	//delay;
	//GPIOA->BSRR=(0XF03<<16);
	GPIOC->BSRR=(0X3F0<<16);  // RESET BITS DE COMUN DISPLAY

	//GPIOB->BSRR = (0x3DC<<16);
	GPIOB->BSRR = (0xFF<<16);   // RESET BITS SEGMENTS
	print_bcd(hours_decimal);
	//GPIOA->ODR |=(1<<11);
	GPIOC->ODR |=(1<<4);
	TIM21_IRQHandler(delay);
	//delay;
	//GPIOA->BSRR=(0XF03<<16);
	GPIOC->BSRR=(0X3F0<<16);  // RESET BITS DE COMUN DISPLAY
}

void print_bcd(uint8_t val){

		if(val == 0x00){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
			GPIOB->ODR |=0x01<<3;	//SEGMENTO D
			GPIOB->ODR |=0x01<<4;	//SEGMENTO E
			GPIOB->ODR |=0x01<<5;	//SEGMENTO F
		}else if(val == 0x01){
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
		}else if(val == 0x02){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
			GPIOB->ODR |=0x01<<4;	//SEGMENTO E
			GPIOB->ODR |=0x01<<3;	//SEGMENTO D
		}else if(val == 0x03){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
			GPIOB->ODR |=0x01<<3;	//SEGMENTO D
		}else if(val == 0x04){
			GPIOB->ODR |=0x01<<5;	//SEGMENTO F
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
		}else if(val == 0x05){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<5;	//SEGMENTO F
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
			GPIOB->ODR |=0x01<<3;	//SEGMENTO D
		}else if(val == 0x06){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO F
			GPIOB->ODR |=0x01<<5;	//SEGMENTO F
			GPIOB->ODR |=0x01<<4;	//SEGMENTO E
			GPIOB->ODR |=0x01<<3;	//SEGMENTO D
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
		}else if (val == 0x07){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
		}else if (val == 0x08){
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
			GPIOB->ODR |=0x01<<3;	//SEGMENTO D
			GPIOB->ODR |=0x01<<4;	//SEGMENTO E
			GPIOB->ODR |=0x01<<5;	//SEGMENTO F
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
		}else if(val == 0x09){
			GPIOB->ODR |=0x01<<6;	//SEGMENTO G
			GPIOB->ODR |=0x01<<5;	//SEGMENTO F
			GPIOB->ODR |=0x01<<0;	//SEGMENTO A
			GPIOB->ODR |=0x01<<1;	//SEGMENTO B
			GPIOB->ODR |=0x01<<2;	//SEGMENTO C
		}

  }

























/*

#include <stdint.h>
#include "stm32l053xx.h"



void timer2();
void timer6();
void timer21();
void print_bcd(uint8_t val);
void TIM2_IRQHandler();
void TIM21_IRQHandler();
void inicializar();

uint32_t delay=0x00;

volatile uint8_t clock_base_counter = 0x00;
volatile uint8_t seconds_unit = 0x00;
volatile uint8_t seconds_decimal = 0x00;
volatile uint8_t minutes_unit = 0x00;
volatile uint8_t minutes_decimal = 0x00;
volatile uint8_t hours_unit = 0x00;
volatile uint8_t hours_decimal = 0x00;

int main(void)

    {
	inicializar();

//while(1)
//	{
//	}

}
void inicializar()
    {
	//ES UNA FUNCION DE ARM, POR ESO SE AGREGO LAS LIBRERIAS CMSIS
	__disable_irq(); //DESABILITA TODAS LAS INTERRUPCIONES HASTA QUE SE LE INDIQUE

	//ENABLE HSI 16MHZ
		//HSI ON
	RCC->CR |=(1<<0);
		//HSI16 AS SYSCLK
	RCC->CFGR |=(1<<0);

	//ENABLE GPIOA CLK
	RCC->IOPENR |=(1<<0);
	//ENABLE GPIOB CLK
	RCC->IOPENR |=(1<<1);
	//ENABLE GPIOC CLK
	RCC->IOPENR |=(1<<2);

	//ENABLE SYSCFG CLK
	RCC->APB2ENR |=(1<<0);

	//GPIO CONFIG
	GPIOB->MODER &=~(1<<17);	//PB8 COMO SALIDA CON EL SEGMENTO A
	GPIOB->MODER &=~(1<<5);		//PB2 COMO SALIDA CON EL SEGMENTO B
	GPIOB->MODER &=~(1<<7);		//PB3 COMO SALIDA CON EL SEGMENTO C
	GPIOB->MODER &=~(1<<9);		//PB4 COMO SALIDA CON EL SEGMENTO D
	GPIOB->MODER &=~(1<<13);	//PB6 COMO SALIDA CON EL SEGMENTO E
	GPIOB->MODER &=~(1<<15);	//PB7 COMO SALIDA CON EL SEGMENTO F
	GPIOB->MODER &=~(1<<19);	//PB9 COMO SALIDA CON EL SEGMENTO H
	//GPIOB->MODER &=~(1<<3);		//PB1 COMO SALIDA CON EL DOT POINT

	GPIOA->MODER &=~(1<<1);		//PA0 COMO HABILITADOR DEL COMUN DEL DISPLAY 0
	GPIOA->MODER &=~(1<<3);		//PA1 COMO HABILITADOR DEL COMUN DEL DISPLAY 1
	GPIOA->MODER &=~(1<<17);	//PA8 COMO HABILITADOR DEL COMUN DEL DISPLAY 2
	GPIOA->MODER &=~(1<<21);	//PA10 COMO HABILITADOR DEL COMUN DEL DISPLAY 3
	GPIOA->MODER &=~(1<<19);	//PA9 COMO HABILITADOR DEL COMUN DEL DISPLAY 4
	GPIOA->MODER &=~(1<<23);	//PA11 COMO HABILITADOR DEL COMUN DEL DISPLAY 5

		//PA5 AS OUTPUT
	GPIOA->MODER &=~(1<<11);
		//PC13 AS INPUT
	GPIOC->MODER &=~(1<<26);
	GPIOC->MODER &=~(1<<27);

	GPIOC->MODER &=~(1<<21);
	GPIOC->MODER &=~(1<<23);
	GPIOC->MODER &=~(1<<25);

	//EXTI13 CONFIG
	//HABILITA EL MULTIPLEXOR EXTI PUERTO C LINEA 13
	SYSCFG->EXTICR[3] |=(1<<5);
	EXTI->IMR |=(1<<13);		//NO IGNORAR EVENTOS DE LA LINEA EXTI 13
	EXTI->FTSR |=(1<<13);		//HABILITO GENERAR EL EVENTO DE INTERRUPCION MEDIANTE UN FLANCO DE BAJADA

	timer2();
	timer6();
	timer21();

	NVIC_EnableIRQ(EXTI4_15_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM21_IRQn);

	__enable_irq();		//ENABLE GLOBAL INTERRUPS

}
void TIM2_IRQHandler(){

	if((TIM2->SR & 0x00000001)){ //CHECK UIF == 1
				TIM2->SR &=~1;			//CLEAR UIF SI NO LA LIMPIO NUNCA SE VA A LIMPIAR
				//GPIOA->ODR ^=(1<<5);
				clock_base_counter=clock_base_counter + 1;
				if(clock_base_counter == 10){
					seconds_unit = seconds_unit + 1;
					clock_base_counter = 0;
					GPIOC->ODR ^=(1<<10);

				}
				if(seconds_unit == 10){
					seconds_decimal = seconds_decimal + 1;
					seconds_unit = 0;
					GPIOC->ODR ^=(1<<11);
				}
				if(seconds_decimal == 6){
					minutes_unit++;
					seconds_decimal = 0;
					GPIOC->ODR ^=(1<<12);
				}
				if(minutes_unit == 10){
					minutes_decimal++;
					minutes_unit = 0;
				}
				if(minutes_decimal == 6){
					hours_unit++;
					minutes_decimal = 0;
				}
				if(hours_unit == 10){
					hours_decimal++;
					hours_unit = 0;
				}
				if(hours_decimal == 2){
					hours_decimal = 0;
				}
	}
}

void timer6(){

	//TIMER 6 CONFIG 250MS
		//ENABLE TIMER2
	RCC->APB1ENR |=(1<<4);	//AQUI ES DONDE VIVE EL TIMER2

		//LOAD PRESCALER
	TIM6->PSC = 5000-1;	//ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS

		//LOAD ARR
	TIM6->ARR = 210-1;	//SE RESTA UNO PARA QUE AVANCE LOS CICLOS

		//DIRECTION [0: (DEFAULT) UP COUNTER, 1: DOWN COUNTER]
	//TIM2->CR1 |=(1<<0);
		//COUNT TO 0
	TIM6->CNT = 0;
		//COUNTER ENABLE
	TIM6->CR1 |=(1<<0);
}

void timer21(){

	//TIMER 21 CONFIG 1S
		//ENABLE TIMER21
	RCC->APB2ENR |=(1<<2);	//AQUI ES DONDE VIVE EL TIMER 21

		//LOAD PRESCALER
	TIM21->PSC = 1600-1;

		//LOAD ARR
	TIM21->ARR = 100-1;
		//COUNT TO 0
	TIM21->CNT = 0;
		//COUNTER ENABLE
	TIM21->CR1 |=(1<<0);

}

void timer2(){

	//TIMER 2 CONFIG 500MS
		//ENABLE TIMER2
	RCC->APB1ENR |=(1<<0);	//AQUI ES DONDE VIVE EL TIMER2

		//LOAD PRESCALER
	TIM2->PSC = 1600-1;	//ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS

		//LOAD ARR
	TIM2->ARR = 1000-1;	//SE RESTA UNO PARA QUE AVANCE LOS CICLOS

		//DIRECTION [0: (DEFAULT) UP COUNTER, 1: DOWN COUNTER]
	//TIM2->CR1 |=(1<<0);
		//COUNT TO 0
	TIM2->CNT = 0;
		//COUNTER ENABLE
	TIM2->CR1 |=(1<<0);

}

void TIM21_IRQHandler(){
	if((TIM21->SR & 0x00000001)){
		TIM21->SR &=~1;
		delay=delay+1;
	}
}
void EXTI4_15_IRQHandler(){

	TIM2_IRQHandler();
	GPIOB->BSRR = (0x3DC<<16);
	print_bcd(seconds_unit);
	GPIOA->ODR |=(1<<0);
	//delay;
	TIM21_IRQHandler(delay);
	GPIOA->BSRR=(0XF03<<16);

	GPIOB->BSRR = (0x3DC<<16);
	print_bcd(seconds_decimal);
	GPIOA->ODR |=(1<<1);
	TIM21_IRQHandler(delay);
	//delay;
	GPIOA->BSRR=(0XF03<<16);

	GPIOB->BSRR = (0x3DC<<16);
	print_bcd(minutes_unit);
	GPIOA->ODR |=(1<<8);
	TIM21_IRQHandler(delay);
	//delay;
	GPIOA->BSRR=(0XF03<<16);

	GPIOB->BSRR = (0x3DC<<16);
	print_bcd(minutes_decimal);
	GPIOA->ODR |=(1<<10);
	TIM21_IRQHandler(delay);
	//delay;
	GPIOA->BSRR=(0XF03<<16);

	GPIOB->BSRR = (0x3DC<<16);
	print_bcd(hours_unit);
	GPIOA->ODR |=(1<<9);
	TIM21_IRQHandler(delay);
	//delay;
	GPIOA->BSRR=(0XF03<<16);

	GPIOB->BSRR = (0x3DC<<16);
	print_bcd(hours_decimal);
	GPIOA->ODR |=(1<<11);
	TIM21_IRQHandler(delay);
	//delay;
	GPIOA->BSRR=(0XF03<<16);
}

void print_bcd(uint8_t val)

{

		if(val == 0x00){
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
			GPIOB->ODR |=0x01<<4;	//SEGMENTO D
			GPIOB->ODR |=0x01<<6;	//SEGMENTO E
			GPIOB->ODR |=0x01<<7;	//SEGMENTO F
		}else if(val == 0x01){
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		}else if(val == 0x02){
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
			GPIOB->ODR |=0x01<<6;	//SEGMENTO E
			GPIOB->ODR |=0x01<<4;	//SEGMENTO D
		}else if(val == 0x03){
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
			GPIOB->ODR |=0x01<<4;	//SEGMENTO D
		}else if(val == 0x04){
			GPIOB->ODR |=0x01<<7;	//SEGMENTO F
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		}else if(val == 0x05){
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<7;	//SEGMENTO F
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
			GPIOB->ODR |=0x01<<4;	//SEGMENTO D
		}else if(val == 0x06){
			GPIOB->ODR |=0x01<<7;	//SEGMENTO F
			GPIOB->ODR |=0x01<<6;	//SEGMENTO E
			GPIOB->ODR |=0x01<<4;	//SEGMENTO D
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
		}else if (val == 0x07){
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		}else if (val == 0x08){
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
			GPIOB->ODR |=0x01<<4;	//SEGMENTO D
			GPIOB->ODR |=0x01<<6;	//SEGMENTO E
			GPIOB->ODR |=0x01<<7;	//SEGMENTO F
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
		}else if(val == 0x09){
			GPIOB->ODR |=0x01<<9;	//SEGMENTO G
			GPIOB->ODR |=0x01<<7;	//SEGMENTO F
			GPIOB->ODR |=0x01<<8;	//SEGMENTO A
			GPIOB->ODR |=0x01<<2;	//SEGMENTO B
			GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		}

}
*/
