/************************************/
/**  Villogó led  ****  STM32F072  **/
/************************************/
/********  Kreinicker Gábor  ********/
/************************************/


// include-ok, define-ok  és globális változók

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stm32f0xx.h>

int ledallapot = 0;


// várakozás függvény msec-ben

void alvas(double s) {
	int idoall = 727479;	// 1 sec
	s /= 1000;
	int szamol = s * idoall;	
	for (volatile int i = 0; i < szamol; i++);
}


// led függvény

int led(int allapot) {
	if (allapot == 1) {
		GPIOA->ODR |=  (1<<5);
	}
	if (allapot == 0) {
		GPIOA->ODR &= ~(1<<5);
	}
	if (allapot == 2) {
		if (ledallapot == 0) {
			GPIOA->ODR &= ~(1<<5);
			ledallapot = 1;
			return(1);
		}
		if (ledallapot == 1) {
			GPIOA->ODR |=  (1<<5);
			ledallapot = 0;
			return(1);
		}
	}
	return(1);
}


// main

void main(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	while ( 1 ) {
		led(2);
		alvas(1000);

	}
}
