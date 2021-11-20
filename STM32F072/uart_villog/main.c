/**************************************************/
/**  Villogó led és UART teszt  ****  STM32F072  **/
/**************************************************/
/***************  Kreinicker Gábor  ***************/
/**************************************************/


// include-ok, define-ok és globális változók

#define		BCD_VERSION_CC		0x20
#define		BCD_VERSION_YY		0x14
#define		BCD_VERSION_MM		0x03
#define		BCD_VERSION_DD		0x14
#define		F_CPU			48000000

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include "delay_basic.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "config.h"


// várakozás függvény msec-ben

void alvas(double s) {
	int idoall = 1999568;	// 1 sec
	s /= 1000;
	int szamol = s * idoall;	
	for (volatile int i = 0; i < szamol; i++);
}


// led függvény

void led(int allapot) {
	if (allapot == 1) {
		GPIOA->ODR |=  (1<<5);
	}
	if (allapot == 0) {
		GPIOA->ODR &= ~(1<<5);
	}
}


// main

int main(void) {

	stm32_rcc_init();
	
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1); 
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	USART2->BRR=278;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;

	while (1) {
	 	if (USART2->ISR & USART_ISR_RXNE) {
	  		uint8_t c;
			c=USART2->RDR;
			if ('a' <= c && c <= 'z') {
				c-='a'-'A';				
				USART2->TDR=c;
			}
			else {
				for(int i = 0; i < 4; i++) {
					led(1);	// világít
					alvas(500);
					led(0);	// nem világít
					alvas(500);
				}
			}
		 }
	};

	return(0);
}

/*****************************************************************************/
