/***********************************************************/
/**  UART "print" függvény és led villog ****  STM32F072  **/
/***********************************************************/
/*******************  Kreinicker Gábor  ********************/
/***********************************************************/


// define-ok, include-ok és globális változók

#define		BCD_VERSION_CC		0x20
#define		BCD_VERSION_YY		0x14
#define		BCD_VERSION_MM		0x03
#define		BCD_VERSION_DD		0x14
#define		F_CPU			48000000
#define		STR_HOSSZ		255

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

void alvas (double s) {
	int idoall = 727479;	// 1 sec
	s /= 1000;
	int szamol = s * idoall;	
	for (volatile int i = 0; i < szamol; i++);
}


// led függvény

void led(int allapot) {
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	
	if (allapot == 1) {
		GPIOA->ODR |= (1<<5);
	}
	if (allapot == 0) {
		GPIOA->ODR &= ~(1<<5);
	}
}


// "print" függvény UART-on

void kiirat (char* kif) {
	if (USART2->ISR & USART_ISR_RXNE) {
		int kif_hossz = strlen(kif);
		for (int i = 0; i < kif_hossz; i++) {	// kifejezés karakterekre bontása
			char c = kif[i];
			USART2 -> TDR = c;		// karakter kiírása
			alvas(5);
		}
	}
	alvas(5);
}


// "scanf" függvény UART-on

void beolvas (char* kif) {
	int index = 0;
	if (USART2->ISR & USART_ISR_RXNE) {
		uint8_t c;
		while (c != '\n') {
			c = USART2 -> RDR;
			kif[index] = c;
			index++;
		}
		alvas(5);
	}
}


// main

int main(void) {
	stm32_rcc_init();
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);	// tehát: (GPIOA, PA2, HIGH) PA2: USART2_TX
	stm32_gpio_alternate(GPIOA,3,1);	// tehát: (GPIOA, PA3, HIGH) PA3: USART2_RX
	USART2->BRR=278;			// baud ráta beállítása 115200-ra (32MHz)
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;

	while (1) {
		alvas(500);
		kiirat("Szereted a szolot? ");
		char* kifejezes;
		alvas(5);
		beolvas(kifejezes);
		kiirat(kifejezes);
	};
	return(0);
}
