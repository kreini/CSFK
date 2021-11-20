/*****************************************/
/**** linked lsits ***** STM32F072  ******/
/*****************************************/
/**********  Gabor Kreinicker  ***********/
/*****************************************/


// includes, defines  és global variables

#define		F_CPU			48000000
#define		_GNU_SOURCE

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <math.h>
#include "delay_basic.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "config.h"

struct node {
	int data;
	struct node* next;
};

int i = 0;


// "delay" function

static inline void msleep (int millisec) {
	while (0 < millisec) {
	_delay_loop(F_CPU/8000L);
	millisec--;
	};
}


// "write" function via UART

ssize_t usart_write(void *cookie, const char *buff, size_t size) {
	USART_TypeDef *USART = cookie;
	ssize_t ssize;
	ssize = 0;
	while (size > 0) {
		while (!(USART->ISR & USART_ISR_TXE));
		USART->TDR = *buff;
		buff++;
		size--;
		ssize++;
	};
	return(ssize);
}


// list printing function

void printlist(struct node* n) {
	printf("[%i]\t", i);
	fflush(stdout);		
	while (n != NULL) {
		printf("%d ", n->data);
		fflush(stdout);
		n = n->next;
	}
	printf("\n");
}

// "main" function

int main(void)	{
	
	cookie_io_functions_t cookie_uart;
	stm32_rcc_init();
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	USART2->BRR=417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;

	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);
	
	
	struct node* first = NULL;	// create structures as NULL pointer
	struct node* second = NULL;
	struct node* third = NULL;

	first = (struct node*)malloc(sizeof(struct node));	// make them dynamically changable
	second = (struct node*)malloc(sizeof(struct node));
	third = (struct node*)malloc(sizeof(struct node));
	 
	first->data = 1;		// assign a value
	first->next = second;		// link with the second list
	second->data = 2;		// assign a value
	second->next = third;		// link with the third list
	third->data = 3;		// assign a value
	third->next = NULL;		// end of linked lists
	// third->next = first;		// cyclic list

	while (1) {
		printlist(first);
		msleep(1000);
	}
	return(0);
}
