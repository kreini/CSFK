/******************************************/
/**  FreeRTOS test  ********  STM32F072  **/
/******************************************/
/***********  Kreinicker Gábor  ***********/
/******************************************/


// includes, defines, and global variables

#define	F_CPU			48000000
#define	_GNU_SOURCE

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <math.h>

#include "delay_basic.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "config.h"

#include "rtos/FreeRTOS.h"	// this must be the first
#include "rtos/task.h"
#include "rtos/queue.h"
#include "rtos/timers.h"
#include "rtos/semphr.h"

int order_1[104] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 410, 400, 390, 380, 370, 360, 350, 340, 330, 320, 310, 300, 290, 280, 270, 260, 250, 240, 230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0};
int order_2[104] = {340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 410, 400, 390, 380, 370, 360, 350, 340, 330, 320, 310, 300, 290, 280, 270, 260, 250, 240, 230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330};
int order_3[104] = {340, 330, 320, 310, 300, 290, 280, 270, 260, 250, 240, 230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 410, 400, 390, 380, 370, 360, 350};


// handlers and vector tables

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t* RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));


//

void vApplicationTickHook(void) {
	static uint32_t ulCount = 0;
	ulCount++;
	if (500UL <= ulCount)
		ulCount = 0UL;
}

void vApplicationMallocFailedHook(void) {
	for(;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName) {
	(void) pcTaskName;
	(void) xTask;
	for(;;);
}

void vApplicationIdleHook(void){};


//

void msleep(int t) {
	vTaskDelay(t / portTICK_RATE_MS);
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


// pulse signal

void pulse(void) {
	for (int i = 0; i < 104; i++) {
		TIM2->CCR4 = order_3[i];			// set dutycycle for CH4
		TIM2->CCR3 = order_2[i];			// set dutycycle for CH3
		TIM2->CCR2 = order_1[i];			// set dutycycle for CH2
		msleep(20);
	}
}


// PWM initialization and application

void init_PWM(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// GPIOB enable
	stm32_gpio_alternate(GPIOB,11,2);			// GPIOB 11 alternate
	stm32_gpio_alternate(GPIOB,10,2);			// GPIOB 10 alternate
	stm32_gpio_alternate(GPIOB,3,2);			// GPIOB 3 alternate
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			// TIM2 enable
	TIM2->PSC = 48-1;					// set prescaler
	TIM2->ARR = 1000-1;					// set autoreload
	// TIM2->CCR4 = 100;					// set dutycycle
}

void init_TIM2_4(void) {
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S;			// set CC4 channel output
	TIM2->CCER &= ~TIM_CCER_CC4P;				// output -> high
	TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;			// output Compare 4 -> PWM Mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;	// output Compare 4 -> PWM Mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;			// enable preload register
	TIM2->CCER |= TIM_CCER_CC4E;				// CC4 enable
}

void init_TIM2_3(void) {
	TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;
	TIM2->CCER &= ~TIM_CCER_CC3P;
	TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
	TIM2->CCER |= TIM_CCER_CC3E;
}

void init_TIM2_2(void) {
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM2->CCER &= ~TIM_CCER_CC2P;
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM2->CCER |= TIM_CCER_CC2E;
}

void start_CNT(void) {
	TIM2->EGR |= TIM_EGR_UG;				// init registers
	TIM2->BDTR |= TIM_BDTR_MOE;				// enable TIM2 output
	TIM2->CR1 |= TIM_CR1_CEN;				// start counting
}


//

void blink_task(void* parameters) {
	float i = 0;
	while(1) {
		stm32_gpio_output_toggle(GPIOA, 5);
		printf("[%.2f]\tsome tasks result\n", sin(i));
		msleep(200);
		i = i + 0.1;
	}
}

void uppercase_task(void* parameters) {
	while(1) {
		if (USART2->ISR & USART_ISR_RXNE) {
			uint8_t c;
			c = USART2->RDR;
			if ('a' <= c && c <= 'z')
				c -= 'a' - 'A';
			USART2->TDR = c;
		}
	}
}

void pulsePWM_task(void* parameters) {
	while(1) {
		pulse();
	}
}


// main function

int main(void) {
	cookie_io_functions_t cookie_uart;
	stm32_rcc_init();
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	RAM_VECTORS[16 + SVC_IRQn] = (uint32_t*)SVC_Handler;
	RAM_VECTORS[16 + PendSV_IRQn] = (uint32_t*)PendSV_Handler;
	RAM_VECTORS[16 + SysTick_IRQn] = (uint32_t*)SysTick_Handler;
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1 | SYSCFG_CFGR1_MEM_MODE_0;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_mode(GPIOA, 5, 1);
	stm32_gpio_otype(GPIOA, 5, 0);
	stm32_gpio_ospeed(GPIOA, 5, 3);
	stm32_gpio_pupd(GPIOA, 5, 0);
	
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	stm32_gpio_alternate(GPIOA, 2, 1);
	stm32_gpio_alternate(GPIOA, 3, 1);
	USART2->BRR = 417;
	USART2->CR1 |= USART_CR1_RE | USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	
	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);
	
	init_PWM();						// PWM init
	init_TIM2_4();						// TIM2 CH4 init
	init_TIM2_3();						// TIM2 CH3 init
	init_TIM2_2();						// TIM2 CH2 init
	start_CNT();						// start counting
	
	xTaskCreate(blink_task,
		(signed char*)"blink",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		NULL);
		
	xTaskCreate(uppercase_task,
		(signed char*)"upper",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		NULL);
	
	xTaskCreate(pulsePWM_task,
		(signed char*)"pulse",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		NULL);
	
	vTaskStartScheduler();
	for(;;);
}
