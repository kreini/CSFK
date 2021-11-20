#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>

#include "rtos/FreeRTOS.h"	/* Must come first. */
#include "rtos/task.h"		/* RTOS task related API prototypes. */
#include "rtos/queue.h"	/* RTOS queue related API prototypes. */
#include "rtos/timers.h"	/* Software timer related API prototypes. */
#include "rtos/semphr.h"	/* Semaphore related API prototypes. */

#include "delay_basic.h"

#include "stm32_rcc.h"
#include "stm32_gpio.h"

#define         F_CPU                   48000000

/*****************************************************************************/

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t * RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));

/*****************************************************************************/

void vApplicationTickHook( void )
{
 static uint32_t ulCount = 0;

 ulCount++;
 if( 500UL <= ulCount )
  {     ulCount = 0UL;
  }
}

void vApplicationMallocFailedHook( void )
{
 for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
 (void) pcTaskName;
 (void) xTask;

 for (;;);
}

void vApplicationIdleHook( void )
{
}

/*****************************************************************************/

void msleep(int t)
{
 vTaskDelay( t / portTICK_RATE_MS );  
}

void blink_task(void * parameters) 
{
 while ( 1 ) 
  {	stm32_gpio_output_toggle(GPIOA,5);
	msleep(500);
  }
}

void uppercase_task(void * parameters) 
{
 while ( 1 ) 
  {	if ( USART2->ISR & USART_ISR_RXNE )
	 {	uint8_t	c;
		c=USART2->RDR;
		if ( 'a' <= c && c <= 'z' )	c -= 'a'-'A';
		USART2->TDR=c;
	 }
  }
}

/* https://www.freertos.org/Hardware-independent-RTOS-example.html */

int main(void)
{
 stm32_rcc_init();

 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

 RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
 RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
 RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;

 SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;

 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

 stm32_gpio_mode(GPIOA,5,1);
 stm32_gpio_otype(GPIOA,5,0);
 stm32_gpio_ospeed(GPIOA,5,3);
 stm32_gpio_pupd(GPIOA,5,0);

 /* USART2 configuration: */
 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
 RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
 stm32_gpio_alternate(GPIOA,2,1);
 stm32_gpio_alternate(GPIOA,3,1);
 USART2->BRR=417;
/* USART2->CR1 |= USART_CR1_RXNEIE; */
 USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
 USART2->CR1 |= USART_CR1_UE;

 xTaskCreate(blink_task,
	( signed char * ) "blink",
	configMINIMAL_STACK_SIZE,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );

 xTaskCreate(uppercase_task,
	( signed char * ) "upper",
	configMINIMAL_STACK_SIZE,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );
 
 vTaskStartScheduler();

 for( ;; );
}

