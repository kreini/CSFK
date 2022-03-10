#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stm32f0xx.h>

#include "rtos/FreeRTOS.h"	/* Must come first. */
#include "rtos/task.h"		/* RTOS task related API prototypes. */
#include "rtos/queue.h"	/* RTOS queue related API prototypes. */
#include "rtos/timers.h"	/* Software timer related API prototypes. */
#include "rtos/semphr.h"	/* Semaphore related API prototypes. */

#include "delay_basic.h"

#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"

#define         F_CPU                   48000000

/*****************************************************************************/

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t * RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));


/*****************************************************************************/

static volatile int idlehookcnt;
static volatile int tickhookcnt;

static QueueHandle_t queue_usart_tx;
static QueueHandle_t queue_usart_rx;

void vApplicationTickHook(void) {
	tickhookcnt++;
}

void vApplicationMallocFailedHook( void ) {
	for(;;);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ) {
	(void) pcTaskName;
	(void) xTask;
	for (;;);
}


void vApplicationIdleHook(void) {
	idlehookcnt++;
	__DSB();
	__WFI();
	__ISB();
}

/*****************************************************************************/

uint32_t data_lost_counter;

void USART2_Handler(void) {
	
 	BaseType_t xTaskWokenByReceive = pdFALSE;
 	char c;

 	if (USART2->ISR & USART_ISR_TXE) {
 		BaseType_t ret;
		ret = xQueueReceiveFromISR(queue_usart_tx, &c, &xTaskWokenByReceive);
		if (ret)
			USART2->TDR = (uint16_t)c;
		else
			USART2->CR1 &= ~USART_CR1_TXEIE;
  	}
 	if (USART2->ISR & USART_ISR_RXNE) {
 		BaseType_t ret;
		c = (uint8_t)USART2->RDR;
		ret = xQueueSendFromISR(queue_usart_rx, &c, &xTaskWokenByReceive);
		if (ret == errQUEUE_FULL)
			data_lost_counter++;
  	}
 	if (xTaskWokenByReceive != pdFALSE)
		taskYIELD();

}

/*****************************************************************************/

ssize_t usart_write_queue(void *cookie, const char *buff, size_t size) {

 	USART_TypeDef *USART = cookie;
 	ssize_t ssize;
 	ssize=0;
 	
 	while (0 < size) {
 		xQueueSend(queue_usart_tx,buff,portMAX_DELAY);
		USART->CR1 |= USART_CR1_TXEIE;
		buff++;
		size--;
		ssize++;
  	}
 	return(ssize);
}

ssize_t usart_write_native(void *cookie, const char *buff, size_t size) {

 	USART_TypeDef *USART = cookie;
 	ssize_t ssize;
 	ssize=0;
 	
 	while (0 < size) {	
  		while (!(USART->ISR & USART_ISR_TXE));
		USART->TDR= *buff;
		buff++;
		size--;
		ssize++;
  	}
 	return(ssize);
}

ssize_t usart_read_queue(void *cookie, char *buff, size_t size) {

 	ssize_t ssize;
 	ssize=0;
 	
 	while (0 < size) {
 		BaseType_t ret;
		ret = xQueueReceive(queue_usart_rx, buff, !ssize?portMAX_DELAY:0);
		buff++;
		size--;
		if (ret)
			ssize++;
		else
			return(ssize);	
  	}
 	return(ssize);
}


/*****************************************************************************/

void msleep(int t) {
	vTaskDelay(t / portTICK_RATE_MS);  
}

/********************************** TASKS *****************************************/

void blink_task(void* parameters) {
	int i;
	i=0;

	while (1) {	
		printf("%i\n", i);
		i++;
		msleep(1000);
		stm32_gpio_output_toggle(GPIOA, 5);
	}
}

void thermometer_task(void* parameters) {
	char buff[64];
	uint8_t array[4];
	int ncycle;
	ncycle = 0;

	while (1) {
		int n;
		int16_t temp_1;

		msleep(1000);

		array[0] = 0xBE;
		array[1] = 0xEF;

		stm32_i2c_write_read(I2C1, 0x4B, "\x00", 1, array, 2);
		temp_1 = ((array[0]<<8)|array[1]);

		n = sprintf(buff, "%.2x %.2x T1: %i.%02i [°C]\n[%i]\n",array[0], array[1],
			temp_1>>7, (100*(temp_1&0b1111000)>>3)>>4, ncycle);

		usart_write_queue(USART2,buff,n);
		ncycle++;
	}
}



int main(void) {
	int x;
	cookie_io_functions_t cookie_uart[2];
	stm32_rcc_init();
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
	RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
	RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;
	RAM_VECTORS[16+USART2_IRQn	]=(uint32_t *)USART2_Handler;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	stm32_gpio_mode(GPIOA,5,1);
	stm32_gpio_otype(GPIOA,5,0);
	stm32_gpio_ospeed(GPIOA,5,3);
	stm32_gpio_pupd(GPIOA,5,0);

/* stm32_gpio_output_toggle(GPIOB,3); */

	/* USART2 configuration: */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	USART2->BRR = 417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);
	USART2->CR1 |= USART_CR1_RXNEIE; 

	/* I2C1 configuration: */
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	stm32_gpio_alternate(GPIOB,8,1);
	stm32_gpio_alternate(GPIOB,9,1);
	stm32_i2c_configure(I2C1);
	I2C1->CR1 |= I2C_CR1_PE;


	while (0) {
		__DSB();
		__WFI();
		__ISB();
		stm32_gpio_output_toggle(GPIOA, 5);
	}

	cookie_uart[0].read = usart_read_queue;
	cookie_uart[0].write = NULL;
	cookie_uart[0].seek = NULL;
	cookie_uart[0].close = NULL;

	stdin = fopencookie(USART2, "r", cookie_uart[0]);

	cookie_uart[1].read = NULL;
	cookie_uart[1].write = usart_write_queue;
	cookie_uart[1].seek = NULL;
	cookie_uart[1].close = NULL;

	stdout = fopencookie(USART2, "w" ,cookie_uart[1]);
	setlinebuf(stdout);


	x=0;
	while(0) {
		x++;
		printf("Valami%d\n", x);
		_delay_loop(6000000);
	}

	/*xTaskCreate(blink_task,
		"blink",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 2,
       	 NULL);
	*/
	

	queue_usart_tx=xQueueCreate(32,1);
	queue_usart_rx=xQueueCreate(32,1);
 
	xTaskCreate(thermometer_task,
		( signed char * ) "upper",
		256,
		NULL,
		tskIDLE_PRIORITY + 1,
        	NULL);
      
	     
	/*xTaskCreate(readSerial_task,
		"readSerial",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
        	NULL);
	*/
 
	vTaskStartScheduler();

	for( ;; );
}
