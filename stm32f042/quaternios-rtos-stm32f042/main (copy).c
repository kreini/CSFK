#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stm32f0xx.h>

#include "rtos/FreeRTOS.h"	/* Must come first. */
#include "rtos/task.h"		/* RTOS task related API prototypes. */
#include "rtos/queue.h"		/* RTOS queue related API prototypes. */
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

static volatile int	idlehookcnt;
static volatile int	tickhookcnt;

static volatile int	input_flag;


static QueueHandle_t	queue_usart_tx;
static QueueHandle_t	queue_usart_rx;
//static QueueHandle_t	queue_led_delay;
//static QueueHandle_t	queue_tick_count;
static QueueHandle_t	queue_quat_input;


void vApplicationTickHook( void )
{
 tickhookcnt++;
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
 idlehookcnt++;

 __DSB();
 __WFI();
 __ISB();
}

/*****************************************************************************/

uint32_t	data_lost_counter;

void USART2_Handler(void)
{
	//stm32_gpio_output_toggle(GPIOB,3);
	
 	BaseType_t xTaskWokenByReceive = pdFALSE;
 	char	c;

 	if ( USART2->ISR & USART_ISR_TXE )
  	{	BaseType_t	ret;
		ret=xQueueReceiveFromISR(queue_usart_tx,&c,&xTaskWokenByReceive);
		if ( ret )
			USART2->TDR = (uint16_t)c;
		else
			USART2->CR1 &= ~USART_CR1_TXEIE;
  	}

 	if ( USART2->ISR & USART_ISR_RXNE )
  	{	BaseType_t	ret;

		c=(uint8_t)USART2->RDR;
		ret=xQueueSendFromISR(queue_usart_rx,&c,&xTaskWokenByReceive);
		if ( ret == errQUEUE_FULL )
			data_lost_counter++;

	
  	}

 	if ( xTaskWokenByReceive != pdFALSE )
		taskYIELD();

}

/*****************************************************************************/

ssize_t usart_write_queue(void *cookie, const char *buff, size_t size)
{
 	USART_TypeDef *USART=cookie;
 	ssize_t        ssize;

 	ssize=0;
 	while ( 0<size )
  	{	xQueueSend(queue_usart_tx,buff,portMAX_DELAY);
		USART->CR1 |= USART_CR1_TXEIE;
		buff++;
		size--;
		ssize++;
  	}
 	return(ssize);
}

ssize_t usart_write_native(void *cookie, const char *buff, size_t size)
{
 	USART_TypeDef *USART=cookie;
 	ssize_t        ssize;

 	ssize=0;
 	while ( 0<size )
  	{	
  		while ( ! (USART->ISR & USART_ISR_TXE) );
		USART->TDR= *buff;
		
		buff++;
		size--;
		ssize++;
  	}
 	return(ssize);
}

ssize_t usart_read_queue(void *cookie, char *buff, size_t size)
{
 	ssize_t        ssize;

 	ssize=0;
 	while ( 0<size )
  	{	BaseType_t	ret;

		ret=xQueueReceive(queue_usart_rx,buff,ssize==0?portMAX_DELAY:100);

		buff++;
		size--;
		if ( ret )
			ssize++;
		else
			return(ssize);	
  	}
 	return(ssize);
}

/* not used
ssize_t usart_write(void *cookie, const char *buff, size_t size)
{
 	USART_TypeDef *USART=cookie;
 	ssize_t        ssize;
 
 	ssize=0;
 	while ( 0<size )
	 {	while ( ! (USART->ISR & USART_ISR_TXE) );
		USART->TDR=*buff;
        	buff++;
        	size--;
        	ssize++;
  	 };

 	return(ssize);
}*/

/*****************************************************************************/

void msleep(int t)
{
 vTaskDelay( t / portTICK_RATE_MS );  
}

int frac_dec(int i)
{
   return(i*10000)>>15 ;
}
/*
int mult_frac(int i1, int i2)
{
   return((i1*i2)>>15);
}*/

void print_quat(int i1, int i2, int i3, int i4)
{
   fprintf(stdout,"0.%04i +0.%04i i+0.%04i j+0.%04i k \n",frac_dec(i1),frac_dec(i2),frac_dec(i3),frac_dec(i4));
}

void mult_quat(int q1[4], int q2[4], int (*q3[4]))
{
   *(q3+0) = ((q1[0]*q2[0])>>15) - ((q1[1]*q2[1])>>15) -((q1[2]*q2[2])>>15) - ((q1[3]*q2[3])>>15);
   *(q3+1) = ((q1[0]*q2[1])>>15) + ((q1[1]*q2[0])>>15) +((q1[2]*q2[3])>>15) - ((q1[3]*q2[2])>>15);
   *(q3+2) = ((q1[0]*q2[2])>>15) + ((q1[2]*q2[0])>>15) +((q1[3]*q2[1])>>15) - ((q1[1]*q2[3])>>15);
   *(q3+3) = ((q1[0]*q2[3])>>15) + ((q1[3]*q2[0])>>15) +((q1[1]*q2[2])>>15) - ((q1[2]*q2[1])>>15);
}


/********************************** TASKS *****************************************/
/*
void blink_task(void * parameters) 
{
 //int i;
 //i=0;
 while ( 1 ) 
  {	
	msleep(led_delay);
	stm32_gpio_output_toggle(GPIOB,3);
	//i++;
	//fprintf(stdout,"%d\n",i);
	//fprintf(stdout,"%d\n",xPortGetFreeHeapSize());
  }
}*/
/*
void blink2_task(void * parameters) 
{
 fprintf(stdout,"RTOS heep size: %d words\n",xPortGetFreeHeapSize()>>2);
 int i;
 int led_delay;
 i = 0;
 led_delay = 1000;
 while ( 1 ) 
    {	
    	if(xQueueSend(queue_tick_count,(void*)&i, 10) == pdTRUE)
    	{
	   stm32_gpio_output_toggle(GPIOB,3);
	   i++;
	}
	xQueueReceive(queue_led_delay,(void*)&led_delay, 0);
	msleep(led_delay);
    }
}
*/
/*
void setLedDelay_task(void *parameters) 
{
while ( 1 ) 
  {	int	n;
	//if ( fgets(buff,16,stdin) != NULL )
	if ( scanf("%d", &n) != NULL)
	 {	
	 	fprintf(stdout,"led_delay = %d ms\n",n);
	 	led_delay = n;
	 }
  }
}*/

void prinMessages_task(void * parameters) 
{
 int num_in, quaternio_1[4] ,quaternio_2[4], quaternio_3[4];
 int8_t  counter, second;
 counter = 0;
 second = 0; 
  
 while ( 1 ) 
  {
	 if ( input_flag && xQueueReceive(queue_quat_input,(void*)&num_in,0) == pdTRUE )
	 {	
	 	if (!second)
	 	{
	 	   quaternio_1[counter] = num_in;
	 	   counter++;
	 	}else{
	 	   quaternio_2[counter] = num_in;
	 	   counter++;
	 	}
	 	if (counter == 4)
	 	{
	 	  input_flag = 0;
	 	  second++;
	 	  counter = 0;
	 	}
	 	if (second == 2)
	 	{
	 	   second = 0;
   	 	   fprintf(stdout,"Quaternio_1: ");
	 	   print_quat(quaternio_1[0],quaternio_1[1],quaternio_1[2],quaternio_1[3]);
   	 	   fprintf(stdout,"Quaternio_2: ");
	 	   print_quat(quaternio_2[0],quaternio_2[1],quaternio_2[2],quaternio_2[3]);
	 	   /*
	 	   quaternio[8] = ((quaternio[0]*quaternio[4])>>14) - ((quaternio[1]*quaternio[5])>>14) -((quaternio[2]*quaternio[6])>>14) - ((quaternio[3]*quaternio[7])>>14);
	 	   quaternio[9] = ((quaternio[0]*quaternio[5])>>14) + ((quaternio[1]*quaternio[4])>>14) +((quaternio[2]*quaternio[7])>>14) - ((quaternio[3]*quaternio[2])>>14);
	 	   quaternio[10] = ((quaternio[0]*quaternio[6])>>14) + ((quaternio[2]*quaternio[4])>>14) +((quaternio[3]*quaternio[5])>>14) - ((quaternio[1]*quaternio[7])>>14);
	 	   quaternio[11] = ((quaternio[0]*quaternio[7])>>14) + ((quaternio[3]*quaternio[4])>>14) +((quaternio[1]*quaternio[6])>>14) - ((quaternio[2]*quaternio[5])>>14);*/
	 	   
	 	   mult_quat(quaternio_1, quaternio_2, &quaternio_3);
	 	   
   	 	   fprintf(stdout,"Quaternio_1*Quaternio_2: ");
	 	   print_quat(quaternio_3[0],quaternio_3[1],quaternio_3[2],quaternio_3[3]);
	 	   
	 	}
	 }else{
	 	msleep(50);
	 }
	
	 
  }
}


void readMessages_task(void * parameters) 
{
 int num_in, counter;
 counter = 0;
  
 while ( 1 ) 
  {
  	 stm32_gpio_output_toggle(GPIOB,3);
	 if ( !input_flag && scanf("%0x", &num_in) != NULL)
	 {	
	 	num_in = (num_in & 0x7fff) + ((num_in & 0x8000))*0x1ffff;
	 	//fprintf(stdout,"kaki: %d , %0x\n", num_in, num_in);
	 	if(xQueueSend(queue_quat_input,(void*)&num_in,10) == pdTRUE)
	 	{
	 		fprintf(stdout,"queue_quat_input[%d] to 0.%+05i\n",counter, frac_dec(num_in));
	 		counter++;
	 		if (counter == 4)
	 		{
	 		   counter = 0;
	 		   input_flag = 1;
	 	
	 		}
	 	}
	 	
	 }
	 
  }

}


/*
void uppercase_task(void * parameters) 
{
 char	buff[32];
  

 while ( 1 ) 
  {	int	n;
	if ( fgets(buff,32,stdin) != NULL )
	//if ( n=usart_read_queue(USART2, buff, 8 ) )
	 {	
	 	stm32_gpio_output_toggle(GPIOB,3);
	 	int	i;

		n=strnlen(buff,32);
		for ( i=0; i<n; i++ )
		 {	buff[i]=toupper(buff[i]);
		 }
		fwrite(buff,1,n,stdout);
		fprintf(stdout,"[%d]\n",n);
		
	 }
  }

}*/
 


/* https://www.freertos.org/Hardware-independent-RTOS-example.html */

/********************************** MAIN *****************************************/

int main(void)
{
 cookie_io_functions_t  cookie_uart[2];

 stm32_rcc_init();

 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

 RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
 RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
 RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;
 RAM_VECTORS[16+USART2_IRQn	]=(uint32_t *)USART2_Handler;

 SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;

 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

 stm32_gpio_mode(GPIOB,3,1);
 stm32_gpio_otype(GPIOB,3,0);
 stm32_gpio_ospeed(GPIOB,3,3);
 stm32_gpio_pupd(GPIOB,3,0);

/* stm32_gpio_output_toggle(GPIOB,3); */

 /* USART2 configuration: */
 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
 RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
 stm32_gpio_alternate(GPIOA,2,1);
 stm32_gpio_alternate(GPIOA,15,1);
 USART2->BRR=417;
 USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
 USART2->CR1 |= USART_CR1_UE;
 NVIC_EnableIRQ(USART2_IRQn);
 NVIC_SetPriority(USART2_IRQn,0);
 USART2->CR1 |= USART_CR1_RXNEIE; 


 while ( 0 ) 
  {	__DSB();
	__WFI();
	__ISB();
	stm32_gpio_output_toggle(GPIOB,3);
  }


//upper case in a while true loop - dosent work for me
 while ( 0 )
  {	uint16_t	c;

	while ( ! (USART2->ISR & USART_ISR_RXNE) );
	c=USART2->RDR;
	if ( 'a' <= c && c <= 'z' )
		c -= 'a'-'A';

	stm32_gpio_output_toggle(GPIOB,3);

	while ( ! (USART2->ISR & USART_ISR_TXE) );
	USART2->TDR=c;
  }

 cookie_uart[0].read=usart_read_queue;
 cookie_uart[0].write=NULL;
 cookie_uart[0].seek=NULL;
 cookie_uart[0].close=NULL;

 stdin=fopencookie(USART2,"r",cookie_uart[0]);

 cookie_uart[1].read=NULL;
 cookie_uart[1].write=usart_write_queue;
 cookie_uart[1].seek=NULL;
 cookie_uart[1].close=NULL;

 stdout=fopencookie(USART2,"w",cookie_uart[1]);

 setlinebuf(stdout);
 
 input_flag = 0;
 
 
 queue_usart_tx=xQueueCreate(32,1);
 queue_usart_rx=xQueueCreate(32,1);
 
 //queue_tick_count=xQueueCreate(2,4);
 //queue_led_delay=xQueueCreate(2,4);
 queue_quat_input=xQueueCreate(8,4);
 

/*
 xTaskCreate(blink_task,
	"blink",
	122,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );
*/

/*

 xTaskCreate(blink2_task,
	"blink",
	128,
	NULL,
	tskIDLE_PRIORITY + 2,
        NULL );
*/

 /*
 xTaskCreate(uppercase_task,
	( signed char * ) "upper",
	256,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );
   */

 xTaskCreate(prinMessages_task,
	"printMessage",
	256,
	NULL,
	tskIDLE_PRIORITY + 2,
        NULL );
     
  
 xTaskCreate(readMessages_task,
	"readMessage",
	256,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );
 

 vTaskStartScheduler();

 for( ;; );
}

