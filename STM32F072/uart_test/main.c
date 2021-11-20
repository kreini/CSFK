/*****************************************************************************/
/* Accelerometer - STM32F072 version -- main.c				     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* (c) 2019-2020; Andras Pal <apal@szofi.net>				     */
/*****************************************************************************/

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

static inline void testled_on(void)
{
 stm32_gpio_set_output_value(GPIOA,5,1);
}
static inline void testled_off(void)
{
 stm32_gpio_set_output_value(GPIOA,5,0);
}
static inline void testled_toggle(void)
{
 stm32_gpio_output_toggle(GPIOA,5);
}

void testled_init(void)
{
 /* Enable GPIOA: */
 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
 
 stm32_gpio_mode(GPIOA,5,1);
 stm32_gpio_otype(GPIOA,5,0);
 stm32_gpio_ospeed(GPIOA,5,3);
 stm32_gpio_pupd(GPIOA,5,0);

 testled_off();
}

int system_reboot(void)
{
 NVIC_SystemReset();
 return(0);
}

int main(void)
{
 /* Start default main RCC clocking and setup latency: */
 stm32_rcc_init();

 testled_init();

 /* USART2 configuration: FTDI <-> MCU UART line*/
 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
 RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
 stm32_gpio_alternate(GPIOA,2,1);      /* STM32F072_AF_PA2_USART2 */
 stm32_gpio_alternate(GPIOA,3,1);      /* STM32F072_AF_PA3_USART2 */
 USART2->BRR=278;                      /* baud: 115200 @32MHz  */
 USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
 USART2->CR1 |= USART_CR1_UE;

 

 while ( 1 )
  {	if ( USART2->ISR & USART_ISR_RXNE )
	 {	uint8_t	c;
		c=USART2->RDR;
		if ( 'a' <= c && c <= 'z' )	c-='a'-'A';
		USART2->TDR=c;
		
		testled_toggle();
	 }
  };

 return(0);
}

/*****************************************************************************/
