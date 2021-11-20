#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>

#include "stm32_rcc.h"

static void stm32_rcc_pll_init(void)
{
 /* set up PLL parameters: MUL=4 */
 RCC->CFGR  = ((RCC->CFGR)&(~RCC_CFGR_PLLMUL))|RCC_CFGR_PLLMUL4;
 /* use HSE as PLL input clock: */
 RCC->CFGR  = (RCC->CFGR & (~RCC_CFGR_PLLSRC))|RCC_CFGR_PLLSRC_HSE_PREDIV;
 /* enable PLL: */
 RCC->CR |= RCC_CR_PLLON;
 /* wait until PLL is locked: */
 while ( ! (RCC->CR & RCC_CR_PLLRDY) );
}

void stm32_rcc_init(void)
{
 /* set up high-speed external clock (quartz and/or clock input): */
 RCC->CR |= RCC_CR_HSEON;

 /* wait unti HSE is settled: */
 while ( ! ( RCC->CR & RCC_CR_HSERDY ) );

 stm32_rcc_pll_init();

 /* Optiimize FLASH access: */
 FLASH->ACR |= FLASH_ACR_LATENCY;
 
 /* use PLL as main clock: */
 RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
}
