/*****************************************************************************/
/* stm32_gpio.h								     */
/*****************************************************************************/

#ifndef	__STM32_GPIO_H_INCLUDED
#define	__STM32_GPIO_H_INCLUDED	1

/*****************************************************************************/

#define	stm32_gpio_mode(GPIOx,n,m)	\
	do 				\
	 {	GPIOx->MODER = (GPIOx->MODER & (~(3<<(2*(n))))) | ((m&3)<<(2*(n)));	\
	 } while(0)

#define	stm32_gpio_input(GPIOx,n)	stm32_gpio_mode((GPIOx),(n),0)

#define	stm32_gpio_input_value(GPIOx,n)		(GPIOx->IDR & (1<<(n)))
#define	stm32_gpio_get_output_value(GPIOx,n)	(GPIOx->ODR & (1<<(n)))

#define stm32_gpio_af(GPIOx,n,f) 	\
	do				\
	 {	 GPIOx->AFR[((n)/8)&1] = (GPIOx->AFR[((n)/8)&1] & (~(15<<(4*((n)%8))))) | ((f&15)<<(4*((n)%8)));	\
	 } while(0)

#define stm32_gpio_alternate(GPIOx,n,f) 		\
	do						\
	 {	stm32_gpio_mode((GPIOx),(n),2);		\
		stm32_gpio_af((GPIOx),(n),(f));		\
	 } while(0)

#define	stm32_gpio_ospeed(GPIOx,n,m)	\
	do 				\
	 {	GPIOx->OSPEEDR = (GPIOx->OSPEEDR & (~(3<<(2*(n))))) | ((m&3)<<(2*(n)));	\
	 } while(0)

#define	stm32_gpio_output_type(GPIOx,n,m)		\
	do 						\
	 {	if (m)	GPIOx->OTYPER  |=  (1<<(n)); 	\
		else	GPIOx->OTYPER  &= ~(1<<(n)); 	\
	 } while(0)

#define	stm32_gpio_output_value(GPIOx,n,m)		\
	do 						\
	 {	if (m)	GPIOx->ODR |=  (1<<(n));	\
		else	GPIOx->ODR &= ~(1<<(n));	\
	 } while(0)

#define	stm32_gpio_set_output_value(GPIOx,n,m)		\
	do 						\
	 {	if (m)	GPIOx->ODR |=  (1<<(n));	\
		else	GPIOx->ODR &= ~(1<<(n));	\
	 } while(0)

#define stm32_gpio_output_toggle(GPIOx,n)		\
	do						\
	 {	GPIOx->ODR ^= (1<<(n));			\
	 } while(0)

#define	stm32_gpio_set_output_bits(GPIOx,mask,value)		\
	do							\
	 {	 GPIOx->ODR = (GPIOx->ODR & ~(mask)) | (value);	\
	 } while(0)

#define	stm32_gpio_pupd(GPIOx,n,m)	\
	do 				\
	 {	GPIOx->PUPDR = (GPIOx->PUPDR & (~(3<<(2*(n))))) | ((m&3)<<(2*(n)));	\
	 } while(0)

#define	stm32_gpio_otype(GPIOx,n,m)	\
	do 				\
	 {	if ( m )				\
			GPIOx->OTYPER |= 1<<(n);	\
		else					\
			GPIOx->OTYPER &= ~(1<<(n));	\
	 } while(0)

#define	stm32_gpio_ahben(port) 			\
	(	(port)==0?RCC_AHBENR_GPIOAEN:	\
		(port)==1?RCC_AHBENR_GPIOBEN:	\
		(port)==2?RCC_AHBENR_GPIOCEN:	\
		(port)==3?RCC_AHBENR_GPIODEN:	\
		(port)==4?RCC_AHBENR_GPIOEEN:	\
		(port)==5?RCC_AHBENR_GPIOFEN:	\
		0				\
	)

#define	stm32_gpio_gpiox(port) 		\
	(	(port)==0?GPIOA:	\
		(port)==1?GPIOB:	\
		(port)==2?GPIOC:	\
		(port)==3?GPIOD:	\
		(port)==4?GPIOE:	\
		(port)==5?GPIOF:	\
		NULL			\
	)

/*****************************************************************************/

#endif

/*****************************************************************************/
