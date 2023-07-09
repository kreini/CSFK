/*****************************************************************************/
/* delay_basic.h							     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* basic delay rotines for ARM (Cortex-M) architectures...		     */
/*****************************************************************************/

#ifndef	__ARM_UTIL_DELAY_BASIC_H_INCLUDED
#define	__ARM_UTIL_DELAY_BASIC_H_INCLUDED	1

/*****************************************************************************/

#include <stdint.h>

static inline void _delay_loop_0(uint32_t loops)
{
 /* this routine waits 4*loops clocks: */
 __asm__ (".syntax unified");

 __asm__ volatile 
  (	"mov	r3,%[loops]	\n\t" 	/* load the initial counter 	*/
	"1:			\n\t"
	"subs	r3, #1		\n\t"
	"bne	1b		\n\t"
	:				/* empty output list		*/
	: [loops] "r" (loops)		/* input to the asm routine	*/
	: "r3", "cc"			/* clobber list			*/
  );

}

static inline void _delay_loop(uint32_t loops)
{
 /* this routine waits 4*loops clocks: */
 __asm__ (".syntax unified");

 __asm__ volatile 
  (	"mov	r3,%[loops]	\n\t" 	/* load the initial counter 	*/
	"1:			\n\t"
	"nop			\n\t"
	"subs	r3, #1		\n\t"
	"bne	1b		\n\t"
	:				/* empty output list		*/
	: [loops] "r" (loops)		/* input to the asm routine	*/
	: "r3", "cc"			/* clobber list			*/
  );

}

/*****************************************************************************/

#endif

/*****************************************************************************/
                                                                     
           
