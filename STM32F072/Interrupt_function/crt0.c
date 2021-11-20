#include <stdint.h>
#include <stm32f0xx.h>

static void	_startup(void);
static void	_nmi_handler(void);
static void	_hardfault_handler(void);

extern	const uint32_t	__stack_top;

int main(void);

/* Define the vector table: */

uint32_t * _VECTOR_M0[16] __attribute__ ((section(".vectors.m0"))) = 
 {	(uint32_t *) &__stack_top,
	(uint32_t *) _startup,
	(uint32_t *) _nmi_handler,
	(uint32_t *) _hardfault_handler,
 };

void  _nmi_handler(void) 
{
    for(;;);
}

void  _hardfault_handler(void)  
{
    for(;;);
}

extern	uint32_t __data_rom_start;
extern	uint32_t __data_ram_start;
extern	uint32_t __data_ram_end;
extern	uint32_t __bss_start;
extern	uint32_t __bss_end;

void  _startup(void) 
{
 uint32_t * data_rom_start_p	= &__data_rom_start;
 uint32_t * data_ram_start_p	= &__data_ram_start;
 uint32_t * data_ram_end_p	= &__data_ram_end;
 uint32_t * bss_start_p		= &__bss_start;
 uint32_t * bss_end_p 		= &__bss_end;

 while ( data_ram_start_p != data_ram_end_p )
  {	*data_ram_start_p = *data_rom_start_p;
	data_ram_start_p++;
	data_rom_start_p++;
  };

 while ( bss_start_p != bss_end_p )
  {	*bss_start_p = 0;
	bss_start_p++;
  }

 main();

 while(1);
}
