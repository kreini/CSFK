/* Memory configuration confirms to STM32F072CBT7 (main firmware section, 128K) */

MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Generic libc-compatible memory mapping: */

SECTIONS
{
	.text :
	 {	
		*(.vectors)		/* Vector table */
		*(.text)	/* Program code */
		. = ALIGN(4);
		*(.rodata)	/* Read only data */
		. = ALIGN(4);
	 } >rom

	.ramvectors :
	 {	*(.ramvectors)	/* soft copy of the vector table */
	 } >ram

	__data_rom_start = LOADADDR(.data);
	.data :
	 {	__data_ram_start = .;
		*(.data)	/* Data memory */
		. = ALIGN(4);	
		__data_ram_end = .;
	 } >ram AT >rom 

	.bss :
	 {	__bss_start = .;
		*(.bss)		/* Zero-filled run time allocate data memory */
		*(COMMON)
		. = ALIGN(4);
		__bss_end = .;
	 } >ram

	. = ALIGN(4);
	end = .;
}

PROVIDE(__stack_top = ORIGIN(ram) + LENGTH(ram)); 
PROVIDE(__flash_end = ORIGIN(rom) + LENGTH(rom));
