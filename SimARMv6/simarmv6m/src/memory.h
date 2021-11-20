/*****************************************************************************/
/* memory.h								     */
/*****************************************************************************/

#ifndef	__MEMORY_H_INCLUDED
#define	__MEMORY_H_INCLUDED	1

/*****************************************************************************/

typedef struct memory_block
 {	
	struct	memory_block	*prev,*next;
	uint32_t		mb_base;
	uint32_t		mb_size;

	unsigned char		*mb_array;	/* non-NULL if a physical RAM/ROM is simulated */
						/* if mb_array is NULL, then callbacks are used */
	void			*mb_param;	/* optional parameter for callback is used for LDRx/STRx (e.g. for peripherals) */

	int			(*mb_callback_ldr )(void *param,uint32_t offset,uint32_t *data);
	int			(*mb_callback_ldrh)(void *param,uint32_t offset,uint16_t *data);
	int			(*mb_callback_ldrb)(void *param,uint32_t offset,uint8_t  *data);
	int			(*mb_callback_str )(void *param,uint32_t offset,uint32_t data);
	int			(*mb_callback_strh)(void *param,uint32_t offset,uint16_t data);
	int			(*mb_callback_strb)(void *param,uint32_t offset,uint8_t  data);

 } memory_block;

typedef struct
 {	memory_block	*ml_block_first;
	memory_block	*progmem_mb;
	memory_block	*ram_mb;
 } memory_layout;

#define         MEMORY_ACCESS_HINT_NONE         0
#define         MEMORY_ACCESS_HINT_PROGMEM      (1<<0)
#define         MEMORY_ACCESS_HINT_RAM          (1<<1)
#define         MEMORY_ACCESS_HINT_STACK        MEMORY_ACCESS_HINT_RAM

memory_block *	memory_get_block(memory_layout *ml,uint32_t address,int hint);
int		memory_ldr(memory_layout *ml,uint32_t address,uint32_t *data,int hint);
int		memory_ldrh(memory_layout *ml,uint32_t address,uint16_t *data);
int		memory_ldrb(memory_layout *ml,uint32_t address,uint8_t *data);
int		memory_str(memory_layout *ml,uint32_t address,uint32_t data,int hint);
int		memory_strh(memory_layout *ml,uint32_t address,uint16_t data);
int		memory_strb(memory_layout *ml,uint32_t address,uint8_t data);

/*****************************************************************************/

#endif

/*****************************************************************************/
