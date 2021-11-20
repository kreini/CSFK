/*****************************************************************************/
/* memory.c								     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* An 32-bit memory bus matrix, designed for the ARMv6-M simulator	     */
/*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>

#include "memory.h"

#define	memory_in_range(mb,address) ((mb)->mb_base <= (address) && (address)<(mb)->mb_base+(mb)->mb_size)

memory_block *memory_get_block(memory_layout *ml,uint32_t address,int hint)
{
 memory_block *mb;

 if ( (hint & MEMORY_ACCESS_HINT_PROGMEM) && ml->progmem_mb != NULL && memory_in_range(ml->progmem_mb,address) )
	return(ml->progmem_mb);
 else if ( (hint & MEMORY_ACCESS_HINT_RAM) && ml->ram_mb != NULL && memory_in_range(ml->ram_mb,address) )
	return(ml->ram_mb);

 for ( mb=ml->ml_block_first; mb != NULL; mb=mb->next )
  {	if ( memory_in_range(mb,address) )
		return(mb);
  }
 return(NULL);
}

int memory_ldr(memory_layout *ml,uint32_t address,uint32_t *data,int hint)
{
 memory_block *mb;
 if ( (mb=memory_get_block(ml,address,hint))==NULL )
	return(-1);
 else if ( data==NULL )
	return(-2);
 else if ( mb->mb_array != NULL )
  {	address -= mb->mb_base;
	if ( address < mb->mb_size )
	 {	*data=*(uint32_t *)(mb->mb_array+address);
		return(0);
	 }
	else
	 {	if ( 0 )
			fprintf(stderr,"memory_ldr(): bus error at block base=%.8x size=%.8x offset=%.8x\n",mb->mb_base,mb->mb_size,address);
		return(-4);
	 }
  }
 else if ( mb->mb_callback_ldr != NULL ) 
  {	address -= mb->mb_base;
	mb->mb_callback_ldr(mb->mb_param,address,data);
	return(0);
  }
 else
	return(-3);
}

int memory_ldrh(memory_layout *ml,uint32_t address,uint16_t *data)
{
 memory_block *mb;
 if ( (mb=memory_get_block(ml,address,0))==NULL )
	return(-1);
 else if ( data==NULL )
	return(-2);
 else if ( mb->mb_array != NULL )
  {	address -= mb->mb_base;
	*data=*(uint16_t *)(mb->mb_array+address);
	return(0);
  }
 else if ( mb->mb_callback_ldrh != NULL ) 
  {	address -= mb->mb_base;
	mb->mb_callback_ldrh(mb->mb_param,address,data);
	return(0);
  }
 else
	return(-3);
}

int memory_ldrb(memory_layout *ml,uint32_t address,uint8_t *data)
{
 memory_block *mb;
 if ( (mb=memory_get_block(ml,address,0))==NULL )
	return(-1);
 else if ( data==NULL )
	return(-2);
 else if ( mb->mb_array != NULL )
  {	address -= mb->mb_base;
	*data=*(uint8_t *)(mb->mb_array+address);
	return(0);
  }
 else if ( mb->mb_callback_ldrb != NULL ) 
  {	address -= mb->mb_base;
	mb->mb_callback_ldrb(mb->mb_param,address,data);
	return(0);
  }
 else
	return(-3);
}

int memory_str(memory_layout *ml,uint32_t address,uint32_t data,int hint)
{
 memory_block *mb;
 if ( (mb=memory_get_block(ml,address,hint))==NULL )
	return(-1);
 else if ( mb->mb_array != NULL )
  {	address -= mb->mb_base;
	*(uint32_t *)(mb->mb_array+address)=data;
	return(0);
  }
 else if ( mb->mb_callback_str != NULL ) 
  {	address -= mb->mb_base;
	mb->mb_callback_str(mb->mb_param,address,data);
	return(0);
  }
 else
	return(-3);
}

int memory_strh(memory_layout *ml,uint32_t address,uint16_t data)
{
 memory_block *mb;
 if ( (mb=memory_get_block(ml,address,0))==NULL )
	return(-1);
 else if ( mb->mb_array != NULL )
  {	address -= mb->mb_base;
	*(uint16_t *)(mb->mb_array+address)=data;
	return(0);
  }
 else if ( mb->mb_callback_strh != NULL ) 
  {	address -= mb->mb_base;
	mb->mb_callback_strh(mb->mb_param,address,data);
	return(0);
  }
 else
	return(-3);
 
}

int memory_strb(memory_layout *ml,uint32_t address,uint8_t data)
{
 memory_block *mb;

 if ( (mb=memory_get_block(ml,address,0))==NULL )
	return(-1);
 else if ( mb->mb_array != NULL )
  {	address -= mb->mb_base;
	*(uint8_t *)(mb->mb_array+address)=data;
	return(0);
  }
 else if ( mb->mb_callback_strb != NULL ) 
  {	address -= mb->mb_base;
	mb->mb_callback_strb(mb->mb_param,address,data);
	return(0);
  }
 else
	return(-3);

}
