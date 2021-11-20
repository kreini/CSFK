#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

#include "list.h"

#include "memory.h"
#include "simarmv6m.h"
#include "timeval.h"
#include "stm32f0_usart.h"

/* 0x40013800
   0x40004400 
   0x40004800 
   0x40004C00 */
typedef struct
 {	uint32_t	usart_cr1;	/* 0x00	*/
	uint32_t	usart_cr2;	/* 0x04 */
	uint32_t	usart_cr3;	/* 0x08	*/
	uint16_t	usart_brr;	/* 0x0C	*/
	uint16_t	__padding_1;	/* 0x0E */
	uint16_t	usart_gtpr;	/* 0x10	*/
	uint16_t	__padding_2;	/* 0x12	*/
	uint32_t	usart_rtor;	/* 0x14	*/
	uint16_t	usart_rqr;	/* 0x18 */
	uint16_t	__padding_3;	/* 0x1A	*/
	uint32_t	usart_isr;	/* 0x1C	*/
	uint32_t	usart_icr;	/* 0x20	*/
	uint16_t	usart_rdr;	/* 0x24 */
	uint16_t	__padding_4;	/* 0x26	*/
	uint16_t	usart_tdr;	/* 0x28	*/
	uint16_t	__padding_5;	/* 0x2A */
					/* 0x2C == 44 bytes in total */
 } stm32f0_register_usart;

typedef struct
 {	stm32f0_register_usart	usart_reg_usart;
	int			usart_word_time_usec;
	uint32_t		*usart_cstk_clock_fmhz;
	struct timeval		usart_putc_tv;
	int			(*usart_putc_callback)(void *,uint16_t);
	void *			usart_putc_param;
	int			usart_getc_fd;
	struct timeval		usart_getc_tv;
	unsigned char		usart_getc_fifo[16];
	int			usart_getc_ptr;
	int			usart_getc_avail;
	int			usart_interrupt_number;
	int			usart_is_virtual; 
 } stm32f0_usart;

static int fd_is_available(int fd)
{
 struct timeval tv;
 fd_set         set;
 int            ret;
 
 tv.tv_sec=0;
 tv.tv_usec=0;
 
 FD_ZERO(&set);
 FD_SET(fd,&set);

 ret=select(fd+1,&set,NULL,NULL,&tv);

 if ( ret<0 && errno==EINTR )
  {     sig_int=1;
        return(0);
  }

 if ( FD_ISSET(fd,&set) )
        return(1);
 else
        return(0);
}

static int stm32f0_usart_getc_update(stm32f0_usart *usart,struct timeval *tv)
{
 if ( 0<=usart->usart_getc_fd )
  {	
	if ( usart->usart_getc_avail <= usart->usart_getc_ptr )
	 {	usart->usart_getc_avail=0;
		usart->usart_getc_ptr=0;
		if ( fd_is_available(usart->usart_getc_fd) )
		 {	int	r;
			r=read(usart->usart_getc_fd,usart->usart_getc_fifo,sizeof(usart->usart_getc_fifo));		 
			usart->usart_getc_avail=r;
			/* fprintf(stderr,"read(): %d\n",r); */
		 }
	 }


	if ( usart->usart_getc_ptr < usart->usart_getc_avail )
	 {	if ( usart->usart_is_virtual )
			usart->usart_reg_usart.usart_isr |=  (1<<5); /* RXNE */
		else
		 {	double	diff;
			diff=timeval_diff(&usart->usart_getc_tv,tv);
			if ( (usart->usart_word_time_usec)*1e-6 <= diff )
				usart->usart_reg_usart.usart_isr |=  (1<<5); /* RXNE */
		 }
	 }
  }
 return(0);
}

static int stm32f0_usart_putc_update(stm32f0_usart *usart,struct timeval *tv)
{
 if ( usart->usart_is_virtual )
  {	usart->usart_reg_usart.usart_isr |=  ((1<<6)|(1<<7));
  }
 else
  {	double	diff;
	diff=timeval_diff(&usart->usart_putc_tv,tv);
	if ( (usart->usart_word_time_usec)*1e-6 <= diff )
		usart->usart_reg_usart.usart_isr |=  ((1<<6)|(1<<7));
	else
		usart->usart_reg_usart.usart_isr &= ~((1<<6)|(1<<7));
  }

 return(0); 
}

static int stm32f0_usart_ldr(void *param,uint32_t offset,uint32_t *data)
{
 stm32f0_usart	*usart=param;
 uint32_t	ret;

/*
 struct	timeval	tv;
 gettimeofday(&tv,NULL);
 stm32f0_usart_putc_update(usart,&tv);
 stm32f0_usart_getc_update(usart,&tv);
*/

 if ( offset<44 )
	memcpy(&ret,(uint32_t *)(&usart->usart_reg_usart)+(offset>>2),4);
 else
	ret=0;

 if ( data != NULL )
	*data=ret;

 return(0);
}

static int stm32f0_usart_ldrh(void *param,uint32_t offset,uint16_t *data)
{
 stm32f0_usart	*usart=param;
 uint16_t	ret;

/*
 struct	timeval	tv;
 gettimeofday(&tv,NULL);
 stm32f0_usart_getc_update(usart,&tv);
*/

 /* read from to RDR: */
 if ( offset==36 && usart->usart_getc_ptr < usart->usart_getc_avail )
  {	
	struct	timeval	tv;
	gettimeofday(&tv,NULL);

	usart->usart_reg_usart.usart_rdr = usart->usart_getc_fifo[usart->usart_getc_ptr];
	usart->usart_getc_ptr++;
	usart->usart_getc_tv = tv;
	usart->usart_reg_usart.usart_isr &= ~(1<<5);
  }

 if ( offset<44 )
	memcpy(&ret,(uint16_t *)(&usart->usart_reg_usart)+(offset>>1),2);
 else
	ret=0;

 if ( data != NULL )
	*data=ret;

 return(0);
}

static int stm32f0_usart_str(void *param,uint32_t offset,uint32_t data)
{
 stm32f0_usart	*usart=param;

 if ( offset<44 )
	memcpy((uint32_t *)(&usart->usart_reg_usart)+(offset>>2),&data,4);

 return(0);
}

static int stm32f0_usart_strh(void *param,uint32_t offset,uint16_t data)
{
 stm32f0_usart	*usart=param;

 if ( offset<44 )
	memcpy((uint16_t *)(&usart->usart_reg_usart)+(offset>>1),&data,2);

 /* one data word is 10 bit times (start, 8x length, stop) */
 /* it might be fine-tuned later to be more accurate... */
 if ( offset==12 )
  {	int	gross_word_length;
	gross_word_length=1+8+1;
	usart->usart_word_time_usec=(data*gross_word_length)/(*usart->usart_cstk_clock_fmhz);
  }

 /* write to TDR: */
 else if ( offset==40 )
  {	struct timeval 	tv;

	if ( usart->usart_reg_usart.usart_isr & (1<<7) )
	 {	gettimeofday(&tv,NULL);
	 	usart->usart_putc_callback(usart->usart_putc_param,data);
		if ( ! usart->usart_is_virtual) 
			usart->usart_reg_usart.usart_isr &= ~((1<<6)|(1<<7));
		usart->usart_putc_tv=tv;
	 }

  }	

 return(0);
}

static int stm32f0_usart_exception_update(void *param)
{
 stm32f0_usart	*usart=param;
 struct timeval	tv;

 gettimeofday(&tv,NULL);
 stm32f0_usart_getc_update(usart,&tv);
 stm32f0_usart_putc_update(usart,&tv);
 
 return(0);
}

static int stm32f0_usart_exception_state(void *param)
{
 stm32f0_usart	*usart=param;

 if ( usart->usart_reg_usart.usart_cr1 & usart->usart_reg_usart.usart_isr & ((1<<7)|(1<<6)|(1<<5)) )
	return(16+usart->usart_interrupt_number);
 else
	return(0);
}

static int stm32f0_usart_exception_delay_getc(stm32f0_usart *usart,struct timeval *curr_tv)
{
 if ( ! (usart->usart_reg_usart.usart_cr1 & (1<<5)) || usart->usart_getc_fd<0 )
	return(-1);
 else if ( usart->usart_reg_usart.usart_cr1 & usart->usart_reg_usart.usart_isr & (1<<5) )
	return(0);
 else if ( usart->usart_getc_ptr < usart->usart_getc_avail )
  {	double	diff;
	int	delay;

	diff=timeval_diff(&usart->usart_getc_tv,curr_tv);
	if ( (usart->usart_word_time_usec)*1e-6 <= diff )
		return(0);

	delay=usart->usart_word_time_usec-(int)(diff*1e6);
	if ( delay<0 )
		delay=0;

	return(delay);
  }
 else
	return(-1);
}

static int stm32f0_usart_exception_delay(void *param)
{
 stm32f0_usart	*usart=param;
 struct	timeval	tv;
 int		delay_rxne;

 gettimeofday(&tv,NULL);

 delay_rxne=stm32f0_usart_exception_delay_getc(usart,&tv);

 return(delay_rxne);
}

static int stm32f0_usart_exception_fdesc(void *param)
{
 stm32f0_usart	*usart=param;

 if ( ! (usart->usart_reg_usart.usart_cr1 & (1<<5)) || usart->usart_getc_fd<0 )
	return(-1);

 if ( usart->usart_reg_usart.usart_cr1 & usart->usart_reg_usart.usart_isr & (1<<5) )
	return(-1);

 if ( usart->usart_getc_ptr < usart->usart_getc_avail )
	return(-1);

 return(usart->usart_getc_fd);
}

static int fstream_usart_putc(void *param,uint16_t data)
{
 FILE	*fw=param;
/*
 static	int	iline=0;

 if ( iline==0 )
  {	struct timeval  tv;
	gettimeofday(&tv,NULL);
	fprintf(stderr,"[%d.%.6d] ",(int)tv.tv_sec,(int)tv.tv_usec);
  }
*/

 fprintf(fw,"%c",(char)data);
/* iline = (data != 10); */

 fflush(fw);

 return(0);
}

static int filedesc_usart_putc(void *param,uint16_t data)
{
 int	*fd_ptr = param,fd;

 fd=*fd_ptr;

 write(fd,&data,1);

 return(0);
}

int stm32f0_usart_register(memory_layout *ml,armv6m_exception_control *aec,armv6m_update_control *auc,peripheral_definition *pd,uint32_t *clock_mhz)
{
 key_value		*kv;
 memory_block		*mb;
 stm32f0_usart		*usart;
 armv6m_exception	*ae;
 armv6m_update		*au;

 int			interrupt_number,is_virtual;
 char			*device;

 mb=memory_layout_add_block(ml);
 mb->mb_base=0x40004400;
 mb->mb_size=64;
 mb->mb_array=NULL;

 interrupt_number=0;
 is_virtual=0;

 device=NULL;

 for ( kv=pd->pd_list_kv; kv != NULL; kv=kv->next )
  {	int	w;
	if ( strcmp(kv->kv_key,"base")==0 && kv->kv_value != NULL && sscanf(kv->kv_value,"%i",&w)==1 )
 	 {	mb->mb_base=(uint32_t)w;
	 }
	else if ( strcmp(kv->kv_key,"interrupt")==0 && kv->kv_value != NULL && sscanf(kv->kv_value,"%i",&w)==1 )
 	 {	interrupt_number=w;
	 }
	else if ( strcmp(kv->kv_key,"device")==0 && kv->kv_value != NULL )
 	 {	device=kv->kv_value;
	 }
	else if ( strcmp(kv->kv_key,"virtual")==0 )
 	 {	is_virtual=1;
	 }
	else
	 {	/*
		fprintf(stderr,"%s: error: invalid %s/%s key/value pair for '%s'.\n",
			argv[0],pd->pd_device,pd->pd_peripheral,kv->kv_key);
		*/
		return(1);
	 }
  }

 usart=malloc(sizeof(stm32f0_usart));
 pd->pd_data=usart;
 mb->mb_param=usart;

 memset(&usart->usart_reg_usart,0,sizeof(stm32f0_register_usart));
 usart->usart_reg_usart.usart_isr=(1<<6)|(1<<7);
 usart->usart_cstk_clock_fmhz = clock_mhz;
 usart->usart_putc_tv.tv_sec=0;
 usart->usart_putc_tv.tv_usec=0;
 usart->usart_getc_tv.tv_sec=0;
 usart->usart_getc_tv.tv_usec=0;

 if ( device != NULL )
  {	int	fd;
	if ( (fd=open(device,O_RDWR))<0 )
	 {	/* fprintf(stderr,"%s: error: unable to open serial device '%s'.\n",argv[0],device); */
		return(1);
	 }
	usart->usart_putc_callback = filedesc_usart_putc;
	usart->usart_getc_fd = fd;
	usart->usart_putc_param = &usart->usart_getc_fd;
  }
 else
  {	usart->usart_putc_callback = fstream_usart_putc;
	usart->usart_putc_param = stdout;
	usart->usart_getc_fd = 0;
  }

 usart->usart_getc_ptr = 0;
 usart->usart_getc_avail = 0;

 usart->usart_interrupt_number=interrupt_number;
 usart->usart_is_virtual=is_virtual;

 /* USARTs has both word access and half-word access: */
 mb->mb_callback_ldr =stm32f0_usart_ldr;
 mb->mb_callback_ldrh=stm32f0_usart_ldrh;
 mb->mb_callback_str =stm32f0_usart_str;
 mb->mb_callback_strh=stm32f0_usart_strh;

 /* create the exception generator: */
 ae=armv6m_exception_add(aec);
               
 ae->ae_callback_exception_state=stm32f0_usart_exception_state;
 ae->ae_callback_exception_delay=stm32f0_usart_exception_delay;
 ae->ae_callback_exception_fdesc=stm32f0_usart_exception_fdesc;
 ae->ae_param=usart;

 au=armv6m_update_add(auc);
 au->au_callback_update=stm32f0_usart_exception_update;
 au->au_param=usart;

 return(0);
}
