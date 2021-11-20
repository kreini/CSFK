/*****************************************************************************/
/* simarmv6m.c								     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* A simple and minimalistic extensible ARMv6-M core simulator with 	     */
/* some essential Cortex-M0 peripherals and additional features.	     */
/*****************************************************************************/

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

#include "simarmv6m.h"
#include "timeval.h"
#include "armv6m.h"
#include "memory.h"
#include "stm32f0_usart.h"

/* 0xE000E100+ */
typedef struct
 {	uint32_t	cn_interrupt_enable;
	uint32_t	cn_interrupt_pending;
	uint32_t	cn_interrupt_prioriy[7];
 } cm0_nvic;

/* 0xE000ED00 */
typedef struct
 {	/* 0x00	*/	uint32_t	cscb_cpuid;
	/* 0x04	*/	uint32_t	cscb_icsr;
	/* 0x08 */	uint32_t	cscb_reserved_1;
	/* 0x0C */	uint32_t	cscb_aircr;
	/* 0x10 */	uint32_t	cscb_scr;
	/* 0x14 */	uint32_t	cscb_ccr;
	/* 0x18 */	uint32_t	cscb_reserved_2;
	/* 0x1C */	uint32_t	cscb_shpr2;
	/* 0x20 */	uint32_t	cscb_shpr3;
 } cm0_scb;

/* 0xE000E010 */
typedef struct
 {	/* 0x00 */	uint32_t	cstk_csr;
	/* 0x04 */	uint32_t	cstk_rvr;
	/* 0x08 */	uint32_t	cstk_cvr;
	/* 0x0C */	uint32_t	cstk_calib;
	uint32_t			cstk_clock_fmhz;	/* used by the simulator */
	struct timeval			cstk_start_tv;		/* used by the simulator */
 } cm0_systick;

/* 0x40010000 */
typedef struct
 {	memory_layout	*sc_ml;
	uint32_t	sc_cfgr1;
 } stm32f0_syscfg;

/* 0x40021000 */
typedef struct
 {	uint32_t	rcc_cr;
 } stm32f0_rcc;

/*****************************************************************************/

volatile int sig_int=0;

void sig_int_handler(int arg)
{
 sig_int=!0;
}

/*****************************************************************************/

int armv6m_exception_check(memory_layout *ml,armv6m_state *as,armv6m_exception_control *aec)
{ 
 armv6m_exception	*ae;
 int			highest_pri_vector;

 if ( aec==NULL )
	return(0);

 highest_pri_vector=0;

 for ( ae=aec->aec_first; ae != NULL ; ae=ae->next )
  {	int	vector;

	if ( ae->ae_callback_exception_state==NULL )
		continue;

	else if ( 0<(vector=ae->ae_callback_exception_state(ae->ae_param)) )
	 {	if ( ! highest_pri_vector )
			highest_pri_vector=vector;
		else if ( vector<highest_pri_vector )
			highest_pri_vector=vector;
	 }

  }

 return(highest_pri_vector);
}

/* 

armv6m_exception_delay()

This function returns:
  - a negative number if there are no timer-activated and enabled (at the 
    peripheral level) interrupts in the system;
  - zero if any of the timer-activeted interrupts are pending; or
  - a positive number, providing the minimum wait time in microseconds 
    until nothing would surely happen due to the presence of enabled 
    timer-activated interrupts. 
See also the notes at armv6m_exception_fdset() for further information.

*/

int armv6m_exception_delay(memory_layout *ml,armv6m_state *as,armv6m_exception_control *aec)
{
 armv6m_exception       *ae;
 int                    min_time;

 if ( aec==NULL )
        return(-1);

 min_time=-1;
 
 for ( ae=aec->aec_first; ae != NULL ; ae=ae->next )
  {     int	mt;

	if ( ae->ae_callback_exception_delay==NULL )
		continue;
	else if ( (mt=ae->ae_callback_exception_delay(ae->ae_param))<0 )
		continue;
	else if ( min_time<0 || mt<min_time )
		min_time=mt;
  }      

 return(min_time);
}

/* 

armv6m_exception_fdset()

This function updates the fd_set and returns:
 - a negative number if there are no input-activated and enabled (at the 
   peripheral level) interrupts in the system; or
 - a non-negative number if there is at least one input-activated and enabled 
   (at the peripheral level) intterupt.

Some notes:
 - If both armv6m_exception_delay() and armv6m_exception_fdset() returns a 
   negative number, then there isn't any internal input-activated and 
   timer-activated interrupts enabled at the peripheral level. It can be 
   normal, however, it means that a WFI instruction will surely stuck
   the system. In this case, i.e. if such a condition is detected, the 
   simulation is going to be stopped.
 - A simulated physical interrupt source can yield both a timer-actived
   and an input-activated interrupt depending on its own state. For instance,
   a simulator for a UART receiver reading data from a file descriptor 
   can behave as follows.
	* At first, no data is in the internal FIFO, so its *_delay() would 
	  return -1 while the corresponding *_fdset() would return (and set)
	  the file descriptor from which data are expected.
	* Once some data are read (and optionally stored in a FIFO),
	  the interrupt bit is set accordingly.
	* Once a data byte is read but there are more data bytes pending 
	  (either from the input file descriptor or there is something in the
	  FIFO), the time instance is saved at the time of the access and 
	  *_delay() should return in accordance to the current time,
  	  the last access time and the baud rate associated to this UART.
	  The corresponding bit (e.g. RXNE, in this case) will be set only
	  after sufficient amount of time has been elapsed since the 
 - In other words, in WFI state the underlying select() of the simulator 
   performs the synchronous multiplexing and timeout control in accordance to 
   the return values of armv6m_exception_fdset() and armv6m_exception_delay(),
   respectively:
	* if armv6m_exception_delay() returns zero, select() is not called
	  since interrupts can then instantly processed;
	* if both are non-negative, select() is called with a proper fd_set
	  and a non-NULL timeout value
	* if armv6m_exception_fdset() returns a negative value, only an
	  sleep() is called with the return value of armv6m_exception_delay().
	* if armv6m_exception_delay() returns a negative value but 
	  armv6m_exception_fdset() returns a non-negative value, then
	  sleect() is called without a timeout (i.e. the processor will
	  be in WFI state until any activity is detected from any of the 
	  read file descriptors)
	* if both functions return a negative value then the system is 
	  expected to be stuck forever in WFI state and the simulation stopped.
 - armv6m_exception_fdset() would return a negative number in both "extreme"
   cases, i.e. if there is no input file descriptor available serving as 
   an interrupt source as well as if the interrupt source has already 
   been read something from that file descriptor that data would certainly
   initiate an interrupt request. On the other hand this latter case 
   implies a zero return value for armv6m_exception_delay() so select()
   would not block. 
*/


int armv6m_exception_fdset(memory_layout *ml,armv6m_state *as,armv6m_exception_control *aec,fd_set *set)
{
 armv6m_exception       *ae;
 int			max;

 FD_ZERO(set);
 max=-1;

 for ( ae=aec->aec_first; ae != NULL ; ae=ae->next )
  {	int	fd;

	if ( ae->ae_callback_exception_fdesc==NULL )
		continue;
	else if ( (fd=ae->ae_callback_exception_fdesc(ae->ae_param))<0 )
		continue;
	else
	 {	FD_SET(fd,set);
		if ( max<fd )	max=fd;
	 }
  }

 return(max);
}


armv6m_exception *armv6m_exception_add(armv6m_exception_control *aec)
{
 armv6m_exception	*ae;
 
 ae=list_new(armv6m_exception);

 ae->ae_callback_exception_state=NULL;
 ae->ae_callback_exception_delay=NULL;
 ae->ae_callback_exception_fdesc=NULL;
 ae->ae_param=NULL;

 list_insert_first(aec->aec_first,ae);
 
 return(ae);
}

armv6m_update *armv6m_update_add(armv6m_update_control *auc)
{
 armv6m_update	*au;
 
 au=list_new(armv6m_update);

 au->au_callback_update=NULL;
 au->au_param=NULL;

 list_insert_first(auc->auc_first,au);
 
 return(au);
}

memory_block *memory_layout_add_block(memory_layout *ml)
{
 memory_block	*mb;

 mb=list_new(memory_block);

 list_insert_first(ml->ml_block_first,mb);

 mb->mb_base=0;
 mb->mb_size=0;
 mb->mb_array=NULL;
 mb->mb_param=NULL;
 mb->mb_callback_ldr=NULL;
 mb->mb_callback_ldrh=NULL;
 mb->mb_callback_ldrb=NULL;
 mb->mb_callback_str=NULL;
 mb->mb_callback_strh=NULL;
 mb->mb_callback_strb=NULL;

 return(mb);
}

static uint32_t cortexm0_systick_increment(struct timeval *t0,struct timeval *t1,uint32_t fmhz)
{
 struct	timeval	tv;
 uint32_t	increment;

 tv.tv_sec=t1->tv_sec-t0->tv_sec;
 tv.tv_usec=t1->tv_usec-t0->tv_usec;
 if ( tv.tv_usec<0 )	
  {	tv.tv_usec+=1000000;
	tv.tv_sec--;
  }

 increment=1000000*tv.tv_sec*fmhz+tv.tv_usec*fmhz;

 return(increment);
}

static int cortexm0_systick_update_increment(uint32_t *cvr,uint32_t rvr,uint32_t increment)
{
 int	noverflow;

 noverflow = increment/(rvr+1);
 increment = increment%(rvr+1);

 if ( increment < *cvr )
  {	*cvr -= increment;
	return(noverflow);
  }
 else if ( increment == *cvr )
  {	*cvr = 0;
	return(noverflow+1);
  }
 else if ( *cvr == 0 && increment<rvr+1 )
  {	*cvr = rvr+1 - increment;
	return(noverflow);
  }
 else
  {	if ( *cvr < increment )
		*cvr += rvr+1 - increment;
	else
		*cvr -= increment;

	return(noverflow+1);
  }
}

int cortexm0_systick_update_realtime(void *param)
{
 cm0_systick	*cstk=param;

 struct		timeval	tv;
 uint32_t	increment;
 int		noverflow;

 if ( ! ( cstk->cstk_csr & 1 ) )
	return(0);

 gettimeofday(&tv,NULL);

 increment=cortexm0_systick_increment(&cstk->cstk_start_tv,&tv,cstk->cstk_clock_fmhz);

 if ( 0<(noverflow=cortexm0_systick_update_increment(&cstk->cstk_cvr,cstk->cstk_rvr,increment)) )
  { 	cstk->cstk_csr |= (1<<16);
	noverflow--;
	if ( 1 && noverflow )
	 {	fprintf(stderr,"cortexm0_systick_update_realtime(): overflow=%d [increment=%u]\n",noverflow,increment);
	 }
  }

 cstk->cstk_start_tv=tv;
 
 return(0);
}

int cortexm0_systick_ldr(void *param,uint32_t offset,uint32_t *data)
{
 cm0_systick	*cstk=param;
 uint32_t	ret;

 if ( offset==0 || offset==8 )
	cortexm0_systick_update_realtime(cstk);

 if ( offset==0 )
  {	ret=cstk->cstk_csr;
	cstk->cstk_csr &= ~(1<<16);
  }
 else if ( offset==4 )
  {  	ret=cstk->cstk_rvr & 0x00FFFFFF;
  }
 else if ( offset==8 )
  {	ret=cstk->cstk_cvr & 0x00FFFFFF;
  }
 else if ( offset==12 )
  {	ret=cstk->cstk_calib & 0x00FFFFFF;
  }
 else
	ret=0;

/* fprintf(stderr,"SYSTICK: [0x%.8x] = 0x%.8x\n",offset,ret); */

 if ( data != NULL )
	*data=ret;

 return(0);
}


int cortexm0_systick_str(void *param,uint32_t offset,uint32_t data)
{
 cm0_systick	*cstk=param;

/* fprintf(stderr,"SYSTICK: 0x%.8x <= 0x%.8x\n",offset,data);  */

 if ( offset==0 )
  {	
	/* a disable -> enable transition: */
	if ( ! ( cstk->cstk_csr & 1 ) && ( data & 1 ) )
	 {	gettimeofday(&cstk->cstk_start_tv,NULL);
	 }

	/* an enable -> disable transition: */
	else if ( ( cstk->cstk_csr & 1 ) && ! ( data & 1 ) )
		cortexm0_systick_update_realtime(cstk);

	cstk->cstk_csr = (cstk->cstk_csr & (~7)) | (data & 0x7);
  }
 else if ( offset==4 )
  {	cstk->cstk_rvr = data & 0x00FFFFFF;
  }
 else if ( offset==8 )
  {	cstk->cstk_cvr = data & 0x00FFFFFF;
  }
 else if ( offset==12 )
  {	cstk->cstk_calib = 0;
  }

 return(0);
}

int cortexm0_systick_exception_state(void *param)
{
 cm0_systick    *cstk=param;
 
 if ( (cstk->cstk_csr & (1<<1)) && (cstk->cstk_csr & (1<<16)) )
  {	cstk->cstk_csr &= ~(1<<16);
	return(15);
  }
 else
	return(0);
}

int cortexm0_systick_exception_delay(void *param)
{
 cm0_systick    *cstk=param;
 
 if ( cstk->cstk_csr & (1<<1) )
  {	if ( cstk->cstk_csr & (1<<16) )
		return(0);	
	else 
		return(cstk->cstk_cvr/cstk->cstk_clock_fmhz);
  }
 else
	return(-1);
}


int cortexm0_scb_ldr(void *param,uint32_t offset,uint32_t *data)
{
 uint32_t	*cptr=param;
 uint32_t	ret;

 if ( offset<=0x20 && offset%4==0 )
	ret=cptr[offset/4];
 else
	ret=0;

 if ( data != NULL )
	*data=ret;

 return(0);
}


int cortexm0_scb_str(void *param,uint32_t offset,uint32_t data)
{
 uint32_t	*cptr=param;

 if ( offset<=0x20 && offset%4==0 )
	cptr[offset/4]=data;

 return(0);
}

int cortexm0_scb_exception_state(void *param)
{
 cm0_scb	*cscb=param;

 if ( cscb->cscb_icsr & (1<<28) )
  {	cscb->cscb_icsr &= ~(1<<28);
	return(14);
  }
 else if ( cscb->cscb_icsr & (1<<26) )
  {	cscb->cscb_icsr &= ~(1<<26);
	return(15);
  }
 else
	return(0);
}


int map_vector_table(memory_layout *ml,memory_block *vector_mb,uint32_t address)
{
 memory_block	*mb;
 mb=memory_get_block(ml,address,0);
 if ( mb != NULL )
  {	vector_mb->mb_array = mb->mb_array;
	vector_mb->mb_size  = mb->mb_size;
	return(0);
  }
 else
	return(1);
}


int stm32f0_syscfg_ldr(void *param,uint32_t offset,uint32_t *data)
{
 stm32f0_syscfg	*sc=param;
 uint32_t	ret;

 if ( offset==0 )
	ret=sc->sc_cfgr1;
 else
	ret=0;

 if ( data != NULL )
	*data=ret;

 return(0);
}

int stm32f0_syscfg_str(void *param,uint32_t offset,uint32_t data)
{
 stm32f0_syscfg	*sc=param;
 memory_block	*vector_mb;

 vector_mb=memory_get_block(sc->sc_ml,0x00000000,0);

 if ( offset==0 && vector_mb != NULL )
  {	
	sc->sc_cfgr1=data;

	if ( (data & 3) == 3 )
		map_vector_table(sc->sc_ml,vector_mb,0x20000000);
	else if ( ! ( data & 1 ) )
		map_vector_table(sc->sc_ml,vector_mb,0x08000000);

	return(0);
  }
 else
	return(0);
}

int stm32f0_rcc_ldr(void *param,uint32_t offset,uint32_t *data)
{
 stm32f0_rcc	*rcc=param;
 uint32_t	ret;

 if ( offset==0 )
	ret=rcc->rcc_cr;
 else
	ret=0;

 if ( data != NULL )
	*data=ret;

 return(0);
}

int stm32f0_rcc_str(void *param,uint32_t offset,uint32_t data)
{
 stm32f0_rcc	*rcc=param;

 if ( offset==0 )
  {	
	/* for simulations, once HSEON is set, turn on HSERDY instantly (and vice versa): */
	if ( data & (1<<16) )	data |=  (1<<17);
	else			data &= ~(1<<17);

	/* for simulations, once PLLON is set, turn on PLLRDY instantly (and vice versa): */
	if ( data & (1<<24) )	data |=  (1<<25);
	else			data &= ~(1<<25);

	rcc->rcc_cr = data;	

	return(0);
  }
 else
	return(0);
}

int debug_peripheral_strb(void *param,uint32_t offset,uint8_t data)
{
 if ( offset==0 )
  {	printf("%c",data&0x7F);
	fflush(stdout);
  }
 return(0);
}

int fstream_usart_putc(void *param,uint16_t data)
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

int filedesc_usart_putc(void *param,uint16_t data)
{
 int	*fd_ptr = param,fd;

 fd=*fd_ptr;

 write(fd,&data,1);

 return(0);
}


int fprint_usage(FILE *fw,char *argv0)
{
 fprintf(fw,"Usage:\t%s [-h|--help]\n",argv0);
 fprintf(fw,"\t[-m|--memory <base>:{<size>|0}[:<binary_file>]] [-m ...]\n");
 fprintf(fw,"\t[-v|--vector-table-address <address>]\n");
 fprintf(fw,"\t[-p|--peripheral <device/peripheral>:<key>=<value>,...\n");
 fprintf(fw,"\t[-n|--cycles <max_instruction_count>]\n");
 return(0);
}


static void peripheral_definition_free(peripheral_definition *pd)
{
 if ( pd->pd_argument != NULL )
  {	free(pd->pd_argument);
	pd->pd_argument=NULL;
  }
 free(pd);
}

static peripheral_definition *peripheral_definition_parse(char *arg)
{
 peripheral_definition	*pd;
 char			*p,*q;
 key_value		*last_kv;

 pd=list_new(peripheral_definition);
 memset(pd,0,sizeof(peripheral_definition));

 pd->pd_argument=strdup(arg);
 if ( (p=strchr(pd->pd_argument,':'))==NULL )
  {	peripheral_definition_free(pd);
	return(NULL);
  }
 *p=0;
 arg=p+1;
 if ( (q=strchr(pd->pd_argument,'/'))==NULL )
  {	peripheral_definition_free(pd);
	return(NULL);
  }
 *q=0;
 q++;
 pd->pd_device=pd->pd_argument;
 pd->pd_peripheral=q;
 pd->pd_list_kv=NULL;
 last_kv=NULL;

 for ( p=arg; p != NULL; )
  {	char		*p_next,*q;
	key_value	*kv;
	p_next=strchr(p,',');
	if ( p_next != NULL )
	 {	*p_next=0;
		p_next++;
	 }
	q=strchr(p,'=');
	if ( q != NULL )
	 {	*q=0;
		q++;
	 }
	kv=dlist_new(key_value);
	kv->kv_key=p;
	kv->kv_value=q;
	dlist_insert_last(pd->pd_list_kv,last_kv,kv);
	p=p_next;
  }
 
 return(pd);
}

int main(int argc,char *argv[])
{
 int		i;
 struct	timeval	start_tv,sleep_tv;
 uint64_t	cycle_cnt,instr_cnt;
 uint64_t	max_cycle;
 

 memory_layout			ml;
 armv6m_state			as;
 armv6m_exception_control	aec;
 armv6m_update_control		auc;
 stm32f0_syscfg			soc_sc;
 stm32f0_rcc			soc_rcc;
 cm0_systick			cstk;
 cm0_scb			cscb;

 peripheral_definition		*pd_first,*pd_last,*pd;

 int		hint_wfi;

 ml.ml_block_first=NULL;
 ml.progmem_mb=NULL;
 ml.ram_mb=NULL;

 max_cycle=1024;

 pd_first=NULL;
 pd_last=NULL;

 for ( i=1 ; i<argc; i++ )
  {	int	w;

	if ( strcmp(argv[i],"-h")==0 || strcmp(argv[i],"--help")==0 )
	 {	fprint_usage(stdout,argv[0]);
		return(0);
	 }
	else if ( ( strcmp(argv[i],"-m")==0 || strcmp(argv[i],"--memory")==0 ) && i<argc-1 )
	 {	uint32_t	base,size,isize;
		unsigned char	*array;
		char		*p,*type;
		
		i++;
		if ( sscanf(argv[i],"%i:%i",(int *)&base,(int *)&isize)<2 || (p=strchr(argv[i],':'))==NULL )
		 {	fprintf(stderr,"%s: error: invalid memory layout specification.\n",argv[0]);
			return(1);
		 }
		p=strchr(p+1,':');
		if ( p != NULL )	p++;

		if ( p != NULL && p[0]=='+' )
		 {	type=p;
			p=NULL;
		 }
		else if ( p != NULL )
		 {	type=strchr(p+1,':');
			if ( type != NULL )	type++;
		 }

		array=NULL;

		if ( p != NULL )
	 	 {	FILE	*fr;
			char	*filename,*e;
			int	len;

			if ( (e=strchr(p,':')) != NULL )
			 {	len=e-p;
				filename=malloc(len+1);
				memcpy(filename,p,len);
				filename[len]=0;
			 }
			else
			 {	len=0;
				filename=p;
			 }

			size=0;

			if ( (fr=fopen(filename,"rb"))==NULL )
		 	 {	fprintf(stderr,"%s: error: unable to open file '%s' for reading.\n",argv[0],p);
				return(1);
			 }
			if ( 0<len )
				free(filename);

			while ( ! feof(fr) )
			 {	int	r,blocksize;
				blocksize=4096;
				array=realloc(array,size+blocksize);
				r=fread(array+size,1,blocksize,fr);
				if ( r<=0 )
					break;
				else
					size+=r;
			 }
			fclose(fr);

		 }
		else if ( 0<isize )
		 {	size=isize;
			array=malloc(size);
			memset(array,0,size);
		 }
		else
			size=0;

		if ( 0<size )
		 {	memory_block	*mb;
			mb=memory_layout_add_block(&ml);
			mb->mb_base=base;
			mb->mb_size=size;
			mb->mb_array=array;
			if ( strcmp(type,"+progmem")==0 )
				ml.progmem_mb=mb;
			else if ( strcmp(type,"+ram")==0 )
				ml.ram_mb=mb;
		 }
			
	 }
	else if ( ( strcmp(argv[i],"-p")==0 || strcmp(argv[i],"--peripheral")==0 ) && i<argc-1 )
	 {	peripheral_definition	*pd;
		i++;
		if ( (pd=peripheral_definition_parse(argv[i]))==NULL )
		 {	fprintf(stderr,"%s: error: invalid peripheral definition syntax in '%s'.\n",argv[0],argv[i]);
			return(1);
		 }
		dlist_insert_last(pd_first,pd_last,pd);
		
	 }
	else if ( ( strcmp(argv[i],"-n")==0 || strcmp(argv[i],"--cycles")==0 ) && i<argc-1 && sscanf(argv[i+1],"%i",&w)==1 )
	 {	max_cycle=w;
		i++;
	 }
	else 
	 {	fprintf(stderr,"%s: error: invalid command line argument near '%s'.\n",argv[0],argv[i]);
		return(1);
	 }
  }

 if ( 0 )
  {	peripheral_definition *pd;
	for ( pd=pd_first; pd != NULL ; pd=pd->next )
	 {	key_value	*kv;
		fprintf(stderr,"device=%s peripheral=%s\n",pd->pd_device,pd->pd_peripheral);
		for ( kv=pd->pd_list_kv; kv != NULL ; kv=kv->next )
		 {	fprintf(stderr," -> %s='%s'\n",kv->kv_key,kv->kv_value?kv->kv_value:"<UNDEFINED>");
		 }
	 }
	return(0);
  }

 aec.aec_first=NULL;
 auc.auc_first=NULL;

 /* Cortex-M0 SysTick: */
 if ( 1 )
  {	memory_block		*mb;
	armv6m_exception	*ae;
	armv6m_update		*au;

	mb=memory_layout_add_block(&ml);
	mb->mb_base=0xE000E010;
	mb->mb_size=16;
	mb->mb_array=NULL;
	mb->mb_param=&cstk;

	cstk.cstk_csr   = 0x00000000;
	cstk.cstk_rvr   = 0x00000000;	/* should be unknown */
	cstk.cstk_cvr   = 0x00000000;	/* should be unknown */
	cstk.cstk_calib = 0x00000000;	

	cstk.cstk_clock_fmhz=48;

	mb->mb_callback_ldr=cortexm0_systick_ldr;
	mb->mb_callback_str=cortexm0_systick_str;

	ae=armv6m_exception_add(&aec);
	ae->ae_callback_exception_state=cortexm0_systick_exception_state;
	ae->ae_callback_exception_delay=cortexm0_systick_exception_delay;
	ae->ae_param=&cstk;

	au=armv6m_update_add(&auc);
	au->au_callback_update=cortexm0_systick_update_realtime;
	au->au_param=&cstk;
  }

 /* Cortex-M0 System Control Block: */
 if ( 1 )
  {	memory_block		*mb;
	armv6m_exception	*ae;

	mb=memory_layout_add_block(&ml);
	mb->mb_base=0xE000ED00;
	mb->mb_size=64;
	mb->mb_array=NULL;
	mb->mb_param=&cscb;

	cscb.cscb_cpuid = 0x410CC200;
	cscb.cscb_icsr  = 0x00000000;
	cscb.cscb_aircr = 0xFA050000;
	cscb.cscb_scr   = 0x00000000;
	cscb.cscb_ccr   = 0x00000204;
	cscb.cscb_shpr2 = 0x00000000;
	cscb.cscb_shpr3 = 0x00000000;

	mb->mb_callback_ldr=cortexm0_scb_ldr;
	mb->mb_callback_str=cortexm0_scb_str;

	ae=armv6m_exception_add(&aec);

	ae->ae_callback_exception_state=cortexm0_scb_exception_state;
	ae->ae_param=&cscb;
  }

 /* debug peripheral: */
 if ( 0 )
  {	memory_block	*mb;
	mb=memory_layout_add_block(&ml);
	mb->mb_base=0x40000000;
	mb->mb_size=4;
	mb->mb_array=NULL;
	mb->mb_callback_strb=debug_peripheral_strb;
  }

 /* STM32F0 SYSCFG, used only to switch the mapping to the vector table */
 /* (only SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE_x are implemented): */
 /* defaults to the 0x08000000 => 0x00000000 mapping. */
 if ( 1 )
  {	memory_block	*mb;

	mb=memory_layout_add_block(&ml);
	mb->mb_base=0x40010000;
	mb->mb_size=32;
	mb->mb_array=NULL;

	mb->mb_param=&soc_sc;

	soc_sc.sc_ml=&ml;
	soc_sc.sc_cfgr1=0x00000000;

	mb->mb_callback_ldr=stm32f0_syscfg_ldr;
	mb->mb_callback_str=stm32f0_syscfg_str;
  }

 /* STM32F0 RCC, used to turn on/off PLL and HSE */
 if ( 1 )
  {	memory_block	*mb;

	mb=memory_layout_add_block(&ml);
	mb->mb_base=0x40021000;
	mb->mb_size=64;
	mb->mb_array=NULL;

	mb->mb_param=&soc_rcc;

	/* HSITRIM is 16, HSIRDY and HSION is set: */
	soc_rcc.rcc_cr=0x00000083; 

	mb->mb_callback_ldr=stm32f0_rcc_ldr;
	mb->mb_callback_str=stm32f0_rcc_str;
  }

 for ( pd=pd_first; pd != NULL; pd=pd->next )
  {	/* STM32F0 USART2, simpified version for sending and receiving data: */
	if ( strcmp(pd->pd_device,"stm32f0")==0 && strcmp(pd->pd_peripheral,"usart")==0 )
 	 {	stm32f0_usart_register(&ml,&aec,&auc,pd,&cstk.cstk_clock_fmhz);
	 }
  }

 /* the 0x00000000 region: */
 if ( 1 )
  {	memory_block	*vector_mb;

	vector_mb=memory_layout_add_block(&ml);
	vector_mb->mb_base=0x00000000;
	map_vector_table(&ml,vector_mb,0x08000000);
  }
	
 sig_int=0;
 signal(SIGINT,sig_int_handler);

 armv6m_core_reset(&ml,&as);

 armv6m_core_optimize_conditions(&as);

/* fprintf(stderr,"PC=0x%.8x SP=0x%.8x\n",as.as_reg[REG_PC],as.as_reg[REG_SP]); */

 gettimeofday(&start_tv,NULL);
 sleep_tv.tv_sec=0;
 sleep_tv.tv_usec=0;

 hint_wfi=0;

 for ( instr_cnt=cycle_cnt=0; max_cycle<=0 || cycle_cnt<max_cycle; cycle_cnt++ )
  {	armv6m_hint	ah;

	if ( cycle_cnt%64==0 )
 	 {	armv6m_update *au;
		for ( au=auc.auc_first ; au != NULL ; au=au->next )
		 {	au->au_callback_update(au->au_param);
		 }
	 }
	

	if ( !(as.as_primask & (1<<PRIMASK_PRIMASK)) && armv6m_is_mode_thread(&as) )
	 {	int	vector;

		vector=armv6m_exception_check(&ml,&as,&aec);
		if ( vector )
		 {	armv6m_core_exception_enter(&ml,&as,vector);
			hint_wfi=0;
		 }

	 }
	
	/* avmv6m_fprint_state(stderr,&ml,&as,0); */

	/* processor is running normally: */
	if ( ! hint_wfi )
	 {	armv6m_core_step(&ml,&as,&ah);
		if ( ah.ah_hint == STEP_HINT_WFI )
			hint_wfi=1;
		instr_cnt++;
	 }

	/* processor is in the `wait for interrupt` (WFI) state: */
	else 
	 {	int		min_time,fd_max;
		fd_set		set;
		armv6m_update	*au;
		
		/* first, update exceptions... not neccessary but can speed up the simulations under heavier loads: */
		for ( au=auc.auc_first ; au != NULL ; au=au->next )
		 {	au->au_callback_update(au->au_param);
		 }

		min_time=armv6m_exception_delay(&ml,&as,&aec);
		fd_max=armv6m_exception_fdset(&ml,&as,&aec,&set);

		/* fprintf(stderr,"min_time=%d fd_max=%d\n",min_time,fd_max); */

		if ( 0<=fd_max )
		 {	struct	timeval	beg_tv,end_tv;
			int		r;

			gettimeofday(&beg_tv,NULL);

			if ( 0<=min_time )
			 {	struct	timeval	tv;
				timeval_usec(&tv,min_time);
				r=select(fd_max+1,&set,NULL,NULL,&tv);
			 }
			else
			 {	r=select(fd_max+1,&set,NULL,NULL,NULL);
			 }

			gettimeofday(&end_tv,NULL);

			timeval_sub(&end_tv,&beg_tv);
			timeval_add(&sleep_tv,&end_tv);

			if ( r<0 && errno==EINTR )
				sig_int=!0;
		 }
		else if ( 0<min_time )
		 {	usleep(min_time);
			timeval_increment(&sleep_tv,min_time);
		 }
		else if ( min_time<0 )
		 {	fprintf(stderr,"# ARMv6-M: no interrupt sources are available while in WFI state.\n");
			fprintf(stderr,"# ARMv6-M: system is frozen, simulation has been aborted.\n");
			sig_int=!0;
		 }
	 }

	if ( sig_int )
		break;

  }

 if ( 1 )
  {	struct timeval	tv;
	double		diff,mccnt,micnt,mcps,mips,dsleep;

	gettimeofday(&tv,NULL);
	diff=timeval_diff(&start_tv,&tv);
	dsleep=timeval_diff(NULL,&sleep_tv);
	fprintf(stderr,"# ARMv6-M state:\n");
	fprintf(stderr,"state = %s\n",hint_wfi?"idle (WFI)":"running");
	avmv6m_core_fprint_state(stderr,&ml,&as,1);
	fprintf(stderr,"# wall time elapsed: %.3f seconds\n",diff);
	fprintf(stderr,"# total sleep time : %.3f seconds\n",dsleep);
	fprintf(stderr,"# total cycles: %" PRIu64 "/%" PRIu64 "\n",instr_cnt,cycle_cnt);
	mccnt=(double)cycle_cnt/1000000.0;
	micnt=(double)instr_cnt/1000000.0;
	mcps=mccnt/diff;
	mips=micnt/diff;
	fprintf(stderr,"# simulation speed: %.3f/%.3f MIPS/MCPS\n",mips,mcps);
  }

 return(0);
}
