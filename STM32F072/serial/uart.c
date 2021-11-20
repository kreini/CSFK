/*****************************************************************************/
/* uart.c								     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Simple UART configuration library					     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* (c) 2002, 2009; Pal, A. (apal@szofi.net)				     */
/*****************************************************************************/

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <memory.h>
#include <stdlib.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include "uart.h"

/*****************************************************************************/

static speed_t uart_get_baud_rate_speed(int baud)
{
 speed_t	bspeed;

 switch ( baud )
  {	case    300:	bspeed=   B300;break;
 	case    600:	bspeed=   B600;break;
 	case   1200:	bspeed=  B1200;break;
  	case   2400:	bspeed=  B2400;break;
  	case   4800:	bspeed=  B4800;break;
  	case   9600:	bspeed=  B9600;break;
	case  19200:	bspeed= B19200;break;
	case  38400:	bspeed= B38400;break;
	case  57600:	bspeed= B57600;break;
	case 115200:	bspeed=B115200;break;
#ifdef  B230400
	case 230400:	bspeed=B230400;break;
#endif
#ifdef  B460800
	case 460800:	bspeed=B460800;break;
#endif
#ifdef  B921600
	case 921600:	bspeed=B921600;break;
#endif
#ifdef	B1000000
	case 1000000:	bspeed=B1000000;break;
#endif
#ifdef	B1152000
	case 1152000:	bspeed=B1152000;break;
#endif
#ifdef	B1500000
	case 1500000:	bspeed=B1500000;break;
#endif
#ifdef	B2000000
	case 2000000:	bspeed=B2000000;break;
#endif
#ifdef	B2500000
	case 2500000:	bspeed=B2500000;break;
#endif
#ifdef	B3000000
	case 3000000:	bspeed=B3000000;break;
#endif
	default:	bspeed=0;
  }
 return(bspeed);
}

int uart_is_supported_baud(int baud)
{
 if ( 0<uart_get_baud_rate_speed(baud) )
	return(1);
 else
	return(0);
}

int uart_configure(int handle,int baud,int bits,int parity,int is_two_stop)
{
 struct termios	newtio;
 speed_t	bspeed;

 memset(&newtio,0,sizeof(newtio));

 bspeed=uart_get_baud_rate_speed(baud);

 cfsetispeed(&newtio,bspeed);
 cfsetospeed(&newtio,bspeed);
 switch ( bits )
  {	case 5:	newtio.c_cflag |= CS5;break;
	case 6:	newtio.c_cflag |= CS6;break;
	case 7:	newtio.c_cflag |= CS7;break;
	case 8:	newtio.c_cflag |= CS8;break;
	default:	return(-1);
  }
 switch ( parity )
  {  case 'O': case 'o': case 1:
	newtio.c_cflag |= PARODD|PARENB;
	newtio.c_iflag |= INPCK;
	break;
     case 'E': case 'e': case 2:
	newtio.c_cflag &= (~PARODD);
	newtio.c_cflag |= PARENB;
	newtio.c_iflag |= INPCK;
	break;
     case 'N': case 'n': case 0:
	break;
     default:
	return(-1);
  }

 if ( is_two_stop )
	newtio.c_cflag |=  CSTOPB;
 else
	newtio.c_cflag &= ~CSTOPB;

 newtio.c_cflag |= CREAD | HUPCL | CLOCAL;
 newtio.c_iflag &= ~( IGNPAR | PARMRK | INLCR | IGNCR | ICRNL | ISTRIP );
 newtio.c_iflag |= BRKINT;

 newtio.c_oflag = 0;
 newtio.c_lflag = 0;
 newtio.c_cc[VMIN ]=1;
 newtio.c_cc[VTIME]=0;

 if ( tcsetattr(handle,TCSANOW,&newtio)<0 )
	return(-1);
 else if ( tcflush(handle,TCIOFLUSH)<0 )
	return(-1);
 else
	return(0);
}

int uart_wait_write(int handle)
{
 return(tcdrain(handle));
}

int uart_flush_read(int handle)
{
 return(tcflush(handle,TCIFLUSH));
}
int uart_flush_write(int handle)
{
 return(tcflush(handle,TCOFLUSH));
}
int uart_flush(int handle)
{
 return(tcflush(handle,TCIOFLUSH));
}

char * uart_read_buffer(int handle,int timeout,
int (*callback)(char *buffer,size_t length,void *param),void *param)
{
 fd_set		set;
 struct	timeval	tv,t0,t1;
 char		*ret;
 size_t		len;

 gettimeofday(&t0,NULL);

 ret=NULL;
 len=0;

 while ( 1 )
  {	FD_ZERO(&set);
	FD_SET(handle,&set);
	if ( timeout>0 )
	 {	gettimeofday(&t1,NULL);
		if ( t1.tv_usec<t0.tv_usec )
		 {	t1.tv_usec+=1000000;
			t1.tv_sec--;
		 }
		tv.tv_sec =t1.tv_sec -t0.tv_sec ;
		tv.tv_usec=t1.tv_usec-t0.tv_usec;
		if ( tv.tv_sec >= timeout )
		 {	if ( ret != NULL )
				free(ret);
			return(NULL);
		 }
		tv.tv_sec=timeout-tv.tv_sec;
		if ( tv.tv_usec>0 )
		 {	tv.tv_usec=1000000-tv.tv_usec;
			tv.tv_sec--;
		 }
		select(handle+1,&set,NULL,NULL,&tv);
	 }
	else
		select(handle+1,&set,NULL,NULL,NULL);

	if ( FD_ISSET(handle,&set) )
	 {	char	c;
		read(handle,(void *)(&c),1);
		ret=(char *)realloc(ret,len+1);
		ret[len]=c;
		len++;
		if ( callback(ret,len,param) )
			return(ret);
	 }
	else if ( timeout>0 )
	 {	if ( ret != NULL )
			free(ret);
		return(NULL);
	 }
  };

 return(NULL); /* unreachable */
}


int uart_control(int handle,int arg)
{
 int w;

 if ( ioctl(handle,TIOCMGET,&w) )
	return(-1);

 if ( (arg&(~3)) != 0x10 )
  {	if ( arg & 0x01 )	w|= TIOCM_DTR;
	else			w&=~TIOCM_DTR;
	if ( arg & 0x02 )	w|= TIOCM_RTS;
	else			w&=~TIOCM_RTS;
  }
 else
  {	switch (arg) 
	 {	case 0x10:	w|= TIOCM_DTR;break;
		case 0x11:	w&=~TIOCM_DTR;break;
		case 0x12:	w|= TIOCM_RTS;break;
		case 0x13:	w&=~TIOCM_RTS;break;
	 }
  }

 if ( ioctl(handle,TIOCMSET,&w) )
	return(-1);

 return(0);
}

/*****************************************************************************/
     
