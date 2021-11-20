/*****************************************************************************/
/* serial.c								     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Simple interface to serial UARTs					     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* (c) 2002, 2009; Pal, A. (apal@szofi.net)				     */
/*****************************************************************************/

#define			SERIAL_VERSION	"0.4"
#define			SERIAL_RELEASE	"2015.03.16"

/*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "tokenize.h"
#include "uart.h"
#include "hprintf.h"
#include "list.h"

struct serial_config
 {	int	sc_is_hexa;
	int	sc_is_timestamp;
	int	sc_bits;
	int	sc_parity;
	int	sc_baudrate;
	int	sc_nwltype;
	int	sc_uwait;
	int	sc_selective_rts;
	int	sc_skip_config;
	int	sc_bbline;
	int	sc_ignore_transmit;
 };

struct serial_client
 {	struct	serial_client	*next,*prev;
	int	peer;
 };

int fprint_usage(FILE *fw)
{
 fprintf(fw,
"Usage:\tserial [-h|--help] [-v|--version]\n"
"Basic configuration:\n"
"\t[-p|--port <port>|-c|--device <device>] [-k|--skip-config] [-[5678]]\n"
"\t[-e|--parity {N|E|O}|--none|--even|--odd] [-b|--baud <baudrate>] \n"
"Delaying:\n"
"\t[-w|--wait <microseconds>]\n"
"Newline conversions:\n"
"\t[-d|--dos] [-u|--unix]\n");
 fprintf(fw,
"Hex dumping and timestamping:\n"
"\t[-x|--hex [-t|--timestamp] [-y|--block-bytes <line_size>]]\n"
"TCP/IP forwarding:\n"
"\t[-l|--listen <port> [--no-transmit]]\n");
 return(0);
}

int fprint_version(FILE *fw)
{
 fprintf(fw,"serial %s (%s)\n",SERIAL_VERSION,SERIAL_RELEASE);
 fprintf(fw,"Copyright (C) 2002, 2009, 2014-2015; Pal, Andras <apal@szofi.net>\n");
 return(0);
}

int serial_main_loop(int fd,int inhandle,int outhandle,int serv,struct serial_config *sc)
{
 int		m,r;
 unsigned char	buff[256];
 struct serial_client	*clients,*c;

 clients=NULL;

 while ( 1 )
  {	
	fd_set	set;

	FD_ZERO(&set);
	m=fd;
	FD_SET(fd,&set);

	if ( 0<=inhandle )
	 {	FD_SET(inhandle,&set);
		if ( m<inhandle )	m=inhandle;
	 }
	if ( 0<=serv )
	 {	FD_SET(serv,&set);
		if ( m<serv )		m=serv;
	 }
	for ( c=clients ; c != NULL ; c=c->next )
	 {	FD_SET(c->peer,&set);
		if ( m<c->peer )	m=c->peer;
	 }

	select(m+1,&set,NULL,NULL,NULL);

	if ( 0<=serv && FD_ISSET(serv,&set) )
	 {	c=list_new(struct serial_client);
		list_insert_first(clients,c);
		c->peer=accept(serv,NULL,NULL);
		continue;
	 }

	for ( c=clients ; c != NULL ; c=c->next )
	 {	if ( ! FD_ISSET(c->peer,&set) )	
			continue;
		else if ( c->peer < 0 )
			continue;
		r=read(c->peer,buff,256);
		if ( r <= 0 )
		 {	close(c->peer);
			c->peer=-1;
			continue;
		 }
		if ( sc->sc_ignore_transmit )
			continue;

		if ( sc->sc_nwltype )
		 {	int	i;
			for ( i=0 ; i<r ; i++ )
			 {	if ( buff[i]==0x0a )	buff[i]=0x0d;
			 }
		 }
		if ( sc->sc_uwait<=0 )
		 {	/* fprintf(stderr,"main(): write: start.\n");  */
			write(fd,buff,r);
			tcdrain(fd);
			/* fprintf(stderr,"main(): write: stop.\n");   */
		 }
		else
		 {	int	i;
			if ( sc->sc_selective_rts && ! sc->sc_skip_config )
				uart_control(fd,0x02);
			for ( i=0 ; i<r ; i++ )
			 {	if ( buff[i]==0x0a )	continue;
				write(fd,&buff[i],1);
				usleep(sc->sc_uwait);
			 }
			if ( sc->sc_selective_rts && ! sc->sc_skip_config  )
				uart_control(fd,0x00);
		 }
	 }

	for ( c=clients ; c != NULL ; )
	 {	if ( c->peer<0 )
		 {	struct	serial_client	*n;
			n=c->next;
			list_delete(clients,c);
			c=n;
		 }
		else
			c=c->next;
	 }
	
	if ( 0<=inhandle && FD_ISSET(inhandle,&set) && !(sc->sc_is_hexa) )
	 {	r=read(inhandle,buff,256);
		if ( r <= 0 )	break;
		if ( sc->sc_nwltype )
		 {	int	i;
			for ( i=0 ; i<r ; i++ )
			 {	if ( buff[i]==0x0a )	buff[i]=0x0d;
			 }
		 }
		if ( sc->sc_uwait<=0 )
		 {	/* fprintf(stderr,"main(): write: start.\n");  */
			write(fd,buff,r);
			tcdrain(fd);
			/* fprintf(stderr,"main(): write: stop.\n");   */
		 }
		else
		 {	int	i;
			if ( sc->sc_selective_rts && ! sc->sc_skip_config )
				uart_control(fd,0x02);
			for ( i=0 ; i<r ; i++ )
			 {	if ( buff[i]==0x0a )	continue;
				write(fd,&buff[i],1);
				usleep(sc->sc_uwait);
			 }
			if ( sc->sc_selective_rts && ! sc->sc_skip_config  )
				uart_control(fd,0x00);
		 }
	 }
	else if ( 0<=inhandle && FD_ISSET(inhandle,&set) && sc->sc_is_hexa )
	 {	char	buff[1024],**cmd;
		int	i,c;
		if ( fgets(buff,1024,stdin)==NULL )
			break;
		cmd=tokenize_spaces_dyn(buff);
		if ( sc->sc_selective_rts &&  ! sc->sc_skip_config )
			uart_control(fd,0x02);
		for ( i=0 ; cmd != NULL && cmd[i] != NULL ; i++ )
		 {	if ( cmd[i][0]==':' )
			 {	int	j;
				tcdrain(fd);
				for ( j=1 ; cmd[i][j] ; j++ )
				 {	switch ( cmd[i][j] )
					 {   case 'n': case 'N':
						sc->sc_parity='N';
						break;
					     case 'o': case 'O':
						sc->sc_parity='O';
						break;
					     case 'e': case 'E':
						sc->sc_parity='E';
						break;
					 }
				 }
				if ( ! sc->sc_skip_config )
					uart_configure(fd,sc->sc_baudrate,sc->sc_bits,sc->sc_parity,!0);
			 }
			else if ( sscanf(cmd[i],"%i",&c)==1 && 0<=c && c<256 )
			 {	
				write(fd,&c,1);
				tcdrain(fd);
				fprintf(stderr,"# sent: 0x%.2X\n",c);
				usleep(sc->sc_uwait);
			 }
		 }
		if ( sc->sc_selective_rts && ! sc->sc_skip_config )
			uart_control(fd,0x00);
		if ( cmd != NULL )	free(cmd);
	 }
	else if ( FD_ISSET(fd,&set) )
	 {	r=read(fd,buff,256);
		if ( r>0 && ! sc->sc_is_hexa )
		 {	if ( sc->sc_nwltype )
			 {	int	i;
				for ( i=0 ; i<r ; i++ )
				 {	if ( buff[i]==0x0d )	buff[i]=0x0a;
				 }
			 }
			if ( 0<=outhandle )
				write(outhandle,buff,r);
			for ( c=clients ; c != NULL ; c=c->next )
				write(c->peer,buff,r);
		 }
		else if ( r>0 )
		 {	int	i;
			for ( i=0 ; i<r ; i++ )
		 	 {	if ( i%sc->sc_bbline==0 )
				 {	if ( ! sc->sc_is_timestamp )
					 {	if ( 0<=outhandle )
							hprintf(outhandle,">");
						for ( c=clients ; c != NULL ; c=c->next )
							hprintf(c->peer,">");
					 }		
					else
					 {	struct	timeval	tv;
						gettimeofday(&tv,NULL);
						if ( 0<=outhandle )
							hprintf(outhandle,"%d.%.6d >",(int)tv.tv_sec,(int)tv.tv_usec);
						for ( c=clients ; c != NULL ; c=c->next )
							hprintf(c->peer,"%d.%.6d >",(int)tv.tv_sec,(int)tv.tv_usec);
					 }							
				 }
				if ( 0<=outhandle )
				 {	hprintf(outhandle," 0x%.2x",buff[i]);
					if ( (i+1)%sc->sc_bbline==0 )	hprintf(outhandle,"\n");
				 }
				for ( c=clients ; c != NULL ; c=c->next )
				 {	hprintf(c->peer," 0x%.2x",buff[i]);
					if ( (i+1)%sc->sc_bbline==0 )	hprintf(c->peer,"\n");
				 }
			 }
			if ( r%sc->sc_bbline )
			 {	if ( 0<=outhandle )
					hprintf(outhandle,"\n");
				for ( c=clients ; c != NULL ; c=c->next )
					hprintf(c->peer,"\n");
			 }
		 }
	 }
  };

 return(0);
}

int main(int argc,char *argv[])
{
 int			i,r;
 int			port,fd,inhandle,outhandle,servport,serv;
 char			*device,dev_static[16];
 int			set_dtr,set_rts;
 struct	stat		st;

 struct serial_config	sc;

 port=1;
 servport=0;

 sc.sc_baudrate=9600;
 sc.sc_bits=8;
 sc.sc_parity='N';
 sc.sc_nwltype=0;
 sc.sc_ignore_transmit=0;

 sc.sc_selective_rts=0;
 sc.sc_skip_config=0;

 sc.sc_uwait=0;
 sc.sc_is_hexa=0;
 sc.sc_bbline=1;

 device=NULL;
 set_dtr=set_rts=0;
 
 for ( i=1 ; i<argc ; i++ )
  {	if ( strcmp(argv[i],"-h")==0 || strcmp(argv[i],"--help")==0 )
	 {	fprint_usage(stdout);
		return(0);
	 }
	else if ( strcmp(argv[i],"-v")==0 || strcmp(argv[i],"--version")==0 )
	 {	fprint_version(stdout);
		return(0);
	 }
	else if ( strcmp(argv[i],"-k")==0 || strcmp(argv[i],"--skip-config")==0 )
		sc.sc_skip_config=1;
	else if ( ( strcmp(argv[i],"-p")==0 || strcmp(argv[i],"--port")==0 ) && i<argc-1 && sscanf(argv[i+1],"%d",&r)==1 && 1<=r && r<=8 )
	 {	i++;
		port=r;
	 }
	else if ( ( strcmp(argv[i],"-l")==0 || strcmp(argv[i],"--listen")==0 ) && i<argc-1 && sscanf(argv[i+1],"%d",&r)==1 && 0<r )
	 {	i++;
		servport=r;
	 }
	else if ( strcmp(argv[i],"--no-transmit")==0 )
		sc.sc_ignore_transmit=1;
	else if ( ( strcmp(argv[i],"-c")==0 || strcmp(argv[i],"--device")==0 ) && i<argc-1 )
	 {	i++;
		device=argv[i];
	 }
	else if ( ( strcmp(argv[i],"--dtr")==0 ) )
		set_dtr=1;
	else if ( ( strcmp(argv[i],"--rts")==0 ) )
		set_rts=1;
	else if ( ( strcmp(argv[i],"--selective-rts")==0 ) )
		sc.sc_selective_rts=1;
	else if ( ( strcmp(argv[i],"-e")==0 || strcmp(argv[i],"--parity")==0 ) && i<argc-1 )
	 {	i++;
		if ( strcmp(argv[i],"N")==0 || strcmp(argv[i],"n")==0 || strcmp(argv[i],"0")==0 )	
			sc.sc_parity='N';
		else if ( strcmp(argv[i],"E")==0 || strcmp(argv[i],"e")==0 || strcmp(argv[i],"2")==0 )
			sc.sc_parity='E';
		else if ( strcmp(argv[i],"O")==0 || strcmp(argv[i],"o")==0 || strcmp(argv[i],"1")==0 )
			sc.sc_parity='O';
		else
		 {	fprintf(stderr,"%s: error: unexpected parity '%s'.\n",argv[0],argv[i]);
			return(1);
		 }
	 }
	else if ( ( strcmp(argv[i],"--none")==0 ) )
		sc.sc_parity='N';
	else if ( ( strcmp(argv[i],"--even")==0 ) )
		sc.sc_parity='E';
	else if ( ( strcmp(argv[i],"--odd")==0 ) )
		sc.sc_parity='O';
	else if ( strcmp(argv[i],"-8")==0 || strcmp(argv[i],"--bits-8")==0 )
		sc.sc_bits=8;
	else if ( strcmp(argv[i],"-7")==0 || strcmp(argv[i],"--bits-7")==0 )
		sc.sc_bits=7;
	else if ( strcmp(argv[i],"-6")==0 || strcmp(argv[i],"--bits-6")==0 )
		sc.sc_bits=6;
	else if ( strcmp(argv[i],"-5")==0 || strcmp(argv[i],"--bits-5")==0 )
		sc.sc_bits=5;
	else if ( ( strcmp(argv[i],"-y")==0 || strcmp(argv[i],"--block-bytes")==0 ) && i<argc-1 && sscanf(argv[i+1],"%d",&r)==1 )
	 {	i++;
		sc.sc_bbline=r;
	 }
	else if ( ( strcmp(argv[i],"-w")==0 || strcmp(argv[i],"--wait")==0 ) && i<argc-1 && sscanf(argv[i+1],"%d",&r)==1 )
	 {	i++;
		sc.sc_uwait=r;
	 }
	else if ( ( strcmp(argv[i],"-b")==0 || strcmp(argv[i],"--baudrate")==0 || strcmp(argv[i],"--baud")==0 ) && i<argc-1 && sscanf(argv[i+1],"%d",&r)==1 )
	 {	i++;
		sc.sc_baudrate=r;
		if ( ! uart_is_supported_baud(sc.sc_baudrate) || sc.sc_baudrate <= 0 )
		 {	fprintf(stderr,"%s: error: baud rate %d is not supported on this system at all.\n",argv[0],sc.sc_baudrate);
			return(1);	
		 }
	 }
	else if ( strcmp(argv[i],"-d")==0 || strcmp(argv[i],"--dos")==0 )
		sc.sc_nwltype=1;
	else if ( strcmp(argv[i],"-u")==0 || strcmp(argv[i],"--unix")==0 )
		sc.sc_nwltype=0;
	else if ( strcmp(argv[i],"-x")==0 || strcmp(argv[i],"--hex")==0 || strcmp(argv[i],"--hexadecimal")==0 )
		sc.sc_is_hexa=1;
	else if ( strcmp(argv[i],"-t")==0 || strcmp(argv[i],"--timestamp")==0 )
		sc.sc_is_timestamp=1;
	else
	 {	fprintf(stderr,"%s: error: invalid command line argument near '%s'.\n",argv[0],argv[i]);
		return(1);
	 }
  }

 if ( device==NULL )
  {	device=dev_static;
	snprintf(device,sizeof(dev_static),"/dev/ttyS%d",port-1);
  }

 if ( stat(device,&st) )
  {	fprintf(stderr,"%s: error: unable to stat control device '%s'.\n",argv[0],device);
	return(1);
  }
 else if ( ! (st.st_mode & S_IFCHR) )
  {	fprintf(stderr,"%s: error: control device '%s' does not seem to be a character device.\n",argv[0],device);
	return(1);
  }
 else if ( (fd=open(device,O_RDWR))<0 )
  {	fprintf(stderr,"%s: error: unable to open/access control device '%s'.\n",argv[0],device);
	return(1);
  }
 else if ( (!sc.sc_skip_config) && uart_configure(fd,sc.sc_baudrate,sc.sc_bits,sc.sc_parity,!0) )
  { 	fprintf(stderr,"%s: error: unable to initialize control device UART (at %d baud).\n",argv[0],sc.sc_baudrate);
	return(1);
  }

 if ( ! sc.sc_skip_config )
	uart_control(fd,(set_rts?0x02:0x00)|(set_dtr?0x01:0x00));

 if ( 0<servport )
  {	struct	sockaddr_in	sin;
	int			i;

	if ( (serv=socket(AF_INET,SOCK_STREAM,0))<0 )
	 {	fprintf(stderr,"%s: error: unable to create server socket: %s.\n",argv[0],strerror(errno));
		return(1);
	 }
	i=1;
	setsockopt(serv,SOL_SOCKET,SO_REUSEADDR,(void *)&i,sizeof(int));
	sin.sin_family=AF_INET;
	sin.sin_port=htons(servport);
	sin.sin_addr.s_addr=INADDR_ANY;
	if ( bind(serv,(struct sockaddr *)(&sin),sizeof(struct sockaddr_in))<0 )
	 {	fprintf(stderr,"%s: error: unable to bind server socket to port %d: %s.\n",argv[0],servport,strerror(errno));
		return(1);
	 }
	else if ( listen(serv,256)<0 )
	 {	fprintf(stderr,"%s: error: unable to put server socket to listen state: %s.\n",argv[0],strerror(errno));
		return(1);
	 }
  }
 else
	serv=-1;

 signal(SIGPIPE,SIG_IGN);

 if ( serv<0 )
	inhandle=fileno(stdin),outhandle=fileno(stdout);
 else
 	inhandle=outhandle=-1;

 serial_main_loop(fd,inhandle,outhandle,serv,&sc);

 close(fd);
 if ( 0<=serv )
	close(serv);

 return(0);
}

/*****************************************************************************/
                                                              
