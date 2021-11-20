/*****************************************************************************/
/* hprintf.c								     */
/*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>

#include "hprintf.h"

/*****************************************************************************/

#define	HPRINT_BUFFER_SIZE		256

/*****************************************************************************/

int vstrappendf(char **str,char *fmt,va_list ap)
{
 int		n,l,size;

 if ( str==NULL )	return(0);

 if ( *str==NULL )	l=0;
 else			l=strlen(*str);
 size=128;

 *str=realloc(*str,l+size);
 if ( *str==NULL )	return(-1);
 while ( 1 )
  {	n=vsnprintf((*str)+l,size,fmt,ap);
	if ( n>-1 && n<size )
		return(0);
	else if ( n>-1 )
		size=n+1;	
	else
		size=size*2;	
	if ( (*str=realloc(*str,l+size))==NULL )
		return(-1);
  };
 return(0);	
}

/*****************************************************************************/

int vhwprintf(int handle,int wtime,char *msg,va_list ap)
{
 char	buff[HPRINT_BUFFER_SIZE],*tbuff;
 int	n,r;

 n=vsnprintf(buff,HPRINT_BUFFER_SIZE,msg,ap);
 if ( n<HPRINT_BUFFER_SIZE )
  {	if ( wtime <= 0 )
		r=write(handle,buff,n);
	else
	 {	int	i;
		for ( i=0 ; i<n ; i++ )
		 {	r=write(handle,&buff[i],1);
			usleep(wtime);
		 }
	 }
	return(n);
  }
 else
  {	tbuff=NULL;
	vstrappendf(&tbuff,msg,ap);
	if ( tbuff != NULL )
	 {	n=strlen(tbuff);
		if ( wtime <= 0 )
			r=write(handle,tbuff,n);
		else
		 {	int	i;
			for ( i=0 ; i<n ; i++ )
			 {	r=write(handle,&tbuff[i],1);
				usleep(wtime);
			 }
		 }
		(void)r;
		free(tbuff);
	 }
	else
		n=0;

	return(n);
  }

}

int hprintf(int handle,char *msg,...)
{
 int		r;
 va_list	ap;

 va_start(ap,msg);
 r=vhwprintf(handle,0,msg,ap);
 va_end(ap);
 return(r);
}

/*****************************************************************************/
                                                            
                          

