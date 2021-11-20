#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/time.h>

#include "list.h"
#include "timeval.h"

double timeval_diff(struct timeval *t0,struct timeval *t1)
{
 struct	timeval	tv;
 double		diff;

 if ( t0 != NULL )
  {	tv.tv_sec=t1->tv_sec-t0->tv_sec;
	tv.tv_usec=t1->tv_usec-t0->tv_usec;
  }
 else
	tv=(*t1);

 if ( tv.tv_usec<0 )	
  {	tv.tv_usec+=1000000;
	tv.tv_sec--;
  }
 
 diff=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000.0;

 return(diff);
}

int timeval_increment(struct timeval *tv,int usec)
{
 tv->tv_usec += usec;
 tv->tv_sec += tv->tv_usec/1000000;
 tv->tv_usec = tv->tv_usec%1000000;
 return(0);
}

int timeval_usec(struct timeval *tv,int usec)
{
 tv->tv_sec=usec/1000000;
 tv->tv_usec=usec%1000000;
 return(0);
}

int timeval_normalize(struct timeval *tv)
{
 while ( tv->tv_usec < 0 )
  {	tv->tv_usec += 1000000;
	tv->tv_sec --;
  }
 while ( 1000000<=tv->tv_usec )
  {	tv->tv_usec -= 1000000;
	tv->tv_sec ++;
  }
 return(0);
}

int timeval_sub(struct timeval *t0,struct timeval *t1)
{
 t0->tv_sec  -= t1->tv_sec;
 t0->tv_usec -= t1->tv_usec;
 if ( t0->tv_usec<0 )
  {	t0->tv_usec += 1000000;
	t0->tv_sec --;
  }
 return(0);
}

int timeval_add(struct timeval *t0,struct timeval *t1)
{
 t0->tv_sec  += t1->tv_sec;
 t0->tv_usec += t1->tv_usec;
 if ( 1000000<=t0->tv_usec )
  {	t0->tv_usec -= 1000000;
	t0->tv_sec ++;
  }
 return(0);
}

