/*****************************************************************************/

#ifndef	__TIMEVAL_H_INCLUDED
#define	__TIMEVAL_H_INCLUDED	1

/*****************************************************************************/

#include <sys/types.h>
#include <sys/time.h>

double	timeval_diff(struct timeval *t0,struct timeval *t1);
int	timeval_increment(struct timeval *tv,int usec);
int	timeval_usec(struct timeval *tv,int usec);
int	timeval_normalize(struct timeval *tv);
int	timeval_sub(struct timeval *t0,struct timeval *t1);
int	timeval_add(struct timeval *t0,struct timeval *t1);

/*****************************************************************************/

#endif

/*****************************************************************************/
