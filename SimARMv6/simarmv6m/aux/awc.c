#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint32_t armv6m_add_with_carry_32(uint32_t ua,uint32_t ub,int c,int *flags)
{
 uint32_t	uresult;
 int32_t	iresult;
 int32_t	ia,ib;
 uint64_t	uresult64;
 int64_t	iresult64;

 ia=ua;
 ib=ub;

 uresult=ua+ub+(c?1:0);
 iresult=(int32_t)uresult;

 uresult64=(uint64_t)ua+(uint64_t)ub+(c?1:0);
 iresult64=( int64_t)ia+( int64_t)ib+(c?1:0);

 *flags=0;
 if ( uresult64 != (uint64_t)uresult )	*flags |= 2;
 if ( iresult64 != (int64_t)iresult  )	*flags |= 1;

 return(uresult);
}

uint8_t armv6m_add_with_carry_8(uint8_t ua,uint8_t ub,int c,int *flags)
{
 uint8_t	uresult;
 int8_t		iresult;
 int8_t		ia,ib;
 uint16_t	uresult16;
 int16_t	iresult16;


 ia=ua;
 ib=ub;

 uresult=ua+ub+(c?1:0);
 iresult=(int8_t)uresult;

 uresult16=(uint16_t)ua+(uint16_t)ub+(c?1:0);
 iresult16=( int16_t)ia+( int16_t)ib+(c?1:0);

 *flags=0;

 if ( uresult16 != (uint16_t)uresult )	*flags |= 2;
 if ( iresult16 != (int16_t)iresult )	*flags |= 1;

 if ( ! (ua&128) && ! (ub&128) &&   (uresult&128) )	*flags |= 4;
 if (   (ua&128) &&   (ub&128) && ! (uresult&128) )	*flags |= 4;

 return(uresult);
}

int main(int argc,char *argv[])
{
 FILE	*fr,*fw;

 fr=stdin;
 fw=stdout;

 while ( ! feof(fr) )
  {	char		buff[1024];
	uint32_t	a,b,c;
	uint8_t		r8;
	int		flags;

	if ( fgets(buff,1024,fr)==NULL )
		break;
	if ( sscanf(buff,"%i %i %i\n",(int *)&a,(int *)&b,(int *)&c)<3 )
		break;

	r8=armv6m_add_with_carry_8(a,b,(int)c,&flags);
	fprintf(fw,"%3d %3d %d %3d %d %d %d\n",a,b,c,r8,(flags&2?1:0),flags&1,(flags&4)?1:0);
  }

 return(0);
}
