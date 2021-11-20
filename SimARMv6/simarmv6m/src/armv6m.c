#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>

#include "armv6m.h"
#include "simarmv6m.h"
#include "memory.h"

/*****************************************************************************/

static int armv6m_update_nz(armv6m_state *as,uint32_t rd)
{
 if ( rd & (1<<31) )	as->as_xpsr.xpsr_apsr |=  (1<<31);
 else			as->as_xpsr.xpsr_apsr &= ~(1<<31);
 if ( rd==0	 )	as->as_xpsr.xpsr_apsr |=  (1<<30);
 else			as->as_xpsr.xpsr_apsr &= ~(1<<30);
 return(0);
}

static int armv6m_update_carry(armv6m_state *as,int c)
{
 if ( c )	as->as_xpsr.xpsr_apsr |=  (1<<29);
 else		as->as_xpsr.xpsr_apsr &= ~(1<<29);
 return(0);
}

static int armv6m_update_overflow(armv6m_state *as,int v)
{
 if ( v )	as->as_xpsr.xpsr_apsr |=  (1<<28);
 else		as->as_xpsr.xpsr_apsr &= ~(1<<28);
 return(0);
}

static uint32_t armv6m_add_with_carry(armv6m_state *as,uint32_t ua,uint32_t ub,int c)
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

 armv6m_update_carry(as,uresult64 != (uint64_t)uresult);
 armv6m_update_overflow(as,iresult64 != (int64_t)iresult);
 armv6m_update_nz(as,uresult);

 return(uresult);
}

static uint32_t armv6m_zero_extend(uint32_t r,int n)
{
 return(r&((1<<n)-1));
}

static uint32_t armv6m_sign_extend(uint32_t r,int n)
{
 if ( r & (1<<(n-1)) )	r |= ~((1<<(n))-1);
 else			r &=  ((1<<(n))-1);
 return(r);
}

/* armv6m_condition(): flags[3:0] == {n,z,c,v} == APSR[31:28] == APSR>>28 */

static int armv6m_condition(int xx,uint32_t flags)
{
 switch ( xx )
  {	case  0:	return(flags&(1<<2));
	case  1:	return(!(flags&(1<<2)));
	case  2:	return(flags&(1<<1));
 	case  3:	return(!(flags&(1<<1)));
	case  4:	return(flags&(1<<3));
	case  5:	return(!(flags&(1<<3)));
 	case  6:	return(flags&(1<<0));
 	case  7:	return(!(flags&(1<<0)));
 	case  8:	return(flags&(1<<1) && (!(flags&(1<<2))));
	case  9:	return((!(flags&(1<<1))) || flags&(1<<2));
	case 10:	return((!(flags&(1<<3)))==(!(flags&(1<<0))));
 	case 11: 	return((!(flags&(1<<3)))!=(!(flags&(1<<0))));
	case 12:	return((!(flags&(1<<2))) && (!(flags&(1<<3)))==(!(flags&(1<<0))));
	case 13:	return((flags&(1<<2)) || (!(flags&(1<<3)))!=(!(flags&(1<<0))));
	default:	return(0);
  }
}

int armv6m_core_optimize_conditions(armv6m_state *as)
{
 uint32_t	i,flags;
 for ( i=0; i<16; i++ )
  {	as->conditions[i]=0;
	for ( flags=0; flags<16; flags++ )
	 {	if ( armv6m_condition(i,flags) )
			as->conditions[i] |= (1<<flags);
	 }
  }
 return(0);
}


#define	armv6m_is_mode_handler(as)	((as)->as_xpsr.xpsr_ipsr & ISPR_EXCEPTION_MASK)
#define	armv6m_is_mode_thread(as)	(!armv6m_is_mode_handler(as))
#define	armv6m_use_psp(as)		(armv6m_is_mode_thread(as) && ((as)->as_control & (1<<CONTROL_ASPSEL) ))
#define	armv6m_use_msp(as)		(!armv6m_use_psp(as))

#define	armv6m_ipsr_mode(as,mode)	((as)->as_xpsr.xpsr_ipsr = ((as)->as_xpsr.xpsr_ipsr & ~ISPR_EXCEPTION_MASK) | ((mode)&ISPR_EXCEPTION_MASK))

/* dst[n] = src[n] */
#define	copy_bit(dst,src,n)		dst=((dst)&(~(1<<(n))))|((src)&(1<<(n)))

/* dst[n+cnt-1:n] = src[n+cnt-1:n] */
#define	copy_bits(dst,src,n,cnt)	dst=((dst)&(~(((1<<(cnt))-1)<<(n))))|((src)&(((1<<(cnt))-1)<<(n)))

/* dst[n] = !!src */
#define	set_bit(dst,n,src)		dst=((dst)&(~(1<<(n))))|((src?1:0)<<(n))

static void armv6m_xsp_map(armv6m_state *as)
{
 /* map SP as either MSP or PSP: */
 as->as_reg[REG_SP] = armv6m_use_msp(as) ? as->as_msp : as->as_psp;
}

static void armv6m_xsp_unmap(armv6m_state *as)
{
 /* write back xSP to MSP or PSP, depending on the context: */
 if ( armv6m_use_msp(as) )
	as->as_msp = as->as_reg[REG_SP];
 else
	as->as_psp = as->as_reg[REG_SP];
}

int avmv6m_core_fprint_state(FILE *fw,memory_layout *ml,armv6m_state *as,int is_verbose)
{ 
 uint16_t	instr,thumb2;
 int		i;

 memory_ldrh(ml,as->as_reg[REG_PC]&(~1),&instr);
 if ( (instr & 0xF000) == 0xF000 )
	memory_ldrh(ml,(as->as_reg[REG_PC]&(~1))+2,&thumb2);

 armv6m_xsp_map(as);

 if ( is_verbose )
  {	fprintf(fw,"PC = 0x%.8x\n",as->as_reg[REG_PC]);

	if ( (instr & 0xF000) == 0xF000 )
		fprintf(fw,"{ INSTR,THUMB2 } = 0x%.8x\n",(instr<<16)|thumb2);
	else
		fprintf(fw,"INSTR = 0x%.4x\n",instr);

	for ( i=0; i<16; i++ )
	 {	fprintf(fw,"r%d%s = 0x%.8x%c",i,(i<10?" ":""),as->as_reg[i],i%4==3?'\n':' ');
	 }
	fprintf(fw,"MODE    = %s\n",armv6m_is_mode_thread(as)?"thread":"handler");
	fprintf(fw,"xSP     = %s\n",armv6m_use_msp(as)?"MSP":"PSP");
	fprintf(fw,"xPSR    = 0x%.8x\n",as->as_xpsr.xpsr_value);
	fprintf(fw,"CONTROL = 0x%.8x\n",as->as_control);
	fprintf(fw,"PRIMASK = 0x%.8x\n",as->as_primask);
	fprintf(fw,"MSP     = 0x%.8x\n",as->as_msp);
	fprintf(fw,"PSP     = 0x%.8x\n",as->as_psp);
  }
 else
  {	fprintf(fw,"%.8x ",as->as_reg[REG_PC]);

	if ( (instr & 0xF000) == 0xF000 )
		fprintf(fw,"%.4x %.4x",instr,thumb2);
	else
		fprintf(fw,"%.4x     ",instr);

	for ( i=0; i<15; i++ )
	 {	fprintf(fw," %.8x",as->as_reg[i]);
	 }

	fprintf(fw," %c %c %.8x %.8x %.8x %.8x %.8x\n",
		armv6m_is_mode_thread(as)?'T':'H',armv6m_use_msp(as)?'M':'P',
		as->as_xpsr.xpsr_value,as->as_control,as->as_primask,
		as->as_msp,as->as_psp);
  }

 return(0);
}


/* armv6m_core_exception_enter(): conforming to PushStack, B1.5.6 and B1.5.7 in ARM DDI 0419E */
int armv6m_core_exception_enter(memory_layout *ml,armv6m_state *as,int vector_num)
{
 uint32_t	framealign,frameptr,npc;

/* fprintf(stderr,"armv6m_exception_enter(): vector_num=%d\n",vector_num); */

 if ( (as->as_control & (1<<CONTROL_ASPSEL) ) && armv6m_is_mode_thread(as) )
  {	framealign=as->as_psp & 4;
	as->as_psp = (as->as_psp - 32) & (~4);
	frameptr = as->as_psp;
  }
 else
  {	framealign=as->as_msp & 4;
	as->as_msp = (as->as_msp - 32) & (~4);
	frameptr = as->as_msp;
  }

 memory_str(ml,frameptr   ,as->as_reg[0],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+ 4,as->as_reg[1],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+ 8,as->as_reg[2],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+12,as->as_reg[3],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+16,as->as_reg[REG_IP],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+20,as->as_reg[REG_LR],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+24,as->as_reg[REG_PC],MEMORY_ACCESS_HINT_STACK);
 memory_str(ml,frameptr+28,(as->as_xpsr.xpsr_value & ~(1<<XPSR_STACK_ALIGN)) | ((framealign?1:0)<<XPSR_STACK_ALIGN),MEMORY_ACCESS_HINT_STACK);

 if ( armv6m_is_mode_handler(as) )
 	as->as_reg[REG_LR] = 0xFFFFFFF1;
 else if ( ! (as->as_control & (1<<CONTROL_ASPSEL)) )
 	as->as_reg[REG_LR] = 0xFFFFFFF9;
 else
 	as->as_reg[REG_LR] = 0xFFFFFFFD;

 copy_bits(as->as_xpsr.xpsr_ipsr,vector_num,0,6);

 memory_ldr(ml,4*vector_num,&npc,MEMORY_ACCESS_HINT_NONE);
 as->as_reg[REG_PC]=npc;

 return(0);
}

/* armv6m_core_exception_leave(): conforming to PushStack, B1.5.8 in ARM DDI 0419E */
static int armv6m_core_exception_leave(memory_layout *ml,armv6m_state *as,uint32_t exc_return)
{
 uint32_t	frameptr,pc,xpsr;

 exc_return &= 0x0000000F;

 /* return to another handler: */
 if ( exc_return == 0x1 )
  {	frameptr=as->as_msp;
	as->as_control &= ~(1<<CONTROL_ASPSEL);
	armv6m_ipsr_mode(as,0); /* it is not 0 */
  }
 /* return to thread using main stack: */
 else if ( exc_return == 0x9 )
  {	frameptr=as->as_msp;
	as->as_control &= ~(1<<CONTROL_ASPSEL);
	armv6m_ipsr_mode(as,0); /* clear IPSR */
  }
 /* return to thread using process stack: */
 else if ( exc_return == 0xD )
  {	frameptr=as->as_psp;
	as->as_control |= (1<<CONTROL_ASPSEL);
	armv6m_ipsr_mode(as,0); /* clear IPSR */
  }
 else
	frameptr=0;

 memory_ldr(ml,frameptr   ,&as->as_reg[0],MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+ 4,&as->as_reg[1],MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+ 8,&as->as_reg[2],MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+12,&as->as_reg[3],MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+16,&as->as_reg[REG_IP],MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+20,&as->as_reg[REG_LR],MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+24,&pc,MEMORY_ACCESS_HINT_STACK);
 memory_ldr(ml,frameptr+28,&xpsr,MEMORY_ACCESS_HINT_STACK);

 if ( exc_return == 0x1 || exc_return == 0x09)
	as->as_msp = (as->as_msp + 0x20) | (xpsr&(1<<9)?4:0);
 else if ( exc_return == 0xD )
	as->as_psp = (as->as_psp + 0x20) | (xpsr&(1<<9)?4:0);
 
 as->as_xpsr.xpsr_apsr = (as->as_xpsr.xpsr_apsr & ~0xF0000000) | (xpsr & 0xF0000000);
 as->as_xpsr.xpsr_ipsr = (as->as_xpsr.xpsr_ipsr & ~0x0000003F) | (xpsr & 0x0000003F);
 as->as_xpsr.xpsr_epsr = (as->as_xpsr.xpsr_epsr & ~0x01000000) | (xpsr & 0x01000000);

 as->as_reg[REG_PC]=pc|1;

/* fprintf(stderr,"armv6m_core_exception_leave(): done.\n"); */

 return(0);
}

#define		BF_HM(instr)	((instr>>3)&15)
#define		BF_HD(instr)	((instr&7)+(instr&128?8:0))
#define		BF_FLAGS(as)	(((as)->as_xpsr.xpsr_apsr>>28)&15)

int armv6m_core_step(memory_layout *ml,armv6m_state *as,armv6m_hint *ah)
{
 uint16_t	instr;
 uint8_t	d,n,m,xd;
 uint32_t	rd,instr_in;

 int		increment_pc;

 if ( ah != NULL )
  {	ah->ah_hint=0;
	ah->ah_svc=0;
  }

 if ( memory_ldr(ml,as->as_reg[REG_PC]&(~3),&instr_in,MEMORY_ACCESS_HINT_PROGMEM) )
	return(-1);
 if ( ! ( as->as_reg[REG_PC] & (1<<1) ) )
	instr=instr_in;
 else
	instr=instr_in>>16;
	

 /* no return should occur after this point */
 /* only (armv6m_hint *)ah updates allowed to pass information */

 d=instr&7;
 n=(instr>>3)&7;
 m=(instr>>6)&7;
 xd=(instr>>8)&7;

 increment_pc=1;

 /* map SP as either MSP or PSP: */
 armv6m_xsp_map(as);

 switch ( (instr>>11)&0x1F )
  {   	
	uint8_t		b;
	uint8_t		ldrb;
	uint16_t	ldrh;
	uint16_t	thumb;

	/* LSLS */
	case 0:
		b=(instr>>6)&31;
		if ( b==0 )
			rd=as->as_reg[n];
		else	
		 {	rd=as->as_reg[n]<<b;
			armv6m_update_carry(as,as->as_reg[n]&(1<<(32-b)));
		 }	
		armv6m_update_nz(as,rd);
		as->as_reg[d]=rd;	
		break;

	/* LSRS */
	case 1:
		b=(instr>>6)&31;
		if ( b==0 )	
			rd=as->as_reg[n];
		else	
		 {	rd=as->as_reg[n]>>b;
			armv6m_update_carry(as,as->as_reg[n]&(1<<b));
		 }	
		armv6m_update_nz(as,rd);
		as->as_reg[d]=rd;	
		break;

	 /* ASRS */
	case 2:
		b=(instr>>6)&31;
		if ( b==0 )	
			rd=as->as_reg[n];
		else	
		 {	rd=as->as_reg[n]>>b;
			if ( as->as_reg[n]&(1<<31) )
				rd |= ~((1<<(32-b))-1);
			armv6m_update_carry(as,as->as_reg[n]&(1<<b));
		 }	
		armv6m_update_nz(as,rd);
		as->as_reg[d]=rd;	
		break;

	/* MOVS, SUBS, ADDS, SUBS */
	case 3:
		switch ( (instr>>9)&0x3 )
		 {	/* ADDS */
			case 0:
				rd=armv6m_add_with_carry(as,as->as_reg[n], as->as_reg[m],0);
				as->as_reg[d]=rd;
				break;
			/* SUBS */
			case 1:
				rd=armv6m_add_with_carry(as,as->as_reg[n],~as->as_reg[m],1);
				as->as_reg[d]=rd;
				break;

			/* ADDS */
			case 2:
				rd=armv6m_add_with_carry(as,as->as_reg[n], (uint32_t)m,0);
				as->as_reg[d]=rd;
				break;
			 /* SUBS */
			case 3:
				rd=armv6m_add_with_carry(as,as->as_reg[n],~(uint32_t)m,1);
				as->as_reg[d]=rd;
				break;
		 }
		break;

	 /* MOVS */
	case 4:
		rd=armv6m_zero_extend(instr,8);
		armv6m_update_nz(as,rd);
		as->as_reg[xd]=rd;
		break;

	/* CMP */
	case 5: 
	 	armv6m_add_with_carry(as,as->as_reg[xd],~armv6m_zero_extend(instr,8),1);
		break;

	/* ADDS */
	case 6:
		rd=armv6m_add_with_carry(as,as->as_reg[xd], armv6m_zero_extend(instr,8),0);
		as->as_reg[xd]=rd;
		break;

	/* SUBS */
	case 7:
		rd=armv6m_add_with_carry(as,as->as_reg[xd],~armv6m_zero_extend(instr,8),1);
		as->as_reg[xd]=rd;
 		break;

	/* many instructions */
	case 8:
		switch ( (instr>>6)&0x1F )
		 {
			/* ANDS */
			case 0:
				rd = as->as_reg[d] & as->as_reg[n];
				armv6m_update_nz(as,rd);
			        as->as_reg[d]=rd;
				break;

			/* EORS */
			case 1:
				rd = as->as_reg[d] ^ as->as_reg[n];
				armv6m_update_nz(as,rd);
			        as->as_reg[d]=rd;
				break;
			
			/* LSLS */
			case 2:
				b=as->as_reg[n];
				if ( b==0 )
					rd=as->as_reg[d];
				else	
				 {	rd=as->as_reg[d]<<b;
					armv6m_update_carry(as,as->as_reg[d]&(1<<(32-b)));
				 }	
				armv6m_update_nz(as,rd);
				as->as_reg[d]=rd;	
 				break;

			/* LSRS */
			case 3: 
				b=as->as_reg[n];
				if ( b==0 )	
					rd=as->as_reg[d];
				else	
				 {	rd=as->as_reg[d]>>b;
					armv6m_update_carry(as,as->as_reg[d]&(1<<b));
				 }	
				armv6m_update_nz(as,rd);
				as->as_reg[d]=rd;	
				break;

			/* ASRS */
			case 4:
				b=as->as_reg[n];
				if ( b==0 )	
					rd=as->as_reg[d];
				else	
				 {	rd=as->as_reg[d]>>b;
					if ( as->as_reg[d]&(1<<31) )
						rd |= ~((1<<(32-b))-1);
					armv6m_update_carry(as,as->as_reg[d]&(1<<b));
				 }	
				armv6m_update_nz(as,rd);
				as->as_reg[d]=rd;	
				break;

			/* ADCS */
			case 5:
				rd=armv6m_add_with_carry(as,as->as_reg[d], as->as_reg[n],BF_FLAGS(as)&(1<<1));
				as->as_reg[d]=rd;
				break;

			 /* SBCS */
 			case 6:
				rd=armv6m_add_with_carry(as,as->as_reg[d],~as->as_reg[n],BF_FLAGS(as)&(1<<1));
				as->as_reg[d]=rd;
				break;
	
			/* RORS */
			case 7:
				break;


			/* TSTS */
			case 8:
				rd = as->as_reg[d] & as->as_reg[n];
				armv6m_update_nz(as,rd);
 				break;

			 /* RSBS */
			case 9:
				rd=armv6m_add_with_carry(as,~as->as_reg[n],0,1);
				as->as_reg[d]=rd;
				break;

			 /* CMP */
			case 10:
				armv6m_add_with_carry(as,as->as_reg[d],~as->as_reg[n],1);
				break;

			/* CMN */
			case 11:
				armv6m_add_with_carry(as,as->as_reg[d], as->as_reg[n],0);
				break;

			/* ORRS */
			case 12:
				rd = as->as_reg[d] | as->as_reg[n];
				armv6m_update_nz(as,rd);
			        as->as_reg[d]=rd;
				break;
			
			/* MULS */
			case 13:
				rd = as->as_reg[d] * as->as_reg[n];
				armv6m_update_nz(as,rd);
			        as->as_reg[d]=rd;
				break;

			/* BICS */
			case 14:
				rd = as->as_reg[d] & (~as->as_reg[n]);
				armv6m_update_nz(as,rd);
			        as->as_reg[d]=rd;
 				break;

			/* MVNS */
			case 15:
				rd =(~as->as_reg[n]);
				armv6m_update_nz(as,rd);
			        as->as_reg[d]=rd;
				break;

			/* ADD */
			case 16: case 17: case 18: case 19:
				as->as_reg[BF_HD(instr)] += as->as_reg[BF_HM(instr)];
				armv6m_xsp_unmap(as);
				break;

			 /* CMP */
			case 20: case 21: case 22: case 23:
				armv6m_add_with_carry(as,as->as_reg[BF_HD(instr)],~as->as_reg[BF_HM(instr)],1);
				break;

			/* MOV */
			case 24: case 25: case 26: case 27:
				as->as_reg[BF_HD(instr)] = as->as_reg[BF_HM(instr)];
				if ( BF_HD(instr)==15 )
				 {	/* exception return: */
					if ( ( as->as_reg[REG_PC] & 0xFFFFFFF0 ) == 0xFFFFFFF0 )
						armv6m_core_exception_leave(ml,as,as->as_reg[REG_PC]);

					increment_pc=0;
				 }
				else
					armv6m_xsp_unmap(as);

				break;

			 /* BX */
			case 28: case 29:
				as->as_reg[REG_PC] = as->as_reg[BF_HM(instr)];
				if ( ( as->as_reg[REG_PC] & 0xFFFFFFF0 ) == 0xFFFFFFF0 )
					armv6m_core_exception_leave(ml,as,as->as_reg[REG_PC]);

				increment_pc=0;
 				break;

			/* BLX */
			case 30: case 31:
				as->as_reg[REG_LR] = as->as_reg[REG_PC] + 2;
				as->as_reg[REG_PC] = as->as_reg[BF_HM(instr)];
				/* this is highly unlikely, though, not impossible (should be checked in the ARM standard): */
				if ( ( as->as_reg[REG_PC] & 0xFFFFFFF0 ) == 0xFFFFFFF0 )
					armv6m_core_exception_leave(ml,as,as->as_reg[REG_PC]);

				increment_pc=0;
				break;
		 }
		break;

	/* LDR rt, [pc, #imm] */
	case 9:
		memory_ldr(ml,(as->as_reg[REG_PC]&(~3))+4+(armv6m_zero_extend(instr,8)<<2),&rd,MEMORY_ACCESS_HINT_PROGMEM);
		as->as_reg[xd]=rd;
		break;

	case 10: case 11:
		switch ( (instr>>9)&0x7 )
		 {	
		 	uint8_t		ldrb;
			uint16_t	ldrh;

 			/* STR rt, [rn, rm] */
			case 0:
				memory_str(ml,(as->as_reg[m]+as->as_reg[n])&(~3),as->as_reg[d],MEMORY_ACCESS_HINT_NONE);
				break;

			/* STRH rt, [rn, rm] */
			case 1:
				memory_strh(ml,(as->as_reg[m]+as->as_reg[n])&(~1),as->as_reg[d]);
				break;
 
 			/* STRB rt, [rn, rm] */
			case 2:
				memory_strb(ml,as->as_reg[m]+as->as_reg[n],as->as_reg[d]);
				break;

			/* LDRSB rt, [rn, rm] */
			case 3:
				memory_ldrb(ml,as->as_reg[m]+as->as_reg[n],&ldrb);
				rd=ldrb|((ldrb&0x80)?0xFFFFFF00:0);
				as->as_reg[d]=rd;
				break;
			
			/* LDR rt, [rn, rm] */
			case 4:
				memory_ldr(ml,(as->as_reg[m]+as->as_reg[n])&(~3),&rd,MEMORY_ACCESS_HINT_NONE);
				as->as_reg[d]=rd;
				break;
 
 			/* LDRH rt, [rn, rm] */
			case 5:
				memory_ldrh(ml,(as->as_reg[m]+as->as_reg[n])&(~1),&ldrh);
				rd=ldrh;
				as->as_reg[d]=rd;
				break;

			/* LDRB rt, [rn, rm] */
			case 6:
				memory_ldrb(ml,as->as_reg[m]+as->as_reg[n],&ldrb);
				rd=ldrb;
				as->as_reg[d]=rd;
				break;
			
			/* LDRSH rt, [rn, rm] */
			case 7:
				memory_ldrh(ml,(as->as_reg[m]+as->as_reg[n])&(~1),&ldrh);
				rd=ldrh|((ldrh&0x8000)?0xFFFF0000:0);
				as->as_reg[d]=rd;
				break;
		 }
		break;

	/* STR rt, [rn, #imm] */
	case 12:
		b=(instr>>6)&31;
		memory_str(ml,as->as_reg[n]+(b<<2),as->as_reg[d],MEMORY_ACCESS_HINT_NONE);
		break;
 
	/* LDR rt, [rn, #imm] */
	case 13:
		b=(instr>>6)&31;
		memory_ldr(ml,as->as_reg[n]+(b<<2),&rd,MEMORY_ACCESS_HINT_NONE);
		as->as_reg[d]=rd;
		break;

	/* STRB rt, [rn, #imm] */
	case 14:
		b=(instr>>6)&31;
		memory_strb(ml,as->as_reg[n]+b,as->as_reg[d]);
		break;

	/* LDRB rt, [rn, #imm] */
	case 15:
		b=(instr>>6)&31;
		memory_ldrb(ml,as->as_reg[n]+b,&ldrb);
		rd=ldrb;
		as->as_reg[d]=rd;
		break;

	/* STRH rt, [rn, #imm] */
	case 16:
		b=(instr>>6)&31;
		memory_strh(ml,as->as_reg[n]+(b<<1),as->as_reg[d]);
		break;

	/* LDRH rt, [rn, #imm] */
	case 17:
		b=(instr>>6)&31;
		memory_ldrh(ml,as->as_reg[n]+(b<<1),&ldrh);
		rd=ldrh;
		as->as_reg[d]=rd;
		break;

	/* STR rt, [sp, #imm] */
	case 18:
		memory_str(ml,as->as_reg[REG_SP]+(armv6m_zero_extend(instr,8)<<2),as->as_reg[xd],MEMORY_ACCESS_HINT_STACK);
 		break;

	/* LDR rt, [sp, #imm] */
	case 19:
		memory_ldr(ml,as->as_reg[REG_SP]+(armv6m_zero_extend(instr,8)<<2),&rd,MEMORY_ACCESS_HINT_STACK);
		as->as_reg[xd]=rd;
		break;

	 /* ADR rd, [pc, #imm] */
	case 20:
		rd=(as->as_reg[REG_PC]&(~3))+4+(armv6m_zero_extend(instr,8)<<2);
		as->as_reg[xd]=rd;
		break;

	/* ADD rd, sp, #imm */
	case 21:
		rd=as->as_reg[REG_SP]+(armv6m_zero_extend(instr,8)<<2);
		as->as_reg[xd]=rd;
 		break;

	case 22: case 23:

		/* ADD sp, sp, #imm */
		if ( (instr & ~((1<<7)-1)) == 0xB000 )
		 {	as->as_reg[REG_SP] += (armv6m_zero_extend(instr,7)<<2);
			armv6m_xsp_unmap(as);

		 }
		/* SUB sp, sp, #imm */
		else if ( (instr & ~((1<<7)-1)) == 0xB080 )
		 {	as->as_reg[REG_SP] -= (armv6m_zero_extend(instr,7)<<2);
			armv6m_xsp_unmap(as);
		 }
		/* SXTH rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xB200 )
		 {	rd=(as->as_reg[n]&0xFFFF)|(as->as_reg[n]&0x8000?0xFFFF0000:0);
			as->as_reg[d]=rd;
		 }
		/* SXTB rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xB240 )
		 {	rd=(as->as_reg[n]&0xFF)|(as->as_reg[n]&0x80?0xFFFFFF00:0);
			as->as_reg[d]=rd;
		 }
		/* UXTH rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xB280 )
		 {	rd=(as->as_reg[n]&0xFFFF);
			as->as_reg[d]=rd;
		 }
		/* UXTB rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xB2C0 )
		 {	rd=(as->as_reg[n]&0xFF);
			as->as_reg[d]=rd;
		 }
		/* PUSH {...} */
		else if ( (instr & ~((1<<9)-1)) == 0xB400 )
		 {	int	d;
			if ( instr & (1<<8) )
			 {	as->as_reg[REG_SP] -= 4;
				memory_str(ml,as->as_reg[REG_SP],as->as_reg[REG_LR],MEMORY_ACCESS_HINT_STACK);
			 }
			for ( d=7; 0<=d; d-- )
			 {	if ( instr & (1<<d) )
				 {	as->as_reg[REG_SP] -= 4;
					memory_str(ml,as->as_reg[REG_SP],as->as_reg[d],MEMORY_ACCESS_HINT_STACK);
				 }
			 }
			armv6m_xsp_unmap(as);
		 }

		/* CPSIE i, CPSID i */
		else if ( (instr & ~((1<<5)-1)) == 0xB660 )
		 {	if ( instr & (1<<4) )
				as->as_primask |=  (1<<PRIMASK_PRIMASK);
		 	else 
				as->as_primask &= ~(1<<PRIMASK_PRIMASK);
		 }

		/* REV rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xBA00 )
		 {	uint32_t	rn;
			rn=as->as_reg[n];
			rd=(rn>>24)|((rn>>8)&0xFF00)|((rn<<8)&(0xFF0000))|(rn<<24);
			as->as_reg[d]=rd;
		 }
		/* REV16 rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xBA40 )
		 {	uint32_t	rn;
			rn=as->as_reg[n];
			rd=(rn>>16)|(rn<<16);
			as->as_reg[d]=rd;
		 }
		/* REVSH rd, rm */
		else if ( (instr & ~((1<<6)-1)) == 0xBAC0 )
		 {	uint32_t	rn;
			rn=as->as_reg[n];
			rd=(rn>>24)|((rn>>8)&0xFF00);
			if ( rd&0x8000 )	rd |= 0xFFFF0000;
			as->as_reg[d]=rd;
		 }
		/* POP {...} */
		else if ( (instr & ~((1<<9)-1)) == 0xBC00 )
		 {	int	d;
			/* fprintf(stderr,"POP: xSP = %.8x\n",as->as_reg[REG_SP]); */
			for ( d=0; d<8; d++ )
			 {	if ( instr & (1<<d) )
				 {	memory_ldr(ml,as->as_reg[REG_SP],&as->as_reg[d],MEMORY_ACCESS_HINT_STACK);
					as->as_reg[REG_SP] += 4;
				 }
			 }
			if ( instr & (1<<8) )
			 {	uint32_t	npc;

				memory_ldr(ml,as->as_reg[REG_SP],&npc,MEMORY_ACCESS_HINT_STACK);
				as->as_reg[REG_SP] += 4;
				armv6m_xsp_unmap(as);

				/* exception return: */
				if ( ( npc & 0xFFFFFFF0 ) == 0xFFFFFFF0 )
				 {	armv6m_core_exception_leave(ml,as,npc&0x0000000F);
				 }
				/* normal return: */
				else
				 	as->as_reg[REG_PC]=npc;

				increment_pc = 0;
			 }
			else
				armv6m_xsp_unmap(as);
		   }

		/* YIELD */
		else if ( instr == 0xBF10 )
		 {	if ( ah != NULL )
				ah->ah_hint = STEP_HINT_YIELD;
		 }

		/* WFE */
		else if ( instr == 0xBF20 )
		 {	if ( ah != NULL )
				ah->ah_hint = STEP_HINT_WFE;
		 }

		/* WFI */
		else if ( instr == 0xBF30 )
		 {	if ( ah != NULL )
				ah->ah_hint = STEP_HINT_WFI;
		 }

		/* SEV */
		else if ( instr == 0xBF40 )
		 {	if ( ah != NULL )
				ah->ah_hint = STEP_HINT_SEV;
		 }

		break;

	/* STMIA {...} */
	case 24:
		rd=as->as_reg[xd];
		for ( d=0; d<8; d++ )
		 {	if ( instr & (1<<d) )
			 {	memory_str(ml,rd,as->as_reg[d],MEMORY_ACCESS_HINT_RAM);
				rd += 4;
			 }
		 }
		as->as_reg[xd]=rd;
		break;

	/* LDMIA {...} */
	case 25:
		rd=as->as_reg[xd];
		for ( d=0; d<8; d++ )
		 {	if ( instr & (1<<d) )
			 {	memory_ldr(ml,rd,&as->as_reg[d],MEMORY_ACCESS_HINT_RAM);
				rd += 4;
			 }
		 }
		if ( ! (instr & (1<<xd)) )
			as->as_reg[xd]=rd;
		break;

	case 26: case 27:

		 /* SVC */
		if ( (instr & 0xFF00) == 0xDF00 )
		 {	if ( ah != NULL )
				ah->ah_svc=1;
		 }

		/* Bxx */
		else if ( (instr & ~((1<<12)-1)) == 0xD000 )
		 {	if ( as->conditions[(instr>>8)&15] & (1<<BF_FLAGS(as)) )
			 {	as->as_reg[REG_PC] += 4 + 2*armv6m_sign_extend(instr,8);
				increment_pc=0;
			 }
		 }

		break;

	/* B */
	case 28:
		as->as_reg[REG_PC] += 4 + 2*armv6m_sign_extend(instr,11);
		increment_pc=0;
		break;

	/* THUMB2 instructions: */
	case 30: case 31:

		as->as_reg[REG_PC] += 2;
		memory_ldrh(ml,as->as_reg[REG_PC]&(~1),&thumb);

		/* MRS */	/* mrs r0, PRIMASK */
		if ( instr==0xF3EF && (thumb&0xF000) == 0x8000 )
		 {	int		hd,sysm;
			uint32_t	rd;
			hd  =(thumb&0x0F00)>>8;
			sysm=(thumb&0x00FF);
			rd=0;
			if ( (sysm&0xF8)==0x00 )
			 {	if ( sysm&(1<<0) )
				 	rd |= as->as_xpsr.xpsr_ipsr & 0x01FF;
				if ( sysm&(1<<1) )
					rd &= ~(1<<24);
				if ( ! (sysm&(1<<2)) )
					rd |= as->as_xpsr.xpsr_apsr & (15<<28);
			 }
			else if ( (sysm&0xF8)==0x08 )
			 {	if ( (sysm&0x07)==0 )
				 	rd = as->as_msp;
				else if ( (sysm&0x07)==1 )	
					rd = as->as_psp;
			 }
			else if ( (sysm&0xF8)==0x10 )
			 {	if ( (sysm&0x07)==0 )
				 	rd = as->as_primask&1;
				else if ( (sysm&0x07)==4 )	
					rd = as->as_control&3;
			 }
			as->as_reg[hd]=rd;
		 }
		/* MSR */	/* msr PRIMASK, r0 */
		else if ( (instr&0xFFF0)==0xF380 && (thumb &0xFF00) == 0x8800 )
		 {	int		hn,sysm;
			uint32_t	rn;
			hn  =(instr&0x000F);
			sysm=(thumb&0x00FF);
			rn=as->as_reg[hn];
			if ( (sysm&0xF8)==0x00 )
			 {	if ( ! (sysm&(1<<2)) )
					copy_bits(as->as_xpsr.xpsr_apsr,rn,28,4);
			 }	
			else if ( (sysm&0xF8)==0x08 )
			 {	if ( (sysm&0x07)==0 )
				 { 	as->as_msp = rn&(~3);
				 }
				else if ( (sysm&0x07)==1 )	
				 {	as->as_psp = rn&(~3);
				 }
			 }
			else if ( (sysm&0xF8)==0x10 )
			 {	if ( (sysm&0x07)==0 )
					copy_bit(as->as_primask,rn,0);
				else if ( (sysm&0x07)==4 )	
			 	 {	copy_bit(as->as_control,rn,0);
					if ( armv6m_is_mode_thread(as) )
						copy_bit(as->as_control,rn,1);

					armv6m_xsp_map(as);
				 }
			 }		
		 }
		/* BL */
		else if ( (instr & ~((1<<11)-1)) == 0xF000 && (thumb & 0xD000) == 0xD000 )
		 {      uint32_t	s,j1,j2,i1,i2,bl_offset;

			s=(instr>>10)&1;
        	        j1=(thumb>>13)&1;
	                j2=(thumb>>11)&1;
        	        i1=(j1^s?0:1);
	                i2=(j2^s?0:1);
			bl_offset = (s?0xFF000000:0) | (i1<<23) | (i2<<22) | (((uint32_t)instr&0x3FF)<<12) | ((thumb&0x7FF)<<1);
			as->as_reg[REG_LR] = as->as_reg[REG_PC] + 2;
			as->as_reg[REG_PC] += bl_offset + 2;
			increment_pc = 0;
		 }	

		break;
  }

 if ( increment_pc )
	as->as_reg[REG_PC] += 2;

 return(0);
}

/* armv6m_reset(): conforming to TakeReset, see B1.5.5 in ARM DDI 0419E */
int armv6m_core_reset(memory_layout *ml,armv6m_state *as)
{
 uint32_t	VTOR;

 VTOR=0x00000000;

 memory_ldr(ml,VTOR+0,&as->as_msp,MEMORY_ACCESS_HINT_NONE);

 as->as_xpsr.xpsr_value = 0;
 as->as_xpsr.xpsr_ipsr	&= ~ISPR_EXCEPTION_MASK;
 as->as_xpsr.xpsr_epsr 	|= (1<<EPSR_THUMB_STATE);

 as->as_control =  0x00000000;
 as->as_primask	=  0x00000000;

 memory_ldr(ml,VTOR+4,&as->as_reg[REG_PC],MEMORY_ACCESS_HINT_NONE);

 return(0);
}
