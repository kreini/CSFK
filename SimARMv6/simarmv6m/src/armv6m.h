/*****************************************************************************/

#ifndef	__ARMV6M_INCLUDE
#define	__ARMV6M_INCLUDE	1

/*****************************************************************************/

#define		APSR_FLAG_NEGATIVE		31	/*  [31] 	*/
#define		APSR_FLAG_ZERO			30	/*  [30] 	*/
#define		APSR_FLAG_CARRY			29	/*  [29]	*/
#define		APSR_FLAG_OVERFLOW		28	/*  [28] 	*/
#define		EPSR_THUMB_STATE		24	/*  [24] 	*/
#define		XPSR_STACK_ALIGN		 9	/*   [9]	*/
#define		IPSR_EXCEPTION_LENGTH		 6	/* [5:0]	*/
#define		ISPR_EXCEPTION_MASK	      0x3F	/* masks [5:0]  */
#define		PRIMASK_PRIMASK			 0	/*   [0]	*/
#define		CONTROL_ASPSEL			 1	/*   [1]	*/

#define		REG_IP				12	/* intra procedure call register 	*/
#define		REG_SP				13	/* stack pointer register		*/
#define		REG_LR				14	/* link register			*/
#define		REG_PC				15	/* program counter register		*/

#include	"memory.h"

typedef struct
 {	uint32_t	as_reg[16];

	union 
	 {	uint32_t	xpsr_apsr;
		uint32_t	xpsr_epsr;
		uint32_t	xpsr_xpsr;
		uint32_t	xpsr_ipsr;
		uint32_t	xpsr_value;
	 } as_xpsr;

	uint32_t	as_msp;
	uint32_t	as_psp;
	uint32_t	as_primask;
	uint32_t	as_control;

	uint16_t	conditions[16];		/* not a real state, just optimizes the evaluation of the conditional branches */

 } armv6m_state;

#define		STEP_HINT_NONE			0
#define		STEP_HINT_YIELD			1
#define		STEP_HINT_WFE			2
#define		STEP_HINT_WFI			3
#define		STEP_HINT_SEV			4

typedef struct
 {	int		ah_hint;
	int		ah_svc;
 } armv6m_hint;

#define armv6m_is_mode_handler(as)	((as)->as_xpsr.xpsr_ipsr & ISPR_EXCEPTION_MASK)
#define armv6m_is_mode_thread(as)	(!armv6m_is_mode_handler(as))
#define armv6m_use_psp(as)		(armv6m_is_mode_thread(as) && ((as)->as_control & (1<<CONTROL_ASPSEL) ))
#define armv6m_use_msp(as)		(!armv6m_use_psp(as))

int	armv6m_core_reset(memory_layout *ml,armv6m_state *as);
int	armv6m_core_optimize_conditions(armv6m_state *as);
int	avmv6m_core_fprint_state(FILE *fw,memory_layout *ml,armv6m_state *as,int is_verbose);
int	armv6m_core_exception_enter(memory_layout *ml,armv6m_state *as,int vector_num);
int	armv6m_core_step(memory_layout *ml,armv6m_state *as,armv6m_hint *ah);

/*****************************************************************************/

#endif

/*****************************************************************************/
