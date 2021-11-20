/*****************************************************************************/

#ifndef	__SIMARMV6M_INCLUDE
#define	__SIMARMV6M_INCLUDE	1

/*****************************************************************************/

#include "memory.h"

typedef struct armv6m_update
 {	struct armv6m_update	*prev,*next;
	int			(*au_callback_update)(void *param);
	void			*au_param;
	int			au_period;
 } armv6m_update;

typedef struct armv6m_exception
 {	struct armv6m_exception	*prev,*next;
	int			(*ae_callback_exception_state)(void *param);
	int			(*ae_callback_exception_delay)(void *param);
	int			(*ae_callback_exception_fdesc)(void *param);
	void			*ae_param;
 } armv6m_exception;

typedef struct 
 {	armv6m_exception	*aec_first;
 } armv6m_exception_control;

typedef struct 
 {	armv6m_update		*auc_first;
 } armv6m_update_control;

typedef struct key_value
 {	struct key_value	*prev,*next;
	char			*kv_key;
	char			*kv_value;
 } key_value;

typedef struct peripheral_definition
 {	struct peripheral_definition	*prev,*next;
	char				*pd_argument;
	char				*pd_device,*pd_peripheral;
	key_value			*pd_list_kv;
	void				*pd_data;
 } peripheral_definition;

/*****************************************************************************/

extern	volatile int	sig_int;

/*****************************************************************************/

armv6m_exception *armv6m_exception_add(armv6m_exception_control *aec);
armv6m_update	 *armv6m_update_add(armv6m_update_control *auc);
memory_block 	 *memory_layout_add_block(memory_layout *ml);

/*****************************************************************************/

#endif

/*****************************************************************************/
