
#ifndef CK_TIME_HAL_H_
#define CK_TIME_HAL_H_


#define CK_SYSTICK_CTRL_ENABLE				1u<<0
#define CK_SYSTICK_CTRL_INT					1u<<1
#define CK_SYSTICK_CTRL_CLKSOURCE			1u<<2
#define CK_SYSTICK_LOAD_MASK			    		0xFFFFFFu

void CK_TIME_SetTimeOut(uint32_t time);

uint32_t CK_TIME_GetTimeOut(void);

void HAL_IncTick(void);

uint32_t HAL_GetTick(void);

uint32_t CK_milliSec(void);

uint32_t CK_microSec( void );

void CK_delayMs(uint32_t msec);

void CK_delayUs(uint32_t usec);

#endif /* CK_TIME_HAL_H_ */
