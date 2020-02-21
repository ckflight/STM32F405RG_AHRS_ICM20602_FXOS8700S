
#include "stm32f4xx_hal.h"
#include "CK_TIME_HAL.h"
#include "CK_SYSTEM.h"

//int isFirst = 1;
uint32_t sysTickCounter = 0;
uint32_t timeout_num;

void CK_TIME_SetTimeOut(uint32_t time){
	timeout_num = time;
}

uint32_t CK_TIME_GetTimeOut(void){
	return timeout_num;
}

//OVERRWRITE OF WEAK HAL DECLERATION
void HAL_IncTick(void){

	sysTickCounter++;

	if(timeout_num){
		timeout_num--;
	}

}
//OVERRWRITE OF WEAK HAL DECLERATION
uint32_t HAL_GetTick(void){
  return sysTickCounter ;
}

uint32_t CK_milliSec(void){

	//HAL Inits this part
	/*if(isFirst == 1){
		isFirst = 0;
		SysTick->LOAD = ((uint32_t)((F_CPU/1000)-1));//1mS
		SysTick->VAL = 0;
		SysTick->CTRL |= CK_SYSTICK_CTRL_ENABLE | CK_SYSTICK_CTRL_INT | CK_SYSTICK_CTRL_CLKSOURCE;
	}*/

	return sysTickCounter;
}

/*
 * This function count how many microseconds based on millisec
 * passed between two execution of itself
 * */

uint32_t CK_microSec( void ){

	//HAL Inits this part
	/*if(isFirst == 1){
		isFirst = 0;
		SysTick->LOAD = ((uint32_t)((F_CPU/1000)-1));//1mS
		SysTick->VAL = 0;
		SysTick->CTRL |= CK_SYSTICK_CTRL_ENABLE | CK_SYSTICK_CTRL_INT | CK_SYSTICK_CTRL_CLKSOURCE;
	}*/

	uint32_t ticks ;
	uint32_t count ;

	//This logic here prevents some error that i faced with previous implementation
	SysTick->CTRL;
	do{
		ticks = SysTick->VAL;
		count = sysTickCounter;
	}while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);

	return count * 1000 + (SysTick->LOAD + 1 - ticks) / (F_CPU / 1000000);
}

void CK_delayMs(uint32_t msec){
    while(msec--)CK_delayUs(1000);
}
void CK_delayUs(uint32_t usec){
	uint32_t now = CK_microSec();
	while (CK_microSec() - now < usec);
}
