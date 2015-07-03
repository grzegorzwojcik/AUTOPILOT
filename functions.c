/*
 * functions.c
 *
 *  Created on: Jun 29, 2015
 *      Author: Grzegorz
 */


#include "functions.h"


/* Private functions ---------------------------------------------------------*/
FlagStatus SYSTEM_ClockCheck(void){

	GV_SystemReady		= 0;
	RCC_ClocksTypeDef ClksFreq;
	RCC_GetClocksFreq(&ClksFreq);

	if( ClksFreq.SYSCLK_Frequency == 168000000 ){
		if( ClksFreq.HCLK_Frequency == 168000000 ){
			return SET;
		}
	}
	else
		return RESET;
}

/*-- Interrupts --------------------------------------------------------------*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

//void SysTick_Handler(void)
//{
//	GV_SystemCounter++;
//	if( GV_SystemCounter >= 2000 ){
//		GV_SystemCounter = 0;
//	}
//}
