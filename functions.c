/*
 * functions.c
 *
 *  Created on: Jun 29, 2015
 *      Author: Grzegorz
 */


/*
 * functionc.c
 *
 *  Created on: Sep 2, 2014
 *      Author: Grzegorz Wójcik
 */

#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"

#include "functions.h"
#include "CONTROLLER.h"
#include "FAULTS.h"
#include "diskio.h"

/* Private functions ---------------------------------------------------------*/
void PLL_Configurattion(void){
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9); // 72MHz
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){
	}

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while (RCC_GetSYSCLKSource() != 0x08){
	}

	SystemCoreClockUpdate();
}

void SYSTEM_ClockCheck(void){

	GV_TimeStart 		= RESET;
	GV_TimeCounter 		= 0;
	GV_SystemReady		= 0;
	RCC_ClocksTypeDef ClksFreq;
	RCC_GetClocksFreq(&ClksFreq);

	if( ClksFreq.SYSCLK_Frequency == 72000000 ){
		if( ClksFreq.HCLK_Frequency == 72000000 ){
			GV_SystemReady = 1;
		}
	}
	else
		GV_SystemReady = 0;

}

/*-- Interrupts --------------------------------------------------------------*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	GV_SystemCounter++;
	if( GV_SystemCounter >= 2000 ){
		GV_SystemCounter = 0;
	}

	if( GV_SystemCounter % 10 == 0)	// execute it every 10 ms
		disk_timerproc();			// this function is used by the SD card libraries


	if( GV_SystemCounter % 1000 == 0 ){	// 1 second has passed
		NAVI_Struct.TimeSS++;			// increment seconds
		if( NAVI_Struct.TimeSS >= 60 ){
			NAVI_Struct.TimeMM++;
			NAVI_Struct.TimeSS = 0;
		}
		if( NAVI_Struct.TimeMM >= 60 ){
			NAVI_Struct.TimeHH++;
			NAVI_Struct.TimeMM = 0;
		}
		if( NAVI_Struct.TimeHH >= 24 ){
			NAVI_Struct.TimeHH = 0;
		}

		if(GV_TimeStart == SET){
			if(--GV_TimeCounter <= 0){		// decrementing each second
				GV_TimeStart = RESET;
				GV_SDfileCreated = RESET;
			}
		}
	}
}

/*		50 Hz control loop		*/
void TIM2_IRQHandler(void){
	if( TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		if(NAVI_Struct.FaultM == 1){
			FAULTS_injectSERVO();
		}

		if(NAVI_Struct.FaultM == 0)
			FAULTS_removalSERVO();
			//TIM2->CCR3 = 20;

		CTRL_controlAUTOPILOT(0, 100, 110, 190);
	}
}




