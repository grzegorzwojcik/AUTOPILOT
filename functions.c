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


void vhLED_initGPIO(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_initStruct;
	GPIO_initStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_initStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_initStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_initStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_initStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_initStruct);

	GPIO_SetBits(GPIOD, GPIO_Pin_8);
	GPIO_ResetBits(GPIOD, GPIO_Pin_9);
}


void vTaskLED1(void * pvParameters)
{
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		/*		 250ms delay.	 */
		vTaskDelayUntil( &xLastFlashTime, 250 );
		/* 		Toggle D1.		 */
		GPIO_ToggleBits(GPIOD, GPIO_Pin_8);
	}
}


void vTaskLED2(void * pvParameters)
{
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();
	for(;;)
	{
		/*		 500ms delay.	 */
		vTaskDelayUntil( &xLastFlashTime, 250 );
		/* 		Toggle D2.		 */
		GPIO_ToggleBits(GPIOD, GPIO_Pin_9);
	}
}


void vStartLEDTasks(unsigned portBASE_TYPE uxPriority){
	xTaskHandle xHandleTaskLED1, xHandleTaskLED2;
	xTaskCreate(vTaskLED1, "LED1", configMINIMAL_STACK_SIZE, NULL, uxPriority, &xHandleTaskLED1);
	xTaskCreate(vTaskLED2, "LED2", configMINIMAL_STACK_SIZE, NULL, uxPriority, &xHandleTaskLED2);
}
