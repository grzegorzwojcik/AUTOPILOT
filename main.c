/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : Quadcopter autopilot board main function.
**
**  Author		: Grzegorz WÓJCIK
**
**  Environment : Atollic TrueSTUDIO/STM32
**
**	MCU:		: STM32F407VET6
**
**	RTOS		: Free RTOS
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f4xx.h"
#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"

#include "ADC.h"
#include "functions.h"



/****					MAIN FUNCTION					***/
int main(void)
{
	/* TODO - Add your application code here */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	if( SYSTEM_ClockCheck() != RESET ){
		vhLED_initGPIO();
		vhADC_init();

		vStartLEDTasks(tskLED_FLASH_PRIORITY);
		vStartADC_VoltPwrTask(tskADC_TASK_PRIORITY);
		vTaskStartScheduler();
	}
}
/****				END OF MAIN FUNCTION				***/

