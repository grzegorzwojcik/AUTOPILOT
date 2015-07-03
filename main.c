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
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f4xx.h"
#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"

#include "ADC.h"
#include "functions.h"


/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
#define mainFLASH_TASK_PRIORITY					( tskIDLE_PRIORITY + 1 )

void initLED(void);
void vTaskLED1(void * pvParameters);
void vTaskLED2(void * pvParameters);
void vStartLEDTasks(unsigned portBASE_TYPE uxPriority);

/****					MAIN FUNCTION					***/
int main(void)
{
	/* TODO - Add your application code here */


	initLED();
	RCC_ClocksTypeDef ClksFreq;
	RCC_GetClocksFreq(&ClksFreq);
	if( SYSTEM_ClockCheck() != RESET ){
		vhADC_init();
		vStartLEDTasks(mainFLASH_TASK_PRIORITY);
		vTaskStartScheduler();
	}
}
/****				END OF MAIN FUNCTION				***/

void initLED(void){
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
	// Odczytanie stanu licznika systemowego
	xLastFlashTime = xTaskGetTickCount();
	// Nieskonczona petla zadania
	for(;;)
	{
		// Wprowadzenie opoznienia 500ms
		vTaskDelayUntil( &xLastFlashTime, 500 );
		// Zmiana stanu wyprowadzenia PD8 (D1) na przeciwny
		GPIO_ToggleBits(GPIOD, GPIO_Pin_9);
	}
}

void vTaskLED2(void * pvParameters)
{
	portTickType xLastFlashTime;
	// Odczytanie stanu licznika systemowego
	xLastFlashTime = xTaskGetTickCount();
	// Nieskonczona petla zadania
	for(;;)
	{
		// Wprowadzenie opoznienia 500ms
		vTaskDelayUntil( &xLastFlashTime, 250 );
		// Zmiana stanu wyprowadzenia PD8 (D1) na przeciwny
		GPIO_ToggleBits(GPIOD, GPIO_Pin_8);
	}
}

void vStartLEDTasks(unsigned portBASE_TYPE uxPriority){
	xTaskHandle xHandleTaskLED1, xHandleTaskLED2;
	xTaskCreate(vTaskLED1, "LED1", configMINIMAL_STACK_SIZE, NULL, uxPriority, &xHandleTaskLED1);
	xTaskCreate(vTaskLED2, "LED2", configMINIMAL_STACK_SIZE, NULL, uxPriority, &xHandleTaskLED2);
}
