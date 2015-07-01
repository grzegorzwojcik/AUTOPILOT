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

//#include "functions.h"

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
void initLED(void);
void vTaskLED(void * pvParameters);

int main(void)
{

  /* TODO - Add your application code here */
  initLED();

  uint8_t ret;
  ret = xTaskCreate(vTaskLED, "LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  vTaskStartScheduler();
  /* Infinite loop */
  while (1)
  {

  }
}

void initLED(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_initStruct;
	GPIO_initStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_initStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_initStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_initStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_initStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_initStruct);

	GPIO_SetBits(GPIOD, GPIO_Pin_12);
}

void vTaskLED(void * pvParameters)
{
	portTickType xLastFlashTime;

	// Odczytanie stanu licznika systemowego
	xLastFlashTime = xTaskGetTickCount();
	// Nieskonczona petla zadania
	for(;;)
	{
		// Wprowadzenie opoznienia 500ms
		vTaskDelayUntil( &xLastFlashTime, 500/portTICK_RATE_MS );
		// Zmiana stanu wyprowadzenia PC6 (LD1) na przeciwny
		GPIO_ToggleBits(GPIOD, GPIO_Pin_8);
	}
}

