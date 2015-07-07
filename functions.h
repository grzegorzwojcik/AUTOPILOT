/*
 * functions.h
 *
 *  Created on: Jun 29, 2015
 *      Author: Grzegorz W�JCIK
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stddef.h>
#include "stm32f4xx.h"

#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"

/*=====================FreeRTOS SECTION=====================*/

			/* 		Task priorities.		 */
#define tskLED_FLASH_PRIORITY					( tskIDLE_PRIORITY )
#define tskADC_TASK_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define tskUART_NAVI_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define tskI2C_MPU6050_PRIORITY					( tskIDLE_PRIORITY + 3 )

			/* 			Semaphores.			 */
SemaphoreHandle_t xSemaphoreADC_VoltPwr;
SemaphoreHandle_t xSemaphoreUART_NAVITX;
SemaphoreHandle_t xSemaphoreUART_NAVIRX;

			/* 			Queues.			 */
QueueHandle_t xQueueUART_2xMPU_t;
/*===========================================================*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
volatile uint8_t GV_SystemReady;	// 0 = NOT READY, 1 = OK
volatile uint16_t GV_SystemCounter;	// System time counter, incrementing within Systick handler



/* Private functions ---------------------------------------------------------*/
FlagStatus SYSTEM_ClockCheck(void);
void vhLED_initGPIO(void);
void vTaskLED1(void * pvParameters);
void vTaskLED2(void * pvParameters);
void vStartLEDTasks(unsigned portBASE_TYPE uxPriority);
#endif /* FUNCTIONS_H_ */


