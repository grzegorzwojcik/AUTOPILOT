/*
 * ADC.h
 *
 *  Created on: Jul 3, 2015
 *      Author: Grzegorz WÓJCIK
 */

#ifndef ADC_H_
#define ADC_H_

#include <stddef.h>
#include "stm32f4xx.h"
#include "functions.h"

#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"


			/* Hardware port definitions. */
#define portGPIO_ADC			GPIOC

			/* Hardware pin definitions. */
#define pinADC_LIPO				GPIO_Pin_0
#define pinADC_MXR9500_Z		GPIO_Pin_1
#define pinADC_MXR9500_X		GPIO_Pin_2
#define pinADC_MXR9500_Y		GPIO_Pin_3
#define pinADC_IR				GPIO_Pin_4

#define bufferSize_ADC			2
volatile uint16_t ADC_CONVERTED_VALUES[bufferSize_ADC];


			/* Initialization functions. */
void vhADC_initRCC(void);
void vhADC_initGPIO(void);
void vhADC_initNVIC(void);
void vhADC_initTIM1(void);
void vhADC_initDMA(void);
void vhADC_initADC(void);
void vhADC_init(void);		// All init functions above within 1 function


			/*			 Tasks 			*/
void vTaskADC_PwrSup(void);
void vStartADC_VoltPwrTask(unsigned portBASE_TYPE uxPriority);


#endif /* ADC_H_ */
