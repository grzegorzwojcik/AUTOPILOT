/*
 * I2C.h
 *
 *  Created on: Jul 4, 2015
 *      Author: Grzegorz WÓJCIK
 */

#ifndef I2C_H_
#define I2C_H_

#include <stddef.h>
#include "stm32f4xx.h"
#include "functions.h"

//#include "FreeRTOS_Source/include/FreeRTOS.h"
//#include "FreeRTOS_Source/include/task.h"
//#include "FreeRTOS_Source/include/semphr.h"

			/* Hardware port definitions. */
#define portGPIO_I2C			GPIOB

			/* Hardware pin definitions. */
#define pinI2C1_SCL				GPIO_Pin_8
#define pinSourceI2C1_SCL		GPIO_PinSource8
#define pinI2C1_SDA				GPIO_Pin_9
#define pinSourceI2C1_SDA		GPIO_PinSource9

			/* I2C1 setuo definitions. */
#define I2C1_ownAddress			0x00
#define I2C1_clockSpeed			100000	/* Fast mode */

void vhI2C_initRCC(void);
void vhI2C_initGPIO(void);
void vhI2C_initI2C1(void);

#endif /* I2C_H_ */
