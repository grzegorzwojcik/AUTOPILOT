/*
 * functions.h
 *
 *  Created on: Jun 29, 2015
 *      Author: Grzegorz WÓJCIK
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stddef.h>
#include <stdlib.h>
#include "stm32f4xx.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
volatile uint8_t GV_SystemReady;	// 0 = NOT READY, 1 = OK
volatile uint16_t GV_SystemCounter;	// System time counter, incrementing within Systick handler



/* Private functions ---------------------------------------------------------*/
void PLL_Configurattion(void);
void SYSTEM_ClockCheck(void);

#endif /* FUNCTIONS_H_ */


