/*
 * UART.h
 *
 *  Created on: Jul 7, 2015
 *      Author: Grzegorz WÓJCIK
 */

#ifndef UART_H_
#define UART_H_

#include "functions.h"
#include "stm32f4xx.h"

#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"

			/* Hardware port definitions. */
#define portGPIO_UART_NAVI			GPIOD
#define portGPIO_UART_GPS			GPIOB

			/* Hardware pin definitions. */
#define pinUART_1TX			GPIO_Pin_6		//UART_1TX connected to GPS Rx
#define pinUART_1RX			GPIO_Pin_7		//UART_1RX connected to GPS Tx
#define pinUART_2TX			GPIO_Pin_5		//UART_2TX connected to NAVI Rx
#define pinUART_2RX			GPIO_Pin_6		//UART_2RX connected to NAVI Tx


#endif /* UART_H_ */
