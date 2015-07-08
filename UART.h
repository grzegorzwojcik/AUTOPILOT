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
#include "stddef.h"
#include "MPU6050.h"

#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"

			/* Hardware port definitions. */
#define portGPIO_UART_NAVI			GPIOD
#define portGPIO_UART_GPS			GPIOB

			/* Hardware pin definitions. */
#define pinUART_1TX					GPIO_Pin_6		//UART_1TX connected to the GPS Rx
#define pinUART_1RX					GPIO_Pin_7		//UART_1RX connected to the GPS Tx
#define pinUART_2TX					GPIO_Pin_5		//UART_2TX connected to the NAVI Rx
#define pinUART_2RX					GPIO_Pin_6		//UART_2RX connected to the NAVI Tx

#define pinSourceUART_1TX			GPIO_PinSource6
#define pinSourceUART_1RX			GPIO_PinSource7
#define pinSourceUART_2TX			GPIO_PinSource5
#define pinSourceUART_2RX			GPIO_PinSource6

			/* Software UART definitions. */
#define baudRateGPS					9600
#define baudRateNAVI				115200
#define NAVI_BUFFER_LENGTH			30
#define NAVI_DF_CHAR				'#'				// Data frame starting character
#define GPS_BUFFER_LENGTH			50
#define GPS_DF_CHAR					'$'				// Data frame starting character
volatile char GV_bufferNAVIsend[NAVI_BUFFER_LENGTH];
volatile char GV_bufferNAVIrec[NAVI_BUFFER_LENGTH];
volatile char GV_bufferGPSsend[GPS_BUFFER_LENGTH];
volatile char GV_bufferGPSrec[GPS_BUFFER_LENGTH];


/**
 * Set parameters for clearing buffer function
 */
typedef enum {
    UART_NaviBufferSEND 	= 0x00,
    UART_NaviBufferRECEIVE 	= 0x01,
    UART_GpsBufferSEND 		= 0x02,
    UART_GpsBufferRECEIVE 	= 0x03
} UARTbuffer_t;

/* Hardware related functions. */
void vhUART_initNaviRCC(void);
void vhUART_initNaviGPIO(void);
void vhUART_initNaviUART2(void);

/* Hardware related functions. */
void vhUART_initGpsRCC(void);
void vhUART_initGpsGPIO(void);
void vhUART_initGpsUART2(void);

/* Software related functions. */
uint8_t ucUART_calculateCRC(char *Word, char StartChar ,uint8_t Length);
void vUART_puts(USART_TypeDef* USARTx, volatile char *s);
void vUART_ClearBuffer(UARTbuffer_t	UART_buffer);

/*			 Tasks 			*/
void vTaskUART_NAVI(void * pvParameters);
void vStartUART_NAVITask(unsigned portBASE_TYPE uxPriority);

#endif /* UART_H_ */
