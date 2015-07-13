/*
 * UART.c
 *
 *  Created on: Jul 7, 2015
 *      Author: Grzegorz WÓJCIK
 */

#include "UART.h"

/* Hardware related functions. */

void vhUART_initNaviRCC(void)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
}


void vhUART_initNaviGPIO()
{
	/* STM32F407VGT6
	 * USART2:
	 * TX: PD5
	 * RX: PD6 */

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = pinUART_2TX | pinUART_2RX;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOD, pinSourceUART_2RX, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, pinSourceUART_2TX, GPIO_AF_USART2);
}


void vhUART_initNaviUART2(void)
{
/* STM32F407VGT6/EGT6
	 * USART2:
	 * TX: PD5
	 * RX: PD6 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 	// Enabling peripherial clock for USART2

	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate = baudRateNAVI;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 	// we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);


	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 			// enable the USART2 receive and transmit interrupt

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;// this sets the priority group of the USART1 interrupts
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStruct);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	USART_Cmd(USART2, ENABLE);	// Enabling the complete USART1 peripherial
}

/* Software related functions. */

uint8_t ucUART_calculateCRC(char *Word, char StartChar ,uint8_t Length)
{
	uint8_t XOR = 0;	// set CRC to 0
	uint8_t i = 0;

	for( XOR = 0, i = 0; i < Length; i++ )
	{
		if(Word[i] == '*')
			break;
		if(Word[i] != StartChar)
			XOR ^= Word[i];
	}
	return XOR;
}


void vUART_puts(USART_TypeDef* USARTx, volatile char *s)
{
	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}



/*-----------------------------------------------------------
* @brief Function Name  : vUART_ClearBuffer
* @brief Description    : Clears global (volatile) UART buffer, one of the selected by @param
* @param UART_buffer	: one of the UARTbuffer_t typedef.
*/
void vUART_ClearBuffer(UARTbuffer_t	UART_buffer){
	uint8_t tmp = 0;

	switch (UART_buffer) {
		case UART_NaviBufferRECEIVE:
			for (tmp = 0; tmp < NAVI_BUFFER_LENGTH; tmp++)
				GV_bufferNAVIrec[tmp] = 0;
			break;

		case UART_NaviBufferSEND:
			for (tmp = 0; tmp < NAVI_BUFFER_LENGTH; tmp++)
				GV_bufferNAVIsend[tmp] = 0;
			break;

		case UART_GpsBufferRECEIVE:
			for (tmp = 0; tmp < GPS_BUFFER_LENGTH; tmp++)
				GV_bufferGPSrec[tmp] = 0;
			break;

		case UART_GpsBufferSEND:
			for (tmp = 0; tmp < GPS_BUFFER_LENGTH; tmp++)
				GV_bufferGPSsend[tmp] = 0;
			break;

		default:
			break;
	}
}

/****				TASKS				****/

/*-----------------------------------------------------------
* @brief Task Name 		: vTaskUART_NAVIimu
* @brief Description    : This task is responsible for sending IMU data frames to the
* 							 NAVIGATION & FAULT INJECTION BOARD by managing a semaphore
*/

	//MAKER PLOT FRAME
	/*sprintf(GV_bufferNAVIsend, "%i %i %i \n\r",
			MPU6050_Structure.Gyroscope_X,
			MPU6050_Structure.Gyroscope_Y,
			MPU6050_Structure.Gyroscope_Z);*/

/*           //QUADROPLOT FRAME
sprintf(GV_bufferNAVIsend, "a%i\nb%i\nc%i\nd%i\n",
		IMU_Struct.Yaw,
		IMU_Struct.Pitch,
		IMU_Struct.Roll,
		IMU_Struct.GyroZ);
vUART_puts(USART2, GV_bufferNAVIsend);*/

void vTaskUART_NAVIimu(void * pvParameters)
{
	/* Local variables. */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	IMU_t IMU_Struct;
	vIMU_initStruct(&IMU_Struct);

	/*		 25 Hz task [40 ms delay]		 */
	for(;;){
		vTaskDelayUntil( &xLastFlashTime, 40 );

		/* Receive IMU_Structure */
		xQueueReceive(xQueueUART_1xIMU_t, &IMU_Struct, 10);

		/* Sharing xSemaphoreUART_NAVITX MUTEX with vTaskUART_NAVIsns task */
		if( xSemaphoreTake(xSemaphoreUART_NAVITX, 40))
		{
			/* Clear buffer before updating it */
			vUART_ClearBuffer(UART_NaviBufferSEND);

			/* Send data to the Navigation & Fault injection board */
			char tmp_buffer[NAVI_BUFFER_LENGTH] = {0};
			sprintf(tmp_buffer, "%c,3,%i,%i,%i,%i,*", NAVI_DF_CHAR,
					IMU_Struct.Yaw,
					IMU_Struct.Pitch,
					IMU_Struct.Roll,
					IMU_Struct.GyroZ);
				// Add calculated CRC to this string.
			sprintf(GV_bufferNAVIsend, "%s%i\n\r", tmp_buffer,
					ucUART_calculateCRC(tmp_buffer, NAVI_DF_CHAR, NAVI_BUFFER_LENGTH) );
			vUART_puts(USART2, GV_bufferNAVIsend);

			xSemaphoreGive(xSemaphoreUART_NAVITX);
		}

	}
}


/*-----------------------------------------------------------
* @brief Task Name 		: vTaskUART_NAVIsns
* @brief Description    : This task is responsible for sending SENSOR data frames to the
* 							 NAVIGATION & FAULT INJECTION BOARD by managing a semaphore
*/
void vTaskUART_NAVIsns(void * pvParameters)
{
	/* Local variables. */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	SENSOR_t SENSOR_Struct;
	vSENSOR_initStruct(&SENSOR_Struct);

	/*		 10 Hz task [100 ms delay]		 */
	for(;;){
		vTaskDelayUntil( &xLastFlashTime, 100 );

		/* Receive IMU_Structure */
		xQueueReceive(xQueueUART_1xSENSOR_t, &SENSOR_Struct, 100);

		/* Sharing xSemaphoreUART_NAVITX MUTEX with vTaskUART_NAVIimu task */
		if( xSemaphoreTake(xSemaphoreUART_NAVITX, 0))
		{
			/* Clear buffer before updating it */
			vUART_ClearBuffer(UART_NaviBufferSEND);

			/* Send data to the Navigation & Fault injection board */
			char tmp_buffer[NAVI_BUFFER_LENGTH] = {0};
			sprintf(tmp_buffer, "%c,4,%i,%i,0,0,*", NAVI_DF_CHAR,
					SENSOR_Struct.PS_Voltage,
					SENSOR_Struct.IR_Sensor);
				// Add calculated CRC to this string.
			sprintf(GV_bufferNAVIsend, "%s%i\n\r", tmp_buffer,
					ucUART_calculateCRC(tmp_buffer, NAVI_DF_CHAR, NAVI_BUFFER_LENGTH) );
			vUART_puts(USART2, GV_bufferNAVIsend);

			/* Allow putting another IMU_Structure to the xQueueUART_1xIMU_t */
			xSemaphoreGive(xSemaphoreUART_NAVITX);
		}
	}
	/*	sprintf(GV_bufferNAVIsend, "a%i\nb%i\n",
				SENSOR_Struct.PS_Voltage,
				SENSOR_Struct.IR_Sensor);
		vUART_puts(USART2, GV_bufferNAVIsend); // QUADROPLOT DATA FRAME*/
}

void vStartUART_NAVITask(unsigned portBASE_TYPE uxPriority)
{
	/* Creating semaphore related to this task */
	xSemaphoreUART_NAVIRX = NULL;
	xSemaphoreUART_NAVIRX = xSemaphoreCreateBinary();

	xSemaphoreUART_NAVITX = NULL;
	xSemaphoreUART_NAVITX = xSemaphoreCreateMutex();

	/* Creating queue which is responsible for handling IMU_t typedef data */
	xQueueUART_1xIMU_t = xQueueCreate(1, sizeof(IMU_t) );
	/* Creating queue which is responsible for handling SENSOR_t typedef data */
	xQueueUART_1xSENSOR_t = xQueueCreate(1, sizeof(SENSOR_t) );


	/* Creating task [sending IMU data frames] 25 Hz task */
	xTaskHandle xHandleTaskUART_NAVIimu;
	xTaskCreate( vTaskUART_NAVIimu, "UART_NAVI_TXimu", configMINIMAL_STACK_SIZE,
			NULL, uxPriority, &xHandleTaskUART_NAVIimu );

	/* Creating task [sending SENSOR data frames] 10 Hz task*/
	xTaskHandle xHandleTaskUART_NAVIsns;
	xTaskCreate( vTaskUART_NAVIsns, "UART_NAVI_TXsns", configMINIMAL_STACK_SIZE,
			NULL, uxPriority, &xHandleTaskUART_NAVIsns );
}


/* Interrupt handlers */

void USART2_IRQHandler(void){

	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
		static uint8_t cnt = 0; 	// String length
		char t = USART2->DR; 		// Received character from USART1 data register is saved in t
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
	}

	if( USART_GetITStatus(USART2, USART_IT_TXE) ){
		static uint8_t cnt = 0; 	// String length
	}
}
