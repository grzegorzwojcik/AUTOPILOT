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
* @brief Task Name 		: vTaskUART_NAVI
* @brief Description    : This task is responsible for sending proper data frames to the
* 							 NAVIGATION & FAULT INJECTION BOARD by managing a semaphore
*/
void vTaskUART_NAVI(void * pvParameters)
{
	/* Local variables. */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();
	MPU6050_t MPU6050_Structure = tMPU6050_initStruct(&MPU6050_Structure);

	for(;;){
		/*		 250ms delay.	 */
		vTaskDelayUntil( &xLastFlashTime, 20 );
		xQueueReceive(xQueueUART_2xMPU_t, &MPU6050_Structure, 0);
		xSemaphoreGive(xSemaphoreUART_NAVITX);

		vUART_ClearBuffer(UART_NaviBufferSEND);
		char tmp_buffer[NAVI_BUFFER_LENGTH] = {0};
		/* Collecting temporary string */
		/*sprintf(tmp_buffer, "%c,8,%i,%i,%i,0,*", NAVI_DF_CHAR,
				MPU6050_Structure.Gyroscope_X,
				MPU6050_Structure.Gyroscope_Y,
				MPU6050_Structure.Gyroscope_Z);
		 Adding calculated CRC to this string
		sprintf(GV_bufferNAVIsend, "%s%i\n\r", tmp_buffer,
				ucUART_calculateCRC(tmp_buffer, NAVI_DF_CHAR, NAVI_BUFFER_LENGTH) );*/
		static float pitch = 0;
		static float yaw = 0;
		static float roll = 0;
		//MAKER PLOT FRAME
		/*sprintf(GV_bufferNAVIsend, "%i %i %i \n\r",
				MPU6050_Structure.Gyroscope_X,
				MPU6050_Structure.Gyroscope_Y,
				MPU6050_Structure.Gyroscope_Z);*/

		sprintf(GV_bufferNAVIsend, "a%i\nb%i\nc%i\n",
				(int) yaw,
				(int) pitch,
				(int) roll);
		vUART_puts(USART2, GV_bufferNAVIsend);


		/*sprintf(GV_bufferNAVIsend,"{GyroX, T, %i}{GyroY, T, %i}",
				MPU6050_Structure.Gyroscope_X,
				MPU6050_Structure.Gyroscope_Y);
		vUART_puts(USART2, GV_bufferNAVIsend);*/
		float gx, gy, gz; // estimated gravity direction

		/*yaw 	= atan2f(2 * q1 * q2 - 2 * q0 * q3 , 2 * q0 * q0 + 2 * q1 * q1 - 1 );
		roll 	= -asinf( 2 * q1 * q3 + 2 * q0 * q2 );
		pitch 	= atan2f(2 * q2 * q3 - 2 * q0 * q1 , 2 * q0 * q0 + 2 * q3 * q3 - 1 );*/
		/*pitch *= 57.295;	// * PI/180
		yaw   *= 57.295;	// * PI/180
		roll  *= 57.295;	// * PI/180*/

	    gx = 2 * (q1*q3 - q0*q2);
	    gy = 2 * (q0*q1 + q2*q3);
	    gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	    yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1 * q1 - 1) * 180/M_PI;
	    pitch = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
	    roll = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;

	}
}


void vStartUART_NAVITask(unsigned portBASE_TYPE uxPriority)
{
	/* Creating semaphore related to this task */
	xSemaphoreUART_NAVIRX = NULL;
	xSemaphoreUART_NAVIRX = xSemaphoreCreateBinary();

	xSemaphoreUART_NAVITX = NULL;
	xSemaphoreUART_NAVITX = xSemaphoreCreateBinary();


	/* Creating queue which is responsible for handling MPU6050_t typedef data */
	xQueueUART_2xMPU_t = xQueueCreate(2, sizeof(MPU6050_t) );

	/* Creating task  */
	xTaskHandle xHandleTaskUART_NAVI;
	xTaskCreate( vTaskUART_NAVI, "UART_NAVI_TX", configMINIMAL_STACK_SIZE,
			NULL, uxPriority, &xHandleTaskUART_NAVI );
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
