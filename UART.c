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



/****				TASKS				****/
void vTaskUART_NAVI(void * pvParameters)
{
	/* Local variables. */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;){
		/*		 500ms delay.	 */
		vTaskDelayUntil( &xLastFlashTime, 500 );
		vUART_puts(USART2, "test\n\r");
	}
}


void vStartUART_NAVITask(unsigned portBASE_TYPE uxPriority)
{
	/* Creating semaphore related to this task */
	xSemaphoreUART_NAVIRX = NULL;
	xSemaphoreUART_NAVIRX = xSemaphoreCreateBinary();

	xSemaphoreUART_NAVITX = NULL;
	xSemaphoreUART_NAVITX = xSemaphoreCreateBinary();

	/* Creating task */
	xTaskHandle xHandleTaskUART_NAVI;
	xTaskCreate( vTaskUART_NAVI, "UART_NAVI", configMINIMAL_STACK_SIZE,
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
