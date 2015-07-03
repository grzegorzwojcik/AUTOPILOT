/*
 * ADC.c
 *
 *  Created on: Jul 3, 2015
 *      Author: Grzegorz WÓJCIK
 */

#include "ADC.h"

/*-----------------------------------------------------------
* @brief Function Name  : vhADC_initRCC
* @brief Description    : Initializes RCC's for GPIOC, ADC1, DMA2, TIM1
*/
void vhADC_initRCC(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
}


/*-----------------------------------------------------------
* @brief Function Name  : vhADC_initGPIO
* @brief Description    : Initializes ADC connected to the Supply Voltage.
* 							Used predefines: pinADC_LIPO, pinADC_IR, portGPIO_ADC
*/
void vhADC_initGPIO(void){
	/* GPIO initialization */
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = pinADC_LIPO | pinADC_IR;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(portGPIO_ADC, &GPIO_InitStruct);
}


/*-----------------------------------------------------------
* @brief Function Name  : vhADC_initNVIC
* @brief Description    : Initializes NVIC for ADC1
*/
void vhADC_initNVIC(void){
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM1 Update interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM1 Capture interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*-----------------------------------------------------------
* @brief Function Name  : vhADC_initTIM1
* @brief Description    : Initializes TIM1 to give interrupt with specified
* 							frequency.
*/
void vhADC_initTIM1(void){
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_InitStructure.TIM_Prescaler = 4200-1;				//84MHz : 42000 = 20000 Hz
	TIM_InitStructure.TIM_Period = 100;						//20000 Hz : 100 = 200 Hz
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_InitStructure);

	// Channel 1 configuration
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

	// Clear pending interrupt bits
	TIM_ClearITPendingBit( TIM1, TIM_IT_Update | TIM_IT_CC1 );
	TIM_ITConfig( TIM1, TIM_IT_Update | TIM_IT_CC1, ENABLE );
	TIM_Cmd( TIM1, ENABLE );
}


/*-----------------------------------------------------------
* @brief Function Name  : vhADC_initDMA
* @brief Description    : Initializes DMA2
*/
void vhADC_initDMA(void){
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA2_Stream0);	// Deinitializes DMAy_Streamx to default functions

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_CONVERTED_VALUES[0];
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = bufferSize_ADC; 								// Count of 16-bit words
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// auto increment of periph disable
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// auto increment of memory enable
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	/* Enable DMA Stream Half / Transfer Complete interrupt */
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);
}


/*-----------------------------------------------------------
* @brief Function Name  : vhADC_initADC
* @brief Description    : Initializes ADC
*/
void vhADC_initADC(void){
	/* ADC Common initialization */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;		// 84MHz / prescaler(6) = 14MHz (max 30 OR 36... idk)
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			// Enable, because we want to measure more than 1 channel
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_RisingFalling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADCx regular channel configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 2, ADC_SampleTime_28Cycles);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADCx DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADCx */
	ADC_Cmd(ADC1, ENABLE);

	ADC_SoftwareStartConv(ADC1);
}


/*-----------------------------------------------------------
* @brief Function Name  : vhADC_init
* @brief Description    : Executes initialization functions in proper order
*/
void vhADC_init(void){
	vhADC_initRCC();
	vhADC_initGPIO();
	vhADC_initNVIC();
	vhADC_initTIM1();
	vhADC_initDMA();		// ADC_initDMA() should be executed AFTER ADC_initNVIC() and BEFORE ADC_initADC()
	vhADC_initADC();
}

/****				TASKS				****/
void vTaskADC_VoltPwr(void * pvParameters)
{
	/* Local variables. */
	static uint32_t voltage = 0;
	static uint16_t multiplier = 4400;

	for(;;){
		if( xSemaphoreTake(xSemaphoreADC_VoltPwr, 100 ) == pdTRUE)
		{
			uint32_t tmp = ADC_CONVERTED_VALUES[0] * multiplier;
			voltage = tmp*3.3/4095;
		}
	}
}


void vStartADC_VoltPwrTask(unsigned portBASE_TYPE uxPriority)
{
	/* Creating semaphore related to this task */
	xSemaphoreADC_VoltPwr = NULL;
	xSemaphoreADC_VoltPwr = xSemaphoreCreateBinary();

	/* Creating task */
	xTaskHandle xHandleTaskADC_VoltPwr;
	xTaskCreate( vTaskADC_VoltPwr, "ADC_VoltPwr", configMINIMAL_STACK_SIZE,
			NULL, uxPriority, &xHandleTaskADC_VoltPwr );
}


/****			TIMER HANDLERS			****/
void TIM1_CC_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if( TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET )
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
		xSemaphoreGiveFromISR( 	xSemaphoreADC_VoltPwr,
								&xHigherPriorityTaskWoken );
		ADC_SoftwareStartConv(ADC1);
	}
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET )
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
