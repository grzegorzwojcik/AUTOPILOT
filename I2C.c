/*
 * I2C.c
 *
 *  Created on: Jul 4, 2015
 *      Author: Grzegorz WÓJCIK
 */

#include "I2C.h"

void vhI2C_initRCC(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}


void vhI2C_initGPIO(void)
{
	/* GPIO initialization */
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = pinI2C1_SCL | pinI2C1_SDA;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(portGPIO_I2C, &GPIO_InitStruct);

	/* Alternate function [I2C1] config */
	GPIO_PinAFConfig(portGPIO_I2C, pinSourceI2C1_SCL, GPIO_AF_I2C1);
	GPIO_PinAFConfig(portGPIO_I2C, pinSourceI2C1_SDA, GPIO_AF_I2C1);
}


void vhI2C_initI2C1(void)
{
	I2C_InitTypeDef I2C_InitStruct;

	/* Deinit and reset the I2C to avoid it locking up */
    I2C_DeInit(I2C1 );
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);

	/* setup SCL and SDA pins
	 * You can connect the I2C1 functions to two different
	 * pins:
	 * 1. SCL on PB8
	 * 2. SDA on PB9
	 */

	I2C_InitStruct.I2C_ClockSpeed = I2C1_clockSpeed; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = I2C1_ownAddress;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	I2C_Cmd(I2C1, ENABLE);
}
