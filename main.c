/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : Quadcopter autopilot board main function.
**
**  Author		: Grzegorz WÓJCIK
**
**  Environment : Atollic TrueSTUDIO/STM32
**
**	MCU:		: STM32F407VET6
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f4xx.h"

#include "functions.h"
#include "I2C.h"
#include "MPU6050.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
  int i = 0;

  /* TODO - Add your application code here */
  SysTick_CLKSourceConfig(SystemCoreClock/1000);
  SYSTEM_ClockCheck();
  if ( GV_SystemReady )
  {
	  vhI2C_initRCC();
	  vhI2C_initGPIO();
	  vhI2C_initI2C1();

	  MPU6050_t MPU6050_Struct = tMPU6050_initStruct(&MPU6050_Struct);
	  thMPU6050_Init(&MPU6050_Struct, TM_MPU6050_Device_1, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_500s);

	  /* Infinite loop */
	  while (1)
	  {

		  if( GV_SystemCounter % 4 == 0 )
		  {
			  tMPU6050_ReadAll(&MPU6050_Struct);
		  }
		i++;
	  }
  }
  else
	  while(1);
}


