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
#include "I2C.h"
#include "functions.h"

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

		/* Check if device is connected */
		if (!TM_I2C_IsDeviceConnected(I2C1, 0xD2)) {
			/* Return error */
			return TM_MPU6050_Result_DeviceNotConnected;
		}

	  /* Infinite loop */
	  while (1)
	  {
		i++;
	  }
  }
  else
	  while(1);
}


