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
	  uint8_t temp;

				/* Check if device is connected */
				if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, MPU6050_Struct.Address )) {
					/* Return error */
					return TM_MPU6050_Result_DeviceNotConnected;
				}

				/* Check who I am */
				if (TM_I2C_Read(MPU6050_I2C, MPU6050_Struct.Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
					/* Return error */
					return TM_MPU6050_Result_DeviceInvalid;
				}

				/* Wakeup MPU6050 */
				TM_I2C_Write(MPU6050_I2C, MPU6050_Struct.Address, MPU6050_PWR_MGMT_1, 0x00);

				/* Config accelerometer */
				temp = TM_I2C_Read(MPU6050_I2C, MPU6050_Struct.Address, MPU6050_ACCEL_CONFIG);
				temp = (temp & 0xE7) | (uint8_t)MPU6050_ACCE_SENS_8 << 3;
				TM_I2C_Write(MPU6050_I2C, MPU6050_Struct.Address, MPU6050_ACCEL_CONFIG, temp);

				/* Config gyroscope */
				temp = TM_I2C_Read(MPU6050_I2C, MPU6050_Struct.Address, MPU6050_GYRO_CONFIG);
				temp = (temp & 0xE7) | (uint8_t)MPU6050_GYRO_SENS_500 << 3;
				TM_I2C_Write(MPU6050_I2C, MPU6050_Struct.Address, MPU6050_GYRO_CONFIG, temp);

				MPU6050_Struct.Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
				MPU6050_Struct.Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;

	  /* Infinite loop */
				uint8_t data[6];

	  while (1)
	  {

		  if( GV_SystemCounter % 500 == 0 )
		  {
			  TM_MPU6050_ReadAll(&MPU6050_Struct);
		  }
		i++;
	  }
  }
  else
	  while(1);
}


