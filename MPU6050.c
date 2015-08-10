/*
 * MPU6050.c
 *
 *  Created on: Jul 5, 2015
 *      Author: Grzegorz WÓJCIK
 */

#include "MPU6050.h"


void vMPU6050_initStruct(MPU6050_t* MPU6050_Struct)
{
	MPU6050_Struct->Address = MPU6050_I2C_ADDR | TM_MPU6050_Device_1;
	MPU6050_Struct->Acce_Mult		=	0;
	MPU6050_Struct->Accelerometer_X	=	0;
	MPU6050_Struct->Accelerometer_Y	=	0;
	MPU6050_Struct->Accelerometer_Z	=	0;
	MPU6050_Struct->Gyro_Mult		=	0;
	MPU6050_Struct->Gyroscope_X		=	0;
	MPU6050_Struct->Gyroscope_Y		=	0;
	MPU6050_Struct->Gyroscope_Z		=	0;
	MPU6050_Struct->Temperature		=	0;
	MPU6050_Struct->GyroOffsetX		=	0;
	MPU6050_Struct->GyroOffsetY		=	0;
	MPU6050_Struct->GyroOffsetZ		=	0;
	MPU6050_Struct->Gx				=	0;
	MPU6050_Struct->Gy				=	0;
	MPU6050_Struct->Gz				=	0;
}


MPU6050_Result_t tMPU6050_ReadAll(MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;

	/* Read full raw data, 14bytes */
	vI2C_ReadMulti(I2C1, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	//* DataStruct->Acce_Mult;
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);	//* DataStruct->Acce_Mult;
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);	//* DataStruct->Acce_Mult;

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data [RAW] */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Format gyroscope data [RAD/s] considering calculated OFFSET */

	DataStruct->Gx = ((DataStruct->Gyroscope_X - DataStruct->GyroOffsetX)
			* DataStruct->Gyro_Mult) * DEG2RAD;
	DataStruct->Gy = ((DataStruct->Gyroscope_Y - DataStruct->GyroOffsetY)
			* DataStruct->Gyro_Mult) * DEG2RAD;
	DataStruct->Gz = ((DataStruct->Gyroscope_Z - DataStruct->GyroOffsetZ)
			* DataStruct->Gyro_Mult) * DEG2RAD;
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}


MPU6050_Result_t thMPU6050_Init(MPU6050_t* DataStruct, MPU6050_Device_t DeviceNumber, MPU6050_Accelerometer_t AccelerometerSensitivity, MPU6050_Gyroscope_t GyroscopeSensitivity) {
	uint8_t temp;

	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;

	/* Check if device is connected */
	if (!ucI2C_IsDeviceConnected(MPU6050_I2C, DataStruct->Address)) {
		/* Return error */
		return TM_MPU6050_Result_DeviceNotConnected;
	}

	/* Check who I am */
	if (ucI2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return TM_MPU6050_Result_DeviceInvalid;
	}

	/* Wakeup MPU6050 */
	vhI2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_PWR_MGMT_1, 0x00);

	/* Config accelerometer */
	temp = ucI2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	vhI2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG, temp);

	/* Config gyroscope */
	temp = ucI2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	vhI2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG, temp);

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case TM_MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case TM_MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case TM_MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
		default:
			break;
	}

	switch (GyroscopeSensitivity) {
		case TM_MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
			break;
		case TM_MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
			break;
		case TM_MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
			break;
		case TM_MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
		default:
			break;
	}

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}


void vMPU6050_GyroCalibration(MPU6050_t* DataStruct, uint16_t Period_MS)
{
	/* Gyroscope calibration  */
	static uint16_t tmp = 0;
	static int tmp_OffsetX = 0;
	static int tmp_OffsetY = 0;
	static int tmp_OffsetZ = 0;

	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for( tmp; tmp < Period_MS; tmp++)
	{
		tMPU6050_ReadAll(DataStruct);
		tmp_OffsetX += DataStruct->Gyroscope_X;
		tmp_OffsetY += DataStruct->Gyroscope_Y;
		tmp_OffsetZ += DataStruct->Gyroscope_Z;
		vTaskDelayUntil(&xLastFlashTime, 1 );

		if( tmp >= Period_MS - 1 )
		{
			tmp_OffsetX /= Period_MS;
			tmp_OffsetY /= Period_MS;
			tmp_OffsetZ /= Period_MS;

			DataStruct->GyroOffsetX = tmp_OffsetX;
			DataStruct->GyroOffsetY = tmp_OffsetY;
			DataStruct->GyroOffsetZ = tmp_OffsetZ;
		}
	}
}


/****				TASKS				****/
void vTaskI2C_MPU6050(void * pvParameters)
{
	/* Local variables. */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	/* Initial quaternion values */
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;

	IMU_t IMU_Struct;
	vIMU_initStruct(&IMU_Struct);

	/* MPU6050 initialization */
	MPU6050_t MPU6050_Struct;
	vMPU6050_initStruct(&MPU6050_Struct);
	thMPU6050_Init(&MPU6050_Struct,
			TM_MPU6050_Device_1,
			TM_MPU6050_Accelerometer_2G,
			TM_MPU6050_Gyroscope_250s);

	/* GyroCalibration: setting gyroscope offset values */
	vMPU6050_GyroCalibration(&MPU6050_Struct, 2000);

	for(;;)
	{
		/* Read and store data to the MPU6050_Struct via I2C function */
		tMPU6050_ReadAll(&MPU6050_Struct);

		/* Madgwick filter, obtain quaternions and obtain Yaw Pitch & Roll angles */
		vIMU_filterUpdate(MPU6050_Struct.Gx, MPU6050_Struct.Gy, MPU6050_Struct.Gz,
			MPU6050_Struct.Accelerometer_X,
			MPU6050_Struct.Accelerometer_Y,
			MPU6050_Struct.Accelerometer_Z);

		/* Calculate Aerospace sequence Euler angles & update IMU_Structure */
		vIMU_getAngles(&IMU_Struct, MPU6050_Struct.Gz);

		xQueueSend(xQueueUART_1xIMU_t, &IMU_Struct, 0);
		/*		100 Hz loop / 10ms delay	*/
		vTaskDelayUntil(&xLastFlashTime, 10 );
	}
}


void vStartI2C_MPU6050Task(unsigned portBASE_TYPE uxPriority)
{
	/* Creating task */
	xTaskHandle xHandleTaskI2C_MPU6050;
	xTaskCreate( vTaskI2C_MPU6050, "I2C_MPU6050", configMINIMAL_STACK_SIZE,
			NULL, uxPriority, &xHandleTaskI2C_MPU6050 );
}
