/*
 * MPU6050.c
 *
 *  Created on: Jul 5, 2015
 *      Author: Grzegorz WÓJCIK
 */

#include "MPU6050.h"

MPU6050_t tMPU6050_initStruct(MPU6050_t* MPU6050_Struct)
{
	//MPU6050_t MPU6050_Struct;
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

	return *MPU6050_Struct;
}


TM_MPU6050_Result_t TM_MPU6050_ReadAll(MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;

	/* Read full raw data, 14bytes */
	TM_I2C_ReadMulti(I2C1, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}
