/*
 * MPU6050.h
 *
 *  Created on: Jul 5, 2015
 *      Author: Grzegorz W�JCIK
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stddef.h>
#include "stm32f4xx.h"

#include "functions.h"
#include "IMU.h"
#include "I2C.h"

#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"

/* Default I2C used */
#ifndef MPU6050_I2C
#define	MPU6050_I2C					I2C1
#define MPU6050_I2C_PINSPACK		TM_I2C_PinsPack_1
#endif

/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK			400000
#endif

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in �/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

/**
 * MPU6050 can have 2 different slave addresses,
 * depends on it's input AD0 pin [SB1 on Autopilot PCB]
 *
 * Parameters:
 *     - TM_MPU6050_Device_0:
 *         AD0 pin is set to low
 *     - TM_MPU6050_Device_1:
 *         AD0 pin is set to high
 */
typedef enum {
    TM_MPU6050_Device_0 = 0,
    TM_MPU6050_Device_1 = 0x02
} MPU6050_Device_t;

/**
 * Result enumeration
 *
 * ParameterS:
 *     - TM_MPU6050_Result_Ok:
 *         Everything OK
 *     - TM_MPU6050_Result_DeviceNotConnected:
 *         There is no device with valid slave address
 *     - TM_MPU6050_Result_DeviceInvalid:
 *         Connected device with address is not MPU6050
 */
typedef enum {
    TM_MPU6050_Result_Ok = 0x00,
    TM_MPU6050_Result_DeviceNotConnected,
    TM_MPU6050_Result_DeviceInvalid
} MPU6050_Result_t;

/**
 * Set parameters for accelerometer range
 *
 * Parameters:
 *     - TM_MPU6050_Accelerometer_2G:
 *         Range is +- 2G
 *     - TM_MPU6050_Accelerometer_4G:
 *         Range is +- 4G
 *     - TM_MPU6050_Accelerometer_8G:
 *         Range is +- 8G
 *     - TM_MPU6050_Accelerometer_16G:
 *         Range is +- 16G
 */
typedef enum {
    TM_MPU6050_Accelerometer_2G = 0x00,
    TM_MPU6050_Accelerometer_4G = 0x01,
    TM_MPU6050_Accelerometer_8G = 0x02,
    TM_MPU6050_Accelerometer_16G = 0x03
} MPU6050_Accelerometer_t;

/**
 * Set parameters for gyroscope range
 *
 * Parameters:
 *     - TM_MPU6050_Gyroscope_250s:
 *         Range is +- 250�/s
 *     - TM_MPU6050_Gyroscope_500s:
 *         Range is +- 500�/s
 *     - TM_MPU6050_Gyroscope_1000s:
 *         Range is +- 1000�/s
 *     - TM_MPU6050_Gyroscope_2000s:
 *         Range is +- 20000�/s
 */
typedef enum {
    TM_MPU6050_Gyroscope_250s = 0x00,
    TM_MPU6050_Gyroscope_500s = 0x01,
    TM_MPU6050_Gyroscope_1000s = 0x02,
    TM_MPU6050_Gyroscope_2000s = 0x03
} MPU6050_Gyroscope_t;

typedef struct {
    /* Private */
    uint8_t Address;
    float Gyro_Mult;
    float Acce_Mult;
    /* Public */
    int16_t Accelerometer_X;		//Raw data
    int16_t Accelerometer_Y;		//Raw data
    int16_t Accelerometer_Z;		//Raw data
    int16_t Gyroscope_X;			//Raw data
    int16_t Gyroscope_Y;			//Raw data
    int16_t Gyroscope_Z;			//Raw data
    int16_t GyroOffsetX;			//Calculated offset
    int16_t GyroOffsetY;			//Calculated offset
    int16_t GyroOffsetZ;			//Calculated offset
    float Temperature;				//�C
    float Gx;						//Rad/s
    float Gy;						//Rad/s
    float Gz;						//Rad/s
} MPU6050_t;

MPU6050_t tMPU6050_initStruct(MPU6050_t* MPU6050_Struct);
MPU6050_Result_t tMPU6050_ReadAll(MPU6050_t* DataStruct);
MPU6050_Result_t thMPU6050_Init(MPU6050_t* DataStruct, MPU6050_Device_t DeviceNumber,
		MPU6050_Accelerometer_t AccelerometerSensitivity, MPU6050_Gyroscope_t GyroscopeSensitivity);
void vMPU6050_GyroCalibration(MPU6050_t* DataStruct, uint16_t Period_MS);

/*			 Tasks 			*/
void vTaskI2C_MPU6050(void * pvParameters);
void vStartI2C_MPU6050Task(unsigned portBASE_TYPE uxPriority);


#endif /* MPU6050_H_ */
