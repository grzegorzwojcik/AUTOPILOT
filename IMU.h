/*
 * FUSION.h
 *
 *  Created on: Jul 10, 2015
 *      Author: Grzegorz WÓJCIK
 *
 *      IMU data fusion is based on:
 *      "An efficient orientation filter for inertial and intertial/magnetic sensor arrays"
 *      by Sebastian O.H. Madgwick, using MPU6050 gyroscope & accelerometer sensor.
 *
 */

#ifndef FUSION_H_
#define FUSION_H_

// Math library required for sqrt
#include <stddef.h>
#include "stm32f4xx.h"
#include <math.h>

			/*  System constants. */

/*Sampling period in seconds (shown as 1 ms)*/
#define sampleFreq			100.0f					// sample frequency in Hz

#define beta				0.5f					// 2 * proportional gain
#define	DEG2RAD				0.017453f				// equals PI/180
#define	RAD2DEG				57.29577f				// equals 180/PI

			/*  Global variables. */

/*Estimated orientation quaternion elements with initial conditions*/
volatile float q0, q1, q2, q3;

			/*  Private typedefs. */
typedef struct {
    /* Private */
	int16_t Yaw;
	int16_t Pitch;
	int16_t Roll;
	int16_t GyroZ;
} IMU_t;

/*-----------------------------------------------------------
* @brief Function Name  : FUSION_filterUpdate
* @brief Description    : This function is related to MPU6050 data fusion,
* 							it updates global quaternion variables SEq_x
* @param w_x, w_y, w_z	: gyroscope measurements [rad/s]
* @param a_x, a_y, a_z	: accelerometer measurements
*/
void vIMU_filterUpdate(float gx, float gy, float gz, float ax, float ay, float az);
void vIMU_getAngles(IMU_t* IMU_Struct, float GyroZ_RadPerS);
void vIMU_initStruct(IMU_t* IMU_Struct);

float invSqrt(float x);


#endif /* FUSION_H_ */
