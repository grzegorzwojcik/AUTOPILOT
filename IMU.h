/*
 * FUSION.h
 *
 *  Created on: Jul 10, 2015
 *      Author: Grzegorz WÓJCIK
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

/*Compute beta*/
#define beta				0.5f					// 2 * proportional gain

			/*  Global variables. */

/*Estimated orientation quaternion elements with initial conditions*/
volatile float q0, q1, q2, q3;


/*-----------------------------------------------------------
* @brief Function Name  : FUSION_filterUpdate
* @brief Description    : This function is related to MPU6050 data fusion,
* 							it updates global quaternion variables SEq_x
* @param w_x, w_y, w_z	: gyroscope measurements [rad/s]
* @param a_x, a_y, a_z	: accelerometer measurements
*/
void vFUSION_filterUpdate(float gx, float gy, float gz, float ax, float ay, float az);

float invSqrt(float x);

#endif /* FUSION_H_ */
