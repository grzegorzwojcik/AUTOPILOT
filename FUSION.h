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
#include "functions.h"
#include "I2C.h"
#include <math.h>

#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/semphr.h"

			/*  System constants. */
/*Sampling period in seconds (shown as 1 ms)*/
#define deltat 0.001f

/*Gyroscope measurement error in rad/s (shown as 5 deg/s)*/
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)

/*Compute beta*/
#define beta sqrt(3.0f / 4.0f) * gyroMeasError


			/*  Global variables. */
/*Estimated orientation quaternion elements with initial conditions*/
volatile float SEq_1, SEq_2, SEq_3, SEq_4;


/*-----------------------------------------------------------
* @brief Function Name  : FUSION_filterUpdate
* @brief Description    : This function is related to MPU6050 data fusion,
* 							it updates global quaternion variables SEq_x
* @param w_x, w_y, w_z	: gyroscope measurements [rad/s]
* @param a_x, a_y, a_z	: accelerometer measurements
*/
void FUSION_filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);


#endif /* FUSION_H_ */
