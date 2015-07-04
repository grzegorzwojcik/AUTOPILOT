/*
 * I2C.h
 *
 *  Created on: Jul 4, 2015
 *      Author: Grzegorz W�JCIK
 */

#ifndef I2C_H_
#define I2C_H_

#include <stddef.h>
#include "stm32f4xx.h"
#include "functions.h"

//#include "FreeRTOS_Source/include/FreeRTOS.h"
//#include "FreeRTOS_Source/include/task.h"
//#include "FreeRTOS_Source/include/semphr.h"

			/* Hardware port definitions. */
#define portGPIO_I2C			GPIOB

			/* Hardware pin definitions. */
#define pinI2C1_SCL				GPIO_Pin_8
#define pinSourceI2C1_SCL		GPIO_PinSource8
#define pinI2C1_SDA				GPIO_Pin_9
#define pinSourceI2C1_SDA		GPIO_PinSource9

			/* I2C1 setup definitions. */
#define I2C1_ownAddress			0x00
#define I2C1_clockSpeed			100000	/* Fast mode */
#define TM_I2C_TIMEOUT			20000	/* Ticks */

/* Hardware related functions. */
void vhI2C_initRCC(void);
void vhI2C_initGPIO(void);
void vhI2C_initI2C1(void);


/**
 * MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 * This feature allows you to use 2 different sensors with this library at the same time
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
} TM_MPU6050_Device_t;

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
} TM_MPU6050_Result_t;

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
} TM_MPU6050_Accelerometer_t;

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
} TM_MPU6050_Gyroscope_t;

typedef struct {
    /* Private */
    uint8_t Address;
    float Gyro_Mult;
    float Acce_Mult;
    /* Public */
    int16_t Accelerometer_X;
    int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
    float Temperature;
} MPU6050_t;
			/* Hardware related functions. */
//uint8_t uc_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);


/**
 * @brief  Reads single byte from slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to read from
 * @retval Data from slave
 */
uint8_t TM_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);

/**
 * @brief  Reads multi bytes from slave
 * @param  *I2Cx: I2C used
 * @param  uint8_t address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  uint8_t reg: register to read from
 * @param  uint8_t *data: pointer to data array to store data from slave
 * @param  uint8_t count: how many bytes will be read
 * @retval None
 */
void TM_I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

/**
 * @brief  Reads byte from slave without specify register address
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @retval Data from slave
 */
uint8_t TM_I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t address);

/**
 * @brief  Reads multi bytes from slave without setting register from where to start read
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  *data: pointer to data array to store data from slave
 * @param  count: how many bytes will be read
 * @retval None
 */
void TM_I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count);

/**
 * @brief  Writes single byte to slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  data: data to be written
 * @retval None
 */
void TM_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data);

/**
 * @brief  Writes multi bytes to slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  *data: pointer to data array to write it to slave
 * @param  count: how many bytes will be written
 * @retval None
 */
void TM_I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

/**
 * @brief  Writes byte to slave without specify register address
 *
 *         Useful if you have I2C device to read like that:
 *            - I2C START
 *            - SEND DEVICE ADDRESS
 *            - SEND DATA BYTE
 *            - I2C STOP
 *
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  data: data byte which will be send to device
 * @retval None
 */
void TM_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t data);

/**
 * @brief  Writes multi bytes to slave without setting register from where to start write
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  *data: pointer to data array to write data to slave
 * @param  count: how many bytes you want to write
 * @retval None
 */
void TM_I2C_WriteMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count);

/**
 * @brief  Checks if device is connected to I2C bus
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @retval Device status:
 *            - 0: Device is not connected
 *            - > 0: Device is connected
 */
uint8_t TM_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address);

/**
 * @brief  I2C Start condition
 * @param  *I2Cx: I2C used
 * @param  address: slave address
 * @param  direction: master to slave or slave to master
 * @param  ack: ack enabled or disabled
 * @retval Start condition status
 * @note   For private use
 */
int16_t TM_I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t ack);

/**
 * @brief  Stop condition on I2C
 * @param  *I2Cx: I2C used
 * @retval Stop condition status
 * @note   For private use
 */
uint8_t TM_I2C_Stop(I2C_TypeDef* I2Cx);

/**
 * @brief  Reads byte without ack
 * @param  *I2Cx: I2C used
 * @retval Byte from slave
 * @note   For private use
 */
uint8_t TM_I2C_ReadNack(I2C_TypeDef* I2Cx);

/**
 * @brief  Reads byte with ack
 * @param  *I2Cx: I2C used
 * @retval Byte from slave
 * @note   For private use
 */
uint8_t TM_I2C_ReadAck(I2C_TypeDef* I2Cx);

/**
 * @brief  Writes to slave
 * @param  *I2Cx: I2C used
 * @param  data: data to be sent
 * @retval None
 * @note   For private use
 */
void TM_I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data);


#endif /* I2C_H_ */
