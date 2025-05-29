/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 传感器驱动头文件
  ******************************************************************************
  * @attention
  *
  * 版权所有 (c) 2025 STMicroelectronics。
  * 保留所有权利。
  *
  * 本软件根据根目录中的 LICENSE 文件中的条款进行许可。
  * 如果没有提供 LICENSE 文件，则按原样提供。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MPU6050_H
#define __MPU6050_H

/* 包含文件 */
#include "main.h"
#include "i2c.h"
#include <stdio.h>

/* 宏定义 */
#define MPU6050_ADDR 0xD0 // MPU6050 I2C 地址 (AD0 引脚低电平: 0x68 << 1)

/* 外部变量声明 */
extern float accel_offset_x, accel_offset_y, accel_offset_z;
extern const float alpha;
extern float fAccelX, fAccelY, fAccelZ;

/* 函数原型 */
void MPU6050_Init(void);
void MPU6050_Read_Accel_Gyro(int16_t *accel, int16_t *gyro);
void Calculate_Angles(int16_t *accel, int16_t *gyro, float *pitch, float *roll, float *yaw_rate);
void Apply_LowPass_Filter(float new_value, float *filtered_value);

#endif /* __MPU6050_H */