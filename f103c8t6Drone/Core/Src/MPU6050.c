/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 传感器驱动实现
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

/* 包含文件 */
#include "mpu6050.h"
#include "usart.h"
#include <string.h>
#include <math.h>

/* 变量定义 */
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
const float alpha = 0.8f;
float fAccelX = 0, fAccelY = 0, fAccelZ = 0;

/* 私有变量 */
static uint8_t txBuffer[200];

/* 函数实现 */
void MPU6050_Init(void)
{
    uint8_t data;
    uint8_t who_am_i;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &who_am_i, 1, HAL_MAX_DELAY);
    if (who_am_i != 0x68) {
        snprintf((char *)txBuffer, sizeof(txBuffer), "MPU6050 未找到: 0x%X\r\n", who_am_i);
        HAL_UART_Transmit(&huart1, txBuffer, strlen((char *)txBuffer), HAL_MAX_DELAY);
        while (1);
    }

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY); // 唤醒 MPU6050
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY); // 加速度计配置
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY); // 陀螺仪配置

    int16_t accel[3];
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (int i = 0; i < 500; i++) {
        MPU6050_Read_Accel_Gyro(accel, NULL);
        ax_sum += accel[0];
        ay_sum += accel[1];
        az_sum += accel[2];
        HAL_Delay(2);
    }
    accel_offset_x = (ax_sum / 500.0f) / 16384.0f;
    accel_offset_y = (ay_sum / 500.0f) / 16384.0f;
    accel_offset_z = (az_sum / 500.0f) / 16384.0f - 1.0f;
    snprintf((char *)txBuffer, sizeof(txBuffer), "Accel offsets: X:%.4f Y:%.4f Z:%.4f\r\n",
             accel_offset_x, accel_offset_y, accel_offset_z);
    HAL_UART_Transmit(&huart1, txBuffer, strlen((char *)txBuffer), HAL_MAX_DELAY);
}

void MPU6050_Read_Accel_Gyro(int16_t *accel, int16_t *gyro)
{
    uint8_t rawData[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, rawData, 14, HAL_MAX_DELAY);

    accel[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
    accel[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    accel[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
    if (gyro != NULL) {
        gyro[0] = (int16_t)((rawData[8] << 8) | rawData[9]);
        gyro[1] = (int16_t)((rawData[10] << 8) | rawData[11]);
        gyro[2] = (int16_t)((rawData[12] << 8) | rawData[13]);
    }
}

void Calculate_Angles(int16_t *accel, int16_t *gyro, float *pitch, float *roll, float *yaw_rate)
{
    static uint32_t last_debug = 0;
    if (HAL_GetTick() - last_debug > 500) {
        last_debug = HAL_GetTick();
    }

    float ax = accel[0] / 16384.0f - accel_offset_x;
    float ay = accel[1] / 16384.0f - accel_offset_y;
    float az = accel[2] / 16384.0f - accel_offset_z;

    fAccelX = alpha * ax + (1.0f - alpha) * fAccelX;
    fAccelY = alpha * ay + (1.0f - alpha) * fAccelY;
    fAccelZ = alpha * az + (1.0f - alpha) * fAccelZ;

    float denom = sqrt(fAccelY * fAccelY + fAccelZ * fAccelZ);
    *pitch = (denom > 0.1) ? atan2(-fAccelX, denom) * 180.0f / M_PI : 0.0f;
    *roll = (fabs(fAccelZ) > 0.1) ? atan2(fAccelY, fAccelZ) * 180.0f / M_PI : 0.0f;
    *yaw_rate = gyro[2] / 131.0f;
}

void Apply_LowPass_Filter(float new_value, float *filtered_value)
{
    *filtered_value = alpha * new_value + (1.0f - alpha) * (*filtered_value);
}