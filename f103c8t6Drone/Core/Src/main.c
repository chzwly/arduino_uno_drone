/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序体
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

/* 包含文件 ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* 私有包含文件 ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pid.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* 私有类型定义 -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* 私有宏定义 ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CHANNELS 6
#define ARM_THRESHOLD 1300
#define YAW_ARM_MAX 1800
#define YAW_DISARM_MIN 1200
#define FAILSAFE_TIMEOUT 1000
#define PPM_SYNC_MIN 3000
#define PPM_PULSE_MIN 900
#define PPM_PULSE_MAX 2100
/* USER CODE END PD */

/* 私有变量 ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t ppm_pulse_widths[MAX_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
uint8_t ppm_channel_index = 0;
uint8_t ppm_frame_complete = 0;
uint32_t last_pulse_width = 0;
uint32_t last_ppm_time = 0;

uint8_t txBuffer[200];
volatile uint8_t txComplete = 1;

uint16_t throttle, roll_in, pitch_in, yaw_in, aux1, aux2;
uint8_t armed = 0;

float filtered_pitch = 0.0f;
float filtered_roll = 0.0f;
float yaw_rate = 0.0f;

PID_t pid_pitch;
PID_t pid_roll;
PID_t pid_yaw;

uint16_t motor[4];
/* USER CODE END PV */

/* 私有函数原型 -----------------------------------------------*/
void SystemClock_Config(void);
void loop_control(float pitch, float roll, float yaw_rate, uint16_t *pwm_values, float *mix_values);
void read_ppm_channels(void);
void failsafe_check(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* 私有用户代码 ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void loop_control(float pitch, float roll, float yaw_rate, uint16_t *pwm_values, float *mix_values)
{
    static uint32_t last_debug = 0;
    if (HAL_GetTick() - last_debug > 500) {
        last_debug = HAL_GetTick();
    }

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        ppm_pulse_widths[i] = fminf(fmaxf(ppm_pulse_widths[i], PPM_PULSE_MIN), PPM_PULSE_MAX);
    }

    float throttle = (ppm_pulse_widths[2] - 1000) / 1000.0f;
    float target_pitch = ((ppm_pulse_widths[1] - 1500) / 500.0f) * 30.0f;
    float target_roll = ((ppm_pulse_widths[0] - 1500) / 500.0f) * 30.0f;
    float target_yaw = ((ppm_pulse_widths[3] - 1500) / 500.0f) * 30.0f;

    throttle = fminf(fmaxf(throttle, 0.0f), 1.0f);
    target_pitch = fminf(fmaxf(target_pitch, -30.0f), 30.0f);
    target_roll = fminf(fmaxf(target_roll, -30.0f), 30.0f);
    target_yaw = fminf(fmaxf(target_yaw, -30.0f), 30.0f);

    float pitch_output = PID_Update(&pid_pitch, target_pitch, pitch, 0.01f);
    float roll_output = PID_Update(&pid_roll, target_roll, roll, 0.01f);
    float yaw_output = PID_Update(&pid_yaw, target_yaw, yaw_rate, 0.01f);

    mix_values[4] = throttle;
    mix_values[5] = target_pitch;
    mix_values[6] = pitch_output;
    mix_values[7] = target_roll;
    mix_values[8] = roll_output;
    mix_values[9] = target_yaw;
    mix_values[10] = yaw_output;

    float m1 = throttle + pitch_output - roll_output + yaw_output;
    float m2 = throttle + pitch_output + roll_output - yaw_output;
    float m3 = throttle - pitch_output + roll_output + yaw_output;
    float m4 = throttle - pitch_output - roll_output - yaw_output;

    mix_values[0] = fminf(fmaxf(m1, 0.0f), 1.0f);
    mix_values[1] = fminf(fmaxf(m2, 0.0f), 1.0f);
    mix_values[2] = fminf(fmaxf(m3, 0.0f), 1.0f);
    mix_values[3] = fminf(fmaxf(m4, 0.0f), 1.0f);

    pwm_values[0] = (uint16_t)(mix_values[0] * 1000 + 1000);
    pwm_values[1] = (uint16_t)(mix_values[1] * 1000 + 1000);
    pwm_values[2] = (uint16_t)(mix_values[2] * 1000 + 1000);
    pwm_values[3] = (uint16_t)(mix_values[3] * 1000 + 1000);

    if (armed) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_values[0]);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_values[1]);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_values[2]);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_values[3]);
    } else {
        for (int i = 0; i < 4; i++) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1 + i, 1000);
            pwm_values[i] = 1000;
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t pulse_width = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (pulse_width >= PPM_PULSE_MIN && pulse_width <= PPM_PULSE_MAX) {
            static uint32_t last_debug = 0;
            if (HAL_GetTick() - last_debug > 100) {
                last_debug = HAL_GetTick();
            }

            if (last_pulse_width > PPM_SYNC_MIN)
            {
                ppm_channel_index = 0;
            }
            if (ppm_channel_index < MAX_CHANNELS)
            {
                ppm_pulse_widths[ppm_channel_index] = pulse_width;
                ppm_channel_index++;
            }
            if (ppm_channel_index >= MAX_CHANNELS)
            {
                ppm_frame_complete = 1;
                last_ppm_time = HAL_GetTick();
                ppm_channel_index = 0;
            }
        }
        last_pulse_width = pulse_width;
    }
}

void read_ppm_channels(void)
{
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        ppm_pulse_widths[i] = (ppm_pulse_widths[i] < PPM_PULSE_MIN || ppm_pulse_widths[i] > PPM_PULSE_MAX) ? 1500 : ppm_pulse_widths[i];
    }
    roll_in = ppm_pulse_widths[0];
    pitch_in = ppm_pulse_widths[1];
    throttle = ppm_pulse_widths[2];
    yaw_in = ppm_pulse_widths[3];
    aux1 = ppm_pulse_widths[4];
    aux2 = ppm_pulse_widths[5];
}

void failsafe_check(void)
{
    if (HAL_GetTick() - last_ppm_time > FAILSAFE_TIMEOUT)
    {
        armed = 0;
        for (int i = 0; i < 4; i++)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1 + i, 1000);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        txComplete = 1;
    }
}
/* USER CODE END 0 */

/**
  * @brief  应用程序入口点。
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    int16_t accel[3], gyro[3];
    float pitch, roll, yaw_rate;
    uint16_t pwm_values[4];
    float mix_values[11];
    uint32_t last_debug = 0;
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */
    MPU6050_Init();
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    PID_Init(&pid_pitch, 0.004f, 0.001f, 0.0005f, 1.0f);
    PID_Init(&pid_roll, 0.004f, 0.001f, 0.0005f, 1.0f);
    PID_Init(&pid_yaw, 0.004f, 0.001f, 0.0005f, 1.0f);
    /* USER CODE END 2 */

    /* 无限循环 */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        failsafe_check();

        if (ppm_frame_complete)
        {
            ppm_frame_complete = 0;
            read_ppm_channels();

            if (throttle < ARM_THRESHOLD && yaw_in > YAW_ARM_MAX)
                armed = 1;
            if (throttle < ARM_THRESHOLD && yaw_in < YAW_DISARM_MIN)
                armed = 0;
        }

        MPU6050_Read_Accel_Gyro(accel, gyro);
        Calculate_Angles(accel, gyro, &pitch, &roll, &yaw_rate);
        Apply_LowPass_Filter(pitch, &filtered_pitch);
        Apply_LowPass_Filter(roll, &filtered_roll);

        loop_control(filtered_pitch, filtered_roll, yaw_rate, pwm_values, mix_values);



        HAL_Delay(2);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief 系统时钟配置
  * @retval 无
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  错误发生时执行的函数。
  * @retval 无
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  报告发生 assert_param 错误的源文件名和行号
  * @param  file: 指向源文件名的指针
  * @param  line: assert_param 错误行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
