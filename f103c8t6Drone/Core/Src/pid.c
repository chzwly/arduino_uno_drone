#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_output = max_output;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    if (error > 30.0f) error = 30.0f;
    if (error < -30.0f) error = -30.0f;
    pid->integral += error * dt;
    if (pid->integral > pid->max_output) pid->integral = pid->max_output;
    if (pid->integral < -pid->max_output) pid->integral = -pid->max_output;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    if (output > pid->max_output) output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;
    return output;
}
