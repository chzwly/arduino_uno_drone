#ifndef PID_H
#define PID_H
typedef struct {
    float Kp, Ki, Kd, max_output;
    float integral, last_error;
} PID_t;
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output);
float PID_Update(PID_t *pid, float setpoint, float measured, float dt);
#endif
