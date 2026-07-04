#ifndef __PID_H
#define __PID_H

// 位置PID结构体
typedef struct {
    float kp, ki, kd;
    float last_error;
    float integral;
    float integral_limit;
    float output_limit;
} PID_t;

float PID_Update(PID_t *pid, float error);

#endif
