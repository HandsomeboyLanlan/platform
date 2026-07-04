#include "main.h"
#include "pid.h"
#include "config.h"

/**
 * @brief 位置式PID更新
 * @param pid   PID参数结构体指针
 * @param error 当前误差（像素差值）
 * @return 电机速度值，范围 [-output_limit, output_limit]
 */
float PID_Update(PID_t *pid, float error) {
    // 积分累加
    pid->integral += error;

    // 积分限幅
    pid->integral = (pid->integral > pid->integral_limit) ? pid->integral_limit : pid->integral;
    pid->integral = (pid->integral < -pid->integral_limit) ? -pid->integral_limit : pid->integral;

    // 微分
    float derivative = error - pid->last_error;
    pid->last_error = error;

    // PID输出计算
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅，匹配 QD4310_SetSpeed 范围 [-1000, 1000]
    output = (output > pid->output_limit) ? pid->output_limit : output;
    output = (output < -pid->output_limit) ? -pid->output_limit : output;

    return output;
}
