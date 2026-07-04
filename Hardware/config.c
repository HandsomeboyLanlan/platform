#include "config.h"

// 初始化yaw轴电机结构体，使用CAN外设，ID为0
QD4310_t motor_yaw = {
    .id = 0,
    .hcan1 = &hcan
};

// 初始化pitch轴电机结构体，使用CAN外设，ID为1
QD4310_t motor_pitch = {
    .id = 1,
    .hcan1 = &hcan
};

/* pid_yaw初始化句柄 */
PID_t pid_yaw = {
    .kp = 0.5f,
    .ki = 0.1f,
    .kd = 0.05f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 1000.0f,
};

/* pid_pitch初始化句柄 */
PID_t pid_pitch = {
    .kp = 0.6f,
    .ki = 0.12f,
    .kd = 0.06f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 1000.0f,
};
