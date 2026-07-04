#include "config.h"

// 初始化yaw轴电机结构体，使用CAN外设，ID为0
QD4310_t motor_yaw_handle = {
    .id = 0,
    .hcan1 = &hcan,
};

// 初始化pitch轴电机结构体，使用CAN外设，ID为1
QD4310_t motor_pitch_handle = {
    .id = 1,
    .hcan1 = &hcan,
};

/* pid_yaw初始化句柄 */
PID_t pid_yaw_handle = {
    .kp = 0.5f,
    .ki = 0.1f,
    .kd = 0.05f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 1000.0f,
};

/* pid_pitch初始化句柄 */
PID_t pid_pitch_handle = {
    .kp = 0.6f,
    .ki = 0.12f,
    .kd = 0.06f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 1000.0f,
};

/* maixcam_data初始化句柄 */
Maixcam_Data_t maixcam_data_handle = {
    .pixel_dx = 0.0f,
    .pixel_dy = 0.0f,
    .target_valid = 0,
};

/* uart3_rx_data初始化句柄 */
UART_Rx_Data_t uart3_rx_data_handle = {
    .huart = &huart3,
    .uart3_rx_buffer = {0},
};
