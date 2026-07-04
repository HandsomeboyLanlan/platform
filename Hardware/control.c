#include "main.h"
#include "control.h"
#include "config.h"
#include "QD4310.h"

/**
 * @brief 云台控制初始化函数
 */
void Control_Init(void) {
    QD4310_Enable(&motor_yaw_handle);         // 使能yaw轴电机
    QD4310_Enable(&motor_pitch_handle);       // 使能pitch轴电机
    QD4310_SetSpeed(&motor_yaw_handle, 0.0f); // 设置yaw轴电机转速为0rpm
    QD4310_SetSpeed(&motor_pitch_handle, 0.0f); // 设置pitch轴电机转速为0rpm
}

/**
 * @brief 云台视觉跟踪，每个控制周期调用一次
 * @param maixcam_data MaixCam数据
 * @note  MaixCam端计算差值后传入，正负号决定电机转向
 */
void Gimbal_Track(Maixcam_Data_t maixcam_data) {
    // 目标丢失，积分清零，电机停转
    if (!maixcam_data.target_valid) {
        pid_yaw_handle.integral   = 0.0f;
        pid_pitch_handle.integral = 0.0f;
        QD4310_SetSpeed(&motor_yaw_handle, 0);
        QD4310_SetSpeed(&motor_pitch_handle, 0);
        return;
    }

    // 像素误差直接送入PID → 输出电机速度
    float speed_yaw = PID_Update(&pid_yaw_handle, maixcam_data.pixel_dx);
    float speed_pitch = PID_Update(&pid_pitch_handle, maixcam_data.pixel_dy);

    QD4310_SetSpeed(&motor_yaw_handle, speed_yaw);
    QD4310_SetSpeed(&motor_pitch_handle, speed_pitch);
}

