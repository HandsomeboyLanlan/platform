#include "main.h"
#include "control.h"
#include "config.h"
#include "QD4310.h"

/**
 * @brief 云台视觉跟踪，每个控制周期调用一次
 * @param pixel_dx MaixCam目标X坐标 - 屏幕中心X坐标
 * @param pixel_dy MaixCam目标Y坐标 - 屏幕中心Y坐标
 * @note  MaixCam端计算差值后传入，正负号决定电机转向
 */
void Gimbal_Track(float pixel_dx, float pixel_dy) {
    // 目标丢失，积分清零，电机停转
    // if (!target_valid) {
    //     pid_yaw.integral   = 0.0f;
    //     pid_pitch.integral = 0.0f;
    //     QD4310_SetSpeed(&motor_yaw, 0);
    //     QD4310_SetSpeed(&motor_pitch, 0);
    //     return;
    // }

    // 像素误差直接送入PID → 输出电机速度
    float speed_yaw = PID_Update(&pid_yaw, pixel_dx);
    float speed_pitch = PID_Update(&pid_pitch, pixel_dy);

    QD4310_SetSpeed(&motor_yaw, speed_yaw);
    QD4310_SetSpeed(&motor_pitch, speed_pitch);
}

