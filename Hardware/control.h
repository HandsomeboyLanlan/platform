#ifndef __CONTROL_H
#define __CONTROL_H

#include "QD4310.h"

#define GIMBAL_RAD_TO_DEG 57.2957795f   // 弧度转角度系数，180/pi

// 云台回零角度，单位rad；pitch软限位以GIMBAL_HOME_PITCH_ANGLE为0点
#define GIMBAL_HOME_YAW_ANGLE                   0.0f
#define GIMBAL_HOME_PITCH_ANGLE                 0.0f

// pitch轴默认上/下限位角度，单位deg；可通过Gimbal_SetPitchLimitsDeg动态修改
#define GIMBAL_PITCH_DEFAULT_UP_LIMIT_DEG       30.0f
#define GIMBAL_PITCH_DEFAULT_DOWN_LIMIT_DEG     30.0f
#define GIMBAL_PITCH_MAX_LIMIT_DEG              170.0f  // pitch轴上下限位限幅

#define GIMBAL_PITCH_POSITIVE_SPEED_IS_UP       1.0f    // 如果pitch正速度会让镜头向下运动，则将该宏改为-1.0f
#define GIMBAL_PITCH_LIMIT_STOP_MARGIN_DEG      1.0f    // 距离软限位小于该角度时停止继续向外转动，单位deg
#define GIMBAL_PITCH_LIMIT_SPEED_GAIN           3.0f    // 靠近软限位时的降速系数，单位rpm/deg
#define GIMBAL_PITCH_MAX_SPEED_RPM              180.0f  // pitch轴软件限速，防止视觉PID输出过大速度，单位rpm

// 视觉误差死区，单位pixel；靶心附近的小抖动直接当作0处理
#define GIMBAL_YAW_PIXEL_DEADBAND               3.0f
#define GIMBAL_PITCH_PIXEL_DEADBAND             2.0f

#define GIMBAL_ERROR_FILTER_ALPHA               0.40f   // 视觉误差一阶低通滤波系数，越小越稳，越大响应越快

// 视觉PID输出速度斜坡限幅，单位rpm/次；yaw轴负载重，变化率要小一点
#define GIMBAL_YAW_SPEED_SLEW_RPM               2.0f
#define GIMBAL_PITCH_SPEED_SLEW_RPM             1.5f

/* MaixCam数据结构体 */
typedef struct {
    float pixel_dx;         // pixel_dx MaixCam目标X坐标 - 屏幕中心X坐标
    float pixel_dy;         // pixel_dy MaixCam目标Y坐标 - 屏幕中心Y坐标
    uint8_t target_valid;   // target_valid 是否识别到目标
} Maixcam_Data_t;

void Control_Init(void);
void Gimbal_GotoZero(void);
void Set_Motor_Speed(QD4310_t *motor, float speed);
void Gimbal_Track(Maixcam_Data_t maixcam_data);
// 修改pitch轴相对0点的上/下限位角度，单位deg
uint8_t Gimbal_SetPitchLimitsDeg(float up_limit_deg, float down_limit_deg);
// 获取pitch轴相对0点的角度，向上为正，单位deg
float Gimbal_GetPitchRelativeAngleDeg(void);

#endif
