#include "main.h"
#include "control.h"
#include "config.h"
#include "vofa_debug.h"

// 弧度转角度系数，180/pi
#define GIMBAL_RAD_TO_DEG 57.2957795f

static float gimbal_pitch_up_limit_deg = GIMBAL_PITCH_DEFAULT_UP_LIMIT_DEG;
static float gimbal_pitch_down_limit_deg = GIMBAL_PITCH_DEFAULT_DOWN_LIMIT_DEG;

/**
 * @brief 求浮点数绝对值
 * @param value 输入值
 * @return 绝对值
 */
static float Gimbal_Abs(float value) {
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 限制浮点数范围
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static float Gimbal_Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 将角度限制到[-pi, pi)范围
 * @param angle 输入角度，单位rad
 * @return 限制后的角度，单位rad
 */
static float Gimbal_WrapRad(float angle) {
    while (angle >= QD4310_PI) {
        angle -= QD4310_TWO_PI;
    }
    while (angle < -QD4310_PI) {
        angle += QD4310_TWO_PI;
    }
    return angle;
}

/**
 * @brief 获取pitch轴相对0点的角度
 * @return pitch轴相对GIMBAL_HOME_PITCH_ANGLE的角度，向上为正，单位deg
 */
float Gimbal_GetPitchRelativeAngleDeg(void) {
    float angle = Gimbal_WrapRad(motor_pitch_handle.angle - GIMBAL_HOME_PITCH_ANGLE);
    return angle * GIMBAL_RAD_TO_DEG * GIMBAL_PITCH_POSITIVE_SPEED_IS_UP;   // 把弧度转换成角度
}

/**
 * @brief 设置pitch轴软限位角度
 * @param up_limit_deg 0点上方允许转动角度，单位deg
 * @param down_limit_deg 0点下方允许转动角度，单位deg
 * @return 1: 设置成功 0: 参数错误
 */
uint8_t Gimbal_SetPitchLimitsDeg(float up_limit_deg, float down_limit_deg) {
    if (up_limit_deg < 0.0f || down_limit_deg < 0.0f) {
        return 0;
    }

    gimbal_pitch_up_limit_deg = Gimbal_Clamp(up_limit_deg, 0.0f, GIMBAL_PITCH_MAX_LIMIT_DEG);
    gimbal_pitch_down_limit_deg = Gimbal_Clamp(down_limit_deg, 0.0f, GIMBAL_PITCH_MAX_LIMIT_DEG);

    return 1;
}

/**
 * @brief 根据pitch轴当前位置限制目标转速，防止超过软限位
 * @param speed 视觉PID输出的pitch目标转速，单位rpm
 * @return 限位后的pitch目标转速，单位rpm
 */
static float Gimbal_LimitPitchSpeed(float speed) {
    float angle_deg = Gimbal_GetPitchRelativeAngleDeg();
    float up_speed = speed * GIMBAL_PITCH_POSITIVE_SPEED_IS_UP;
    float speed_abs = Gimbal_Abs(up_speed);
    float remaining_deg;  // 当前方向距离软限位还剩余的角度，单位deg
    float allowed_speed;  // 根据剩余角度计算出的允许最大速度，单位rpm

    speed_abs = Gimbal_Clamp(speed_abs, 0.0f, GIMBAL_PITCH_MAX_SPEED_RPM);

    if (up_speed > 0.0f) {
        remaining_deg = gimbal_pitch_up_limit_deg - angle_deg;
        if (remaining_deg <= GIMBAL_PITCH_LIMIT_STOP_MARGIN_DEG) {
            return 0.0f;
        }
        allowed_speed = (remaining_deg - GIMBAL_PITCH_LIMIT_STOP_MARGIN_DEG) *
                        GIMBAL_PITCH_LIMIT_SPEED_GAIN;
        speed_abs = Gimbal_Clamp(speed_abs, 0.0f, allowed_speed);
        return speed_abs * GIMBAL_PITCH_POSITIVE_SPEED_IS_UP;
    }

    if (up_speed < 0.0f) {
        remaining_deg = gimbal_pitch_down_limit_deg + angle_deg;
        if (remaining_deg <= GIMBAL_PITCH_LIMIT_STOP_MARGIN_DEG) {
            return 0.0f;
        }
        allowed_speed = (remaining_deg - GIMBAL_PITCH_LIMIT_STOP_MARGIN_DEG) *
                        GIMBAL_PITCH_LIMIT_SPEED_GAIN;
        speed_abs = Gimbal_Clamp(speed_abs, 0.0f, allowed_speed);
        return -speed_abs * GIMBAL_PITCH_POSITIVE_SPEED_IS_UP;
    }

    return 0.0f;
}

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
 * @brief 设置电机转速
 * @param motor 电机句柄
 * @param speed 转速，单位rpm
 */
void Set_Motor_Speed(QD4310_t *motor, float speed) {
    if (motor == &motor_pitch_handle) {
        // pitch轴不允许720度连续旋转，先经过软限位再下发速度
        speed = Gimbal_LimitPitchSpeed(speed);
    }
    // if (speed <= 5 && speed >= -5) {
    //     QD4310_SetLowSpeed(motor, speed); // 低速模式
    // } else {
    //     QD4310_SetSpeed(motor, speed); // 高速模式
    // }
    VOFA_DebugUpdateCommand(motor, speed); // 记录VOFA调试用的目标速度
    QD4310_SetSpeed(motor, speed);
}

/**
 * @brief 云台回零函数
 * @note  将yaw轴和pitch轴电机转到零点位置
 */
void Gimbal_GotoZero(void) {
    QD4310_SetAngle(&motor_yaw_handle, GIMBAL_HOME_YAW_ANGLE);
    QD4310_SetAngle(&motor_pitch_handle, GIMBAL_HOME_PITCH_ANGLE);
    HAL_Delay(200); // 等待电机转到零点位置
    Set_Motor_Speed(&motor_yaw_handle, 0.0f);
    //Set_Motor_Speed(&motor_pitch_handle, 0.0f);
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
        Set_Motor_Speed(&motor_yaw_handle, 0);
        Set_Motor_Speed(&motor_pitch_handle, 0);
        return;
    }

    // 像素误差直接送入PID → 输出电机速度
    float speed_yaw = PID_Update(&pid_yaw_handle, -maixcam_data.pixel_dx);
    float speed_pitch = PID_Update(&pid_pitch_handle, maixcam_data.pixel_dy);

    Set_Motor_Speed(&motor_yaw_handle, speed_yaw);
    Set_Motor_Speed(&motor_pitch_handle, speed_pitch);
}
