#include "main.h"
#include "control.h"
#include "config.h"
#include "vofa_debug.h"

static float gimbal_pitch_up_limit_deg = GIMBAL_PITCH_DEFAULT_UP_LIMIT_DEG;
static float gimbal_pitch_down_limit_deg = GIMBAL_PITCH_DEFAULT_DOWN_LIMIT_DEG;
static float gimbal_yaw_error_filter;   
static float gimbal_pitch_error_filter;
static float gimbal_yaw_speed_last;
static float gimbal_pitch_speed_last;
static uint8_t gimbal_error_filter_initialized;

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
 * @brief 视觉误差死区处理，减小靶心附近像素噪声造成的电机抖动
 * @param error 输入误差，单位pixel
 * @param deadband 死区大小，单位pixel
 * @return 死区处理后的误差
 */
static float Gimbal_ApplyPixelDeadband(float error, float deadband) {
    if (Gimbal_Abs(error) <= deadband) {
        return 0.0f;
    }
    return error;
}

/**
 * @brief 视觉误差一阶低通滤波
 * @param raw_error 原始误差，单位pixel
 * @param filter_value 滤波状态
 * @return 滤波后的误差，单位pixel
 */
static float Gimbal_FilterVisionError(float raw_error, float *filter_value) {
    *filter_value += GIMBAL_ERROR_FILTER_ALPHA * (raw_error - *filter_value);
    return *filter_value;
}

/**
 * @brief 限制目标速度每次变化量，让重负载yaw轴动作更平滑
 * @param target_speed PID输出的目标速度，单位rpm
 * @param last_speed 上一次下发的目标速度，单位rpm
 * @param max_step 每次允许变化的最大速度，单位rpm
 * @return 斜坡限幅后的目标速度，单位rpm
 */
static float Gimbal_LimitSpeedSlew(float target_speed, float *last_speed, float max_step) {
    float delta = target_speed - *last_speed;

    delta = Gimbal_Clamp(delta, -max_step, max_step);
    *last_speed += delta;
    return *last_speed;
}

/**
 * @brief 清除视觉滤波和速度斜坡状态
 * @note  目标丢失或重新进入视觉跟踪时调用，避免旧状态影响下一次跟踪
 */
static void Gimbal_ResetVisionState(void) {
    gimbal_yaw_error_filter = 0.0f;
    gimbal_pitch_error_filter = 0.0f;
    gimbal_yaw_speed_last = 0.0f;
    gimbal_pitch_speed_last = 0.0f;
    gimbal_error_filter_initialized = 0;
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
    // yaw轴实测低速模式效果不好，当前统一使用QD4310_SetSpeed
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
    float yaw_error;
    float pitch_error;
    float speed_yaw;
    float speed_pitch;

    // 目标丢失，积分清零，电机停转
    if (!maixcam_data.target_valid) {
        pid_yaw_handle.integral   = 0.0f;
        pid_pitch_handle.integral = 0.0f;
        pid_yaw_handle.last_error = 0.0f;
        pid_pitch_handle.last_error = 0.0f;
        Gimbal_ResetVisionState();
        Set_Motor_Speed(&motor_yaw_handle, 0);
        Set_Motor_Speed(&motor_pitch_handle, 0);
        return;
    }

    // 像素误差先滤波、死区处理，再送入PID输出电机速度
    yaw_error = -maixcam_data.pixel_dx;
    pitch_error = maixcam_data.pixel_dy;
    if (!gimbal_error_filter_initialized) {
        gimbal_yaw_error_filter = yaw_error;
        gimbal_pitch_error_filter = pitch_error;
        gimbal_error_filter_initialized = 1;
    }

    // 视觉误差先滤波再做死区，减小靶心附近抖动
    yaw_error = Gimbal_FilterVisionError(yaw_error, &gimbal_yaw_error_filter);
    pitch_error = Gimbal_FilterVisionError(pitch_error, &gimbal_pitch_error_filter);
    yaw_error = Gimbal_ApplyPixelDeadband(yaw_error, GIMBAL_YAW_PIXEL_DEADBAND);
    pitch_error = Gimbal_ApplyPixelDeadband(pitch_error, GIMBAL_PITCH_PIXEL_DEADBAND);

    // 误差进入死区后清除PID历史状态，防止积分让电机在靶心附近慢慢爬
    if (yaw_error == 0.0f) {
        pid_yaw_handle.integral = 0.0f;
        pid_yaw_handle.last_error = 0.0f;
        gimbal_yaw_speed_last = 0.0f;
    }
    if (pitch_error == 0.0f) {
        pid_pitch_handle.integral = 0.0f;
        pid_pitch_handle.last_error = 0.0f;
        gimbal_pitch_speed_last = 0.0f;
    }

    speed_yaw = PID_Update(&pid_yaw_handle, yaw_error);
    speed_pitch = PID_Update(&pid_pitch_handle, pitch_error);

    // 目标速度做斜坡限幅，避免重负载yaw轴低速一快一慢
    speed_yaw = Gimbal_LimitSpeedSlew(speed_yaw, &gimbal_yaw_speed_last, GIMBAL_YAW_SPEED_SLEW_RPM);
    speed_pitch = Gimbal_LimitSpeedSlew(speed_pitch, &gimbal_pitch_speed_last, GIMBAL_PITCH_SPEED_SLEW_RPM);

    Set_Motor_Speed(&motor_yaw_handle, speed_yaw);
    Set_Motor_Speed(&motor_pitch_handle, speed_pitch);
}
