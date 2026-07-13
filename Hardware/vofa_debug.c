#include "vofa_debug.h"
#include "config.h"
#include <stdio.h>

#define VOFA_TX_TIMEOUT_MS 10U      // 串口阻塞发送超时时间，VOFA调试不应长时间占用主循环 timeout
#define VOFA_TX_BUFFER_SIZE 192U    // CSV单帧发送缓冲区长度 buffer
#define VOFA_PID_PARAM_MAX 10.0f    // 在线调参PID参数最大值，防止串口误输入过大的参数

static UART_HandleTypeDef *vofa_huart;  // VOFA使用的串口句柄，由VOFA_DebugInit传入 uart
static uint32_t vofa_last_send_tick;    // 上一次发送VOFA数据的系统时间 tick
static float vofa_yaw_cmd_speed;        // yaw轴最近一次下发给电机的目标速度，单位rpm
static float vofa_pitch_cmd_speed;      // pitch轴最近一次下发给电机的目标速度，单位rpm

/* VOFA串口在线调参接收缓存 */
static uint8_t vofa_rx_byte;
static char vofa_pid_cmd_buffer[VOFA_PID_CMD_BUFFER_SIZE];
static char vofa_pid_pending_buffer[VOFA_PID_CMD_BUFFER_SIZE];
static uint8_t vofa_pid_cmd_len;
static volatile uint8_t vofa_pid_cmd_ready;

/**
 * @brief 判断字符是否为数字
 * @param ch 输入字符
 * @return 1: 是数字 0: 不是数字
 */
static uint8_t VOFA_IsDigit(char ch) {
    return (ch >= '0' && ch <= '9');
}

/**
 * @brief 跳过字符串前面的空格
 * @param text 字符串指针
 * @return 跳过空格后的字符串指针
 */
static const char *VOFA_SkipSpace(const char *text) {
    while (*text == ' ') {
        text++;
    }
    return text;
}

/**
 * @brief 比较字符，忽略大小写
 * @param a 字符a
 * @param b 字符b
 * @return 1: 相同 0: 不同
 */
static uint8_t VOFA_CharEqualIgnoreCase(char a, char b) {
    if (a >= 'a' && a <= 'z') {
        a = (char)(a - 'a' + 'A');
    }
    if (b >= 'a' && b <= 'z') {
        b = (char)(b - 'a' + 'A');
    }
    return (a == b);
}

/**
 * @brief 匹配字符串命令关键字，忽略大小写
 * @param text 字符串指针地址
 * @param token 需要匹配的关键字
 * @return 1: 匹配成功 0: 匹配失败
 */
static uint8_t VOFA_MatchToken(const char **text, const char *token) {
    const char *p = VOFA_SkipSpace(*text);

    while (*token != '\0') {
        if (!VOFA_CharEqualIgnoreCase(*p, *token)) {
            return 0;
        }
        p++;
        token++;
    }

    *text = p;
    return 1;
}

/**
 * @brief 从字符串中解析一个浮点数
 * @param text 字符串指针地址
 * @param value 解析得到的浮点数
 * @return 1: 解析成功 0: 解析失败
 */
static uint8_t VOFA_ParseFloat(const char **text, float *value) {
    const char *p = VOFA_SkipSpace(*text);
    float sign = 1.0f;
    float result = 0.0f;
    float base = 0.1f;
    uint8_t has_digit = 0;

    if (*p == '-') {
        sign = -1.0f;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (VOFA_IsDigit(*p)) {
        result = result * 10.0f + (float)(*p - '0');
        p++;
        has_digit = 1;
    }

    if (*p == '.') {
        p++;
        while (VOFA_IsDigit(*p)) {
            result += (float)(*p - '0') * base;
            base *= 0.1f;
            p++;
            has_digit = 1;
        }
    }

    if (!has_digit) {
        return 0;
    }

    *value = result * sign;
    *text = p;
    return 1;
}

/**
 * @brief 解析逗号后面的浮点数
 * @param text 字符串指针地址
 * @param value 解析得到的浮点数
 * @return 1: 解析成功 0: 解析失败
 */
static uint8_t VOFA_ParseCommaFloat(const char **text, float *value) {
    const char *p = VOFA_SkipSpace(*text);

    if (*p != ',') {
        return 0;
    }
    p++;
    *text = p;
    return VOFA_ParseFloat(text, value);
}

/**
 * @brief 清除PID历史状态，避免换参数后旧积分继续影响输出
 * @param pid PID句柄
 */
static void VOFA_ResetPidState(PID_t *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief 应用新的PID参数
 * @param pid PID句柄
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
static void VOFA_SetPid(PID_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    VOFA_ResetPidState(pid);
}

/**
 * @brief 解析VOFA在线调参命令
 * @param cmd 命令字符串
 * @note  支持命令: PID,YAW,kp,ki,kd 或 PID,PITCH,kp,ki,kd
 */
static void VOFA_ParsePidCommand(char *cmd) {
    const char *p = cmd;
    PID_t *pid = NULL;
    float kp, ki, kd;

    if (!VOFA_MatchToken(&p, "PID")) {
        return;
    }
    p = VOFA_SkipSpace(p);
    if (*p != ',') {
        return;
    }
    p++;

    if (VOFA_MatchToken(&p, "YAW")) {
        pid = &pid_yaw_handle;
    } else if (VOFA_MatchToken(&p, "PITCH")) {
        pid = &pid_pitch_handle;
    } else {
        return;
    }

    if (!VOFA_ParseCommaFloat(&p, &kp)) return;
    if (!VOFA_ParseCommaFloat(&p, &ki)) return;
    if (!VOFA_ParseCommaFloat(&p, &kd)) return;

    if (kp < 0.0f || ki < 0.0f || kd < 0.0f) return;
    if (kp > VOFA_PID_PARAM_MAX || ki > VOFA_PID_PARAM_MAX || kd > VOFA_PID_PARAM_MAX) return;

    VOFA_SetPid(pid, kp, ki, kd);
}

/**
 * @brief 在主循环中处理已经接收完整的在线调参命令
 * @note  串口中断只负责缓存命令，真正修改PID参数在主循环中完成
 */
static void VOFA_ProcessPidCommand(void) {
    char cmd[VOFA_PID_CMD_BUFFER_SIZE];
    uint8_t i;

    if (!vofa_pid_cmd_ready) {
        return;
    }

    __disable_irq();
    for (i = 0; i < VOFA_PID_CMD_BUFFER_SIZE; i++) {
        cmd[i] = vofa_pid_pending_buffer[i];
        if (cmd[i] == '\0') {
            break;
        }
    }
    vofa_pid_cmd_ready = 0;
    __enable_irq();

    cmd[VOFA_PID_CMD_BUFFER_SIZE - 1U] = '\0';
    VOFA_ParsePidCommand(cmd);
}

/**
 * @brief 初始化VOFA调试模块 init
 * @param huart VOFA使用的串口句柄，例如&huart3
 */
void VOFA_DebugInit(UART_HandleTypeDef *huart) {
    vofa_huart = huart;
    vofa_last_send_tick = 0;
    vofa_yaw_cmd_speed = 0.0f;
    vofa_pitch_cmd_speed = 0.0f;
    vofa_pid_cmd_len = 0;
    vofa_pid_cmd_ready = 0;
    HAL_UART_Receive_IT(vofa_huart, &vofa_rx_byte, 1);
}

/**
 * @brief 记录云台控制输出，用于对比电机反馈速度 cmd
 * @param motor 电机句柄
 * @param speed 当前下发给电机的目标速度，单位rpm
 */
void VOFA_DebugUpdateCommand(QD4310_t *motor, float speed) {
    if (motor == &motor_yaw_handle) {
        vofa_yaw_cmd_speed = speed;
    } else if (motor == &motor_pitch_handle) {
        vofa_pitch_cmd_speed = speed;
    }
}

/**
 * @brief VOFA串口接收完成回调，用于在线修改PID参数
 * @param huart 触发接收中断的串口句柄
 */
void VOFA_DebugRxCpltCallback(UART_HandleTypeDef *huart) {
    char ch;
    uint8_t i;

    if (vofa_huart == NULL || huart != vofa_huart) {
        return;
    }

    ch = (char)vofa_rx_byte;
    if (ch == '\r' || ch == '\n') {
        if (vofa_pid_cmd_len > 0U) {
            vofa_pid_cmd_buffer[vofa_pid_cmd_len] = '\0';
            for (i = 0; i <= vofa_pid_cmd_len; i++) {
                vofa_pid_pending_buffer[i] = vofa_pid_cmd_buffer[i];
            }
            vofa_pid_cmd_ready = 1;
            vofa_pid_cmd_len = 0;
        }
    } else if (vofa_pid_cmd_len < (VOFA_PID_CMD_BUFFER_SIZE - 1U)) {
        vofa_pid_cmd_buffer[vofa_pid_cmd_len++] = ch;
    } else {
        vofa_pid_cmd_len = 0;
    }

    HAL_UART_Receive_IT(vofa_huart, &vofa_rx_byte, 1);
}

/**
 * @brief 立即发送一帧VOFA CSV数据 send
 * @note  格式:
 *        0 time_ms,
 *        1 target_valid,
 *        2 pixel_dx,
 *        3 pixel_dy,
 *        4 yaw_cmd_speed,
 *        5 pitch_cmd_speed,
 *        6 yaw_motor_speed,
 *        7 pitch_motor_speed,
 *        8 yaw_angle,
 *        9 pitch_angle,
 *        10 yaw_current,
 *        11 pitch_current
 */
void VOFA_DebugSendNow(void) {
    char tx_buf[VOFA_TX_BUFFER_SIZE];
    int len;

    if (vofa_huart == NULL) {
        return;
    }

    len = snprintf(tx_buf, sizeof(tx_buf),
                   "%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.3f,%.3f\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned int)maixcam_data_handle.target_valid,
                   maixcam_data_handle.pixel_dx,
                   maixcam_data_handle.pixel_dy,
                   vofa_yaw_cmd_speed,
                   vofa_pitch_cmd_speed,
                   motor_yaw_handle.speed,
                   motor_pitch_handle.speed,
                   motor_yaw_handle.angle,
                   motor_pitch_handle.angle,
                   motor_yaw_handle.current,
                   motor_pitch_handle.current);

    /* snprintf失败或数据过长时直接丢弃/截断，避免调试输出影响控制 safe */
    if (len <= 0) {
        return;
    }
    if ((uint32_t)len >= sizeof(tx_buf)) {
        len = sizeof(tx_buf) - 1;
    }

    HAL_UART_Transmit(vofa_huart, (uint8_t *)tx_buf, (uint16_t)len, VOFA_TX_TIMEOUT_MS);
}

/**
 * @brief 按周期发送VOFA调试数据，建议在while(1)中调用 send
 * @param period_ms 发送周期，单位ms；传0时使用默认周期 default
 */
void VOFA_DebugSendPeriod(uint32_t period_ms) {
    uint32_t now = HAL_GetTick();

    VOFA_ProcessPidCommand();

    if (period_ms == 0U) {
        period_ms = VOFA_DEBUG_DEFAULT_PERIOD_MS;
    }

    if (now - vofa_last_send_tick >= period_ms) {
        vofa_last_send_tick = now;
        VOFA_DebugSendNow();
    }
}
