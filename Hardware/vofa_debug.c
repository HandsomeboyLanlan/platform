#include "vofa_debug.h"
#include "config.h"
#include <stdio.h>

/* 串口阻塞发送超时时间，VOFA调试不应长时间占用主循环 timeout */
#define VOFA_TX_TIMEOUT_MS 10U

/* CSV单帧发送缓冲区长度 buffer */
#define VOFA_TX_BUFFER_SIZE 192U

/* VOFA使用的串口句柄，由VOFA_DebugInit传入 uart */
static UART_HandleTypeDef *vofa_huart;

/* 上一次发送VOFA数据的系统时间 tick */
static uint32_t vofa_last_send_tick;

/* yaw/pitch轴最近一次下发给电机的目标速度，单位rpm */
static float vofa_yaw_cmd_speed;
static float vofa_pitch_cmd_speed;

/**
 * @brief 初始化VOFA调试模块 init
 * @param huart VOFA使用的串口句柄，例如&huart3
 */
void VOFA_DebugInit(UART_HandleTypeDef *huart) {
    vofa_huart = huart;
    vofa_last_send_tick = 0;
    vofa_yaw_cmd_speed = 0.0f;
    vofa_pitch_cmd_speed = 0.0f;
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
 * @brief 立即发送一帧VOFA CSV数据 send
 * @note  格式:
 *        time_ms,target_valid,pixel_dx,pixel_dy,
 *        yaw_cmd_speed,pitch_cmd_speed,
 *        yaw_motor_speed,pitch_motor_speed,
 *        yaw_angle,pitch_angle,yaw_current,pitch_current
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

    if (period_ms == 0U) {
        period_ms = VOFA_DEBUG_DEFAULT_PERIOD_MS;
    }

    if (now - vofa_last_send_tick >= period_ms) {
        vofa_last_send_tick = now;
        VOFA_DebugSendNow();
    }
}
