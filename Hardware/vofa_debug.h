#ifndef __VOFA_DEBUG_H
#define __VOFA_DEBUG_H

#include "main.h"
#include "QD4310.h"

#define VOFA_DEBUG_DEFAULT_PERIOD_MS 50U    // VOFA默认发送周期，单位ms period
#define VOFA_PID_CMD_BUFFER_SIZE 64U    // VOFA串口在线调参命令长度

/**
 * @brief 初始化VOFA调试串口 init
 * @param huart VOFA使用的串口句柄，例如&huart3
 */
void VOFA_DebugInit(UART_HandleTypeDef *huart);

/**
 * @brief 更新电机控制命令速度 update
 * @param motor 电机句柄
 * @param speed 当前下发给电机的目标速度，单位rpm
 */
void VOFA_DebugUpdateCommand(QD4310_t *motor, float speed);

/**
 * @brief VOFA串口接收完成回调，用于在线修改PID参数
 * @param huart 触发接收中断的串口句柄
 * @note  支持命令: PID,YAW,kp,ki,kd 或 PID,PITCH,kp,ki,kd
 */
void VOFA_DebugRxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief 立即发送一帧VOFA调试数据 send
 */
void VOFA_DebugSendNow(void);

/**
 * @brief 按固定周期发送VOFA调试数据 send
 * @param period_ms 发送周期，单位ms；传0时使用默认周期 default
 */
void VOFA_DebugSendPeriod(uint32_t period_ms);

#endif
