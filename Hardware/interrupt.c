#include "interrupt.h"
#include "config.h"
#include "usart.h"
#include "vofa_debug.h"

static volatile uint8_t g_maixcam_ready;  // 新数据就绪标志
static FrameState_t g_maixcam_frame_state = FRAME_HEAD1;
static uint8_t g_maixcam_frame_buf[FRAME_DATA_LEN];
static uint8_t g_maixcam_frame_idx;

/**
  * @brief  重置MaixCam串口解析状态
  * @note   串口错误或半包异常后调用，避免旧数据影响下一帧
  */
static void MaixCam_ResetParser(void) {
    g_maixcam_frame_state = FRAME_HEAD1;
    g_maixcam_frame_idx = 0;
}

/**
  * @brief  启动MaixCam串口DMA空闲接收
  * @note   MaixCam上电较慢或串口异常后，可重复调用恢复接收
  */
void MaixCam_StartReceive(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(uart1_rx_data_handle.huart,
                                 uart1_rx_data_handle.uart_rx_buffer,
                                 MAIXCAM_RX_BUFFER_SIZE);
}

/**
  * @brief  MaixCam串口数据解析函数，逐字节解析
  * @param  byte: 接收到的字节
  */
static uint8_t MaixCam_ParseByte(uint8_t byte) {
    switch (g_maixcam_frame_state) {
        case FRAME_HEAD1:
            if (byte == FRAME_DATA_HEAD1) g_maixcam_frame_state = FRAME_HEAD2;
            break;
        case FRAME_HEAD2:
            if (byte == FRAME_DATA_HEAD2) {
                g_maixcam_frame_state = FRAME_DATA;
                g_maixcam_frame_idx  = 0;
            } else if (byte != FRAME_DATA_HEAD1) {
                g_maixcam_frame_state = FRAME_HEAD1;
            }
            break;
        case FRAME_DATA:
            g_maixcam_frame_buf[g_maixcam_frame_idx ++] = byte;
            if (g_maixcam_frame_idx >= FRAME_DATA_LEN) g_maixcam_frame_state = FRAME_TAIL1;
            break;
        case FRAME_TAIL1:
            if (byte == FRAME_DATA_TAIL1) {
                g_maixcam_frame_state = FRAME_TAIL2;
            } else {
                g_maixcam_frame_state = (byte == FRAME_DATA_HEAD1) ? FRAME_HEAD2 : FRAME_HEAD1;
            }
            break;
        case FRAME_TAIL2:
            if (byte == FRAME_DATA_TAIL2) {
                int16_t x = (int16_t)(g_maixcam_frame_buf[1] | ((int16_t)g_maixcam_frame_buf[2] << 8));
                int16_t y = (int16_t)(g_maixcam_frame_buf[3] | ((int16_t)g_maixcam_frame_buf[4] << 8));

                maixcam_data_handle.target_valid = g_maixcam_frame_buf[0];
                if (g_maixcam_frame_buf[0]) {
                    maixcam_data_handle.pixel_dx = (float)x;
                    maixcam_data_handle.pixel_dy = (float)y;
                }
                g_maixcam_ready = 1;
            }
            g_maixcam_frame_state = (byte == FRAME_DATA_HEAD1) ? FRAME_HEAD2 : FRAME_HEAD1;
            break;
    }
    return 0;
}

/* 查询是否有新数据，同时清除标志 */
uint8_t MaixCam_DataReady(void) {
    uint8_t ret = g_maixcam_ready;
    g_maixcam_ready = 0;
    return ret;
}

/**
  * @brief  CAN接收中断回调函数
  * @param  hcan: CAN句柄
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == motor_yaw_handle.hcan1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
            uint8_t motor_id = rx_header.StdId & 0xFF;
            if (motor_id == motor_yaw_handle.id) {
                QD4310_Update(&motor_yaw_handle, rx_data);
            } else if (motor_id == motor_pitch_handle.id) {
                QD4310_Update(&motor_pitch_handle, rx_data);
            }
        }
    }
}

/**
  * @brief  UART空闲中断回调，逐字节解析MaixCam帧
  * @param  huart: UART句柄
  * @param  Size: DMA收到的数据长度
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        for (uint16_t i = 0; i < Size; i++) {
            MaixCam_ParseByte(uart1_rx_data_handle.uart_rx_buffer[i]);
        }
        /* 重新启动DMA接收 */
        MaixCam_StartReceive();
    }
}

/**
  * @brief  UART错误回调函数
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // MaixCam由控制板供电，上电过程可能产生串口错误；清解析状态并重新开启接收。
        MaixCam_ResetParser();
        HAL_UART_AbortReceive(huart);
        MaixCam_StartReceive();
    }
}

/**
  * @brief  UART接收完成中断回调，用来进行vofa调试
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        VOFA_DebugRxCpltCallback(huart);
    }
}
