#include "interrupt.h"
#include "config.h"
#include "usart.h"
#include "vofa_debug.h"

static volatile uint8_t g_maixcam_ready;  // 新数据就绪标志

/**
  * @brief  MaixCam串口数据解析函数，逐字节解析
  * @param  byte: 接收到的字节
  */
static uint8_t MaixCam_ParseByte(uint8_t byte) {
    static FrameState_t state = FRAME_HEAD1;
    static uint8_t buf[FRAME_DATA_LEN];
    static uint8_t idx;

    switch (state) {
        case FRAME_HEAD1:
            if (byte == FRAME_DATA_HEAD1) state = FRAME_HEAD2;
            break;
        case FRAME_HEAD2:
            if (byte == FRAME_DATA_HEAD2) {
                state = FRAME_DATA;
                idx  = 0;
            } else if (byte != FRAME_DATA_HEAD1) {
                state = FRAME_HEAD1;
            }
            break;
        case FRAME_DATA:
            buf[idx ++] = byte;
            if (idx >= FRAME_DATA_LEN) state = FRAME_TAIL1;
            break;
        case FRAME_TAIL1:
            if (byte == FRAME_DATA_TAIL1) {
                state = FRAME_TAIL2;
            } else {
                state = (byte == FRAME_DATA_HEAD1) ? FRAME_HEAD2 : FRAME_HEAD1;
            }
            break;
        case FRAME_TAIL2:
            if (byte == FRAME_DATA_TAIL2) {
                int16_t x = (int16_t)(buf[1] | ((int16_t)buf[2] << 8));
                int16_t y = (int16_t)(buf[3] | ((int16_t)buf[4] << 8));

                maixcam_data_handle.target_valid = buf[0];
                if (buf[0]) {
                    maixcam_data_handle.pixel_dx = (float)x;
                    maixcam_data_handle.pixel_dy = (float)y;
                }
                g_maixcam_ready = 1;
            }
            state = (byte == FRAME_DATA_HEAD1) ? FRAME_HEAD2 : FRAME_HEAD1;
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
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_rx_data_handle.uart_rx_buffer, UART3_RX_BUFFER_SIZE);
    }
}

/**
  * @brief  UART接收完成中断回调
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        VOFA_DebugRxCpltCallback(huart);
    }
}
