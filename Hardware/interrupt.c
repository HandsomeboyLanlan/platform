#include "interrupt.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == motor_yaw.hcan1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
            if ((rx_header.StdId & 0xFF) == motor_yaw.id) {
                QD4310_Update(&motor_yaw, rx_data);
            }
        }
    }
}
