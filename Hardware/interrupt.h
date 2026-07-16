#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "main.h"
#include "can.h"

/*  MaixCam串口帧协议 (共9字节):
 *  帧头: 0xAA 0x55    (2B)
 *  状态: uint8_t      (1B, 0=无目标 1=有目标)
 *  X坐标: int16 LE    (2B, 像素偏差，小端)
 *  Y坐标: int16 LE    (2B, 像素偏差，小端)
 *  帧尾: 0x0D 0x0A    (2B)
 */
#define FRAME_DATA_HEAD1    0xAA
#define FRAME_DATA_HEAD2    0x55
#define FRAME_DATA_TAIL1    0x0D
#define FRAME_DATA_TAIL2    0x0A
#define FRAME_DATA_LEN      5       // status(1) + X(2) + Y(2) 数据帧

#define MAIXCAM_RX_BUFFER_SIZE 32

/* 串口初始化句柄 */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t uart_rx_buffer[MAIXCAM_RX_BUFFER_SIZE];
} UART_Rx_Data_t;

/* MaixCam帧解析状态机 */
typedef enum {
    FRAME_HEAD1,
    FRAME_HEAD2,
    FRAME_DATA,
    FRAME_TAIL1,
    FRAME_TAIL2,
} FrameState_t;

void MaixCam_StartReceive(void);
uint8_t MaixCam_DataReady(void);

#endif
