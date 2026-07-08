#ifndef __CONFIG_H
#define __CONFIG_H

#include "main.h"
#include "QD4310.h"
#include "pid.h"
#include "control.h"
#include "interrupt.h"
#include "usart.h"

extern PID_t pid_yaw_handle, pid_pitch_handle;
extern QD4310_t motor_yaw_handle, motor_pitch_handle;
extern Maixcam_Data_t maixcam_data_handle;
extern UART_Rx_Data_t uart1_rx_data_handle;

#endif
