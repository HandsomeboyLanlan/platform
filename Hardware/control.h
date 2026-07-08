#ifndef __CONTROL_H
#define __CONTROL_H

#include "QD4310.h"

/* MaixCam数据结构体 */
typedef struct {
    float pixel_dx;         // pixel_dx MaixCam目标X坐标 - 屏幕中心X坐标
    float pixel_dy;         // pixel_dy MaixCam目标Y坐标 - 屏幕中心Y坐标
    uint8_t target_valid;   // target_valid 是否识别到目标
} Maixcam_Data_t;

void Control_Init(void);
void Gimbal_GotoZero(void);
void Set_Motor_Speed(QD4310_t *motor, float speed);
void Gimbal_Track(Maixcam_Data_t maixcam_data);

#endif
