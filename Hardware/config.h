#ifndef __CONFIG_H
#define __CONFIG_H

#include "main.h"
#include "QD4310.h"
#include "pid.h"

extern PID_t pid_yaw, pid_pitch;
extern QD4310_t motor_yaw, motor_pitch;

#endif
