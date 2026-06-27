#ifndef __MY_CAN_H
#define __MY_CAN_H

#include "main.h"
#include "QD4310.h"
#include "can.h"

extern QD4310_t Motor_0;

void CAN_InterfaceInit(void);

#endif
