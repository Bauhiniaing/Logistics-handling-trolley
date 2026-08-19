#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#include "tim.h"

extern float Max_angle[4];
extern float Min_angle[4];

extern uint32_t channel_select(uint8_t ID);

void Servo_Init(float servo1,float servo2,float servo3,float servo4);
void Servo_SetAngle(TIM_HandleTypeDef *htim,uint16_t i,uint32_t channel,float angle);

#endif
