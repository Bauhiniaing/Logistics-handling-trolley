#include "bsp_servo.h"
#include "my_usart.h"

float Max_angle[4];
float Min_angle[4] = {0};

void Servo_Init(float servo1,float servo2,float servo3,float servo4)
{
		Max_angle[0] = servo1;
	  Max_angle[1] = servo2;
	  Max_angle[2] = servo3;
	  Max_angle[3] = servo4;
}

uint32_t channel_select(u8 ID)
{
		if(ID == 1)
		return TIM_CHANNEL_1;
		if(ID == 2)
		return TIM_CHANNEL_2;
		if(ID == 3)
		return TIM_CHANNEL_3;
		if(ID == 4)
		return TIM_CHANNEL_4;
		return 0;
}

/**
@brief 设置舵机输出角度，转换为 PWM 比较值并输出
@param htim 定时器句柄指针
@param id 舵机编号，用于读取该舵机的角度限位
@param channel 定时器 PWM 输出通道
@param angle 目标设定角度
@retval 无
*/

void Servo_SetAngle(TIM_HandleTypeDef *htim,uint16_t id,uint32_t channel,float angle)
{
    uint16_t pulse;

    if (angle < Min_angle[id])
        angle = Min_angle[id];

    if (angle > Max_angle[id])
        angle = Max_angle[id];

    pulse = 500 + (uint16_t)(angle * 2000.0f / Max_angle[id]);

    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}


