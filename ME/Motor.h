#ifndef _MOTOR_H
#define _MOTOR_H

#include "stdint.h"

typedef struct 
{
    int16_t speed;        // 速度
    int16_t angle;        // 角度
    int16_t torque;       // 扭矩
    int16_t temperate;    // 温度
    int16_t offset_angle;     // 角度偏移
    int16_t last_angle;   // 上一次的角度
    int16_t round_cnt;    // 圈数计数
    float total_angle;  // 总角度
    float real_angle;   // 实际角度


}Motor_Msg;//存放6020和2006电机反馈及累计角度信息

#endif
