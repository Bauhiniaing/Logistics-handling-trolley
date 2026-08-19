#ifndef __MYTIM_H
#define __MYTIM_H
#include "system.h"

#define RAD_PER_DEG (PI / 180.0f)
extern  int tim4_delay ; //0-->2000 = 1s
extern int delay_flag;  // 延时完成标志
extern u8 flag_stop ;

void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);
void TIM2_int(void);
void TIM4_int(void);
void My_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif
