#ifndef __BUFFER_H
#define __BUFFER_H
 
#include "system.h"
 
extern float Slow_Get_pos(int id,float duration,float targetPos);
extern float percent[8]; 
extern uint8_t state[8];

void Buffer_Tick(void);

void Clean_parameter(void);

#endif

