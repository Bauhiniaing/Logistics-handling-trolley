#ifndef __MISSION_H
#define __MISSION_H

#include "stdint.h"
#include "stdbool.h"

#define pia 3.14159265358979323846264338327950288419716939937510

extern uint32_t pos_pulse(float pos);
extern float zuobiao_x_pos(float x);
extern float zuobiao_z_pos(float z);

void motor_pos_Init(void);
void moter2006_test(float speed);
void moter6020_test(float speed);
void motor42_Init(void);
void motor42_SetPos(uint8_t addr, int32_t pos, uint16_t vel, uint8_t acc, bool raF, bool snF);
void motor42_SetPos2(uint8_t addr, int32_t pos, uint16_t vel, uint8_t acc, bool snF);
void motor42_SetVel(uint8_t addr, int16_t vel, uint8_t acc, bool snF);

#endif

