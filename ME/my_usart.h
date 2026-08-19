#ifndef __MY_USART_H__
#define __MY_USART_H__

#include "system.h"
#include "cmsis_os2.h"

extern u8 rec_data1;
extern u8 rec_data3;
extern u8 rec_data4;
extern u8 rec_data5;
extern u8 rec_data6;
extern u8 data1;
extern volatile u16 data[3];
extern volatile u8 servoID;
extern volatile u8 motor42ID;
extern volatile int16_t motor42_target_angle;
extern volatile uint16_t vision_values[4];
extern volatile uint8_t vision_data_ready;

extern osMessageQueueId_t UartRxQueueHandle;

void jieshou(u8 ucData);
void jieshou_hmi(u8 ucData);
void vision_uart_process_byte(uint8_t byte);
void jieshou_mission(u8 ucData);

#endif

