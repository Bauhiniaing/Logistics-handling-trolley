#ifndef __CAN_1
#define __CAN_1
#include "can.h"
#include "Motor.h"//存放motor结构体

void Can_Filter_Init(void);
void Get_msg(Motor_Msg *msg,uint8_t Date[] );
void Motor_Control_Update(void);
void Send_Can1_Msg1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Send_Can1_Msg2006(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Send_Can1_Msg2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Send_Can2_Msg1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Send_Can2_Msg2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void get_moto_offset(Motor_Msg *msg);

extern Motor_Msg m6020[8];//管理8个 M6020 型号电机的测量数据
extern Motor_Msg m2006[8];//管理8个 M2006 型号电机的测量数据
extern int change_speed_flag;
extern float pos[8];
extern float speeds[8];
extern float speeds1[8];



#endif
