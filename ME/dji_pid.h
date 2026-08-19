#ifndef _PID_1_H
#define _PID_1_H
#include "main.h"//包含头文件#include "stm32f4xx_hal.h"
#include "math.h"
#include "Motor.h"
#define PI 3.14159265358979323846264338f
typedef struct
{
    uint8_t mode;
    //速度环即内环参数
    float in_Kp;  
    float in_Ki;
    float in_Kd;

    float in_max_out;  //最大输出
    float in_max_I_out; //最大积分输出

    float in_set;     //内环期望
    float in_get;     //内环输入

    float in_out;     //内环总输出
    float in_P_out;    //比例项输出
    float in_I_out;    //积分项输出
    float in_D_out;    //微分项输出
    float in_D_buf[3];  //微分项 0最新 1上一次 2上上次
    float in_error[3]; //误差项 0最新 1上一次 2上上次
	  //位置环即内环
	  float out_Kp;
    float out_Ki;
    float out_Kd;

    float out_max_out;  //最大输出
    float out_max_D_out; //最大积分输出
    float deadband;	//只有外环设置了死区 即死区内不计算
		
    float out_set;     //期望位置
    float out_get;     //位置输入

    float out_out;     //外环总输出
    float out_P_out;    //比例项输出
    float out_I_out;    //积分项输出
    float out_D_out;    //微分项输出
    float out_D_buf[3];  //微分项 0最新 1上一次 2上上次
    float out_error[3]; //误差项 0最新 1上一次 2上上次

    float ag_Kp;  
    float ag_Ki;
    float ag_Kd;

    float ag_max_out;  //最大输出
    float ag_max_I_out; //最大积分输出

    float ag_set;     //角度期望
    float ag_get;     //角度实际

    float ag_out;     //内环总输出
    float ag_P_out;    //比例项输出
    float ag_I_out;    //积分项输出
    float ag_D_out;    //微分项输出
    float ag_D_buf[3];  //微分项 0最新 1上一次 2上上次
    float ag_error[3]; //误差项 0最新 1上一次 2上上次

} PID_typedef;//电机pid设定结构体，存放着所有要用的pid参数


typedef struct
{
    float current;//电流
    float setspeed;//期望速度
    float setpos;  //期望位置
    float setangle;
} PID_motor;//存放pid输出的电流

void PID_motorinit(PID_typedef *pid,uint8_t mode,float PID[7], float max_out[2], float max_I_D_out[2], float deadband);
void Motor_PID_Init(void);
float PID_motoloop(PID_typedef *pid, Motor_Msg*msg, float setspeed, float setpos);
float PID_Angle(PID_typedef *pid,Motor_Msg *msg,float setangle);
extern PID_typedef Motor_PID[16];			
extern PID_motor m6020set[8];
extern PID_motor m2006set[8];
extern uint8_t change;
#endif
