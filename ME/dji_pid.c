#include "main.h"//包含头文件#include "stm32f4xx_hal.h"
#include "dji_pid.h"
#include "Motor.h"//存放motor结构体
/*
 Kp
    可以加快系统的响应速度，但过大会导致系统不稳定或振荡
 Ki
    可以减小稳态误差，但过大会导致系统响应变慢或振荡
 Kd
    可以减少超调和振荡，但过大会对噪声敏感，导致控制信号波动
*/

/**********限幅函数**********/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//PID_typedef;电机pid设定结构体，存放着所有要用的pid参数
PID_typedef Motor_PID[16] = {0};//第x个电机的PID设置（共16个）
//PID_motor;存放pid输出的电流
PID_motor m6020set[8];
PID_motor m2006set[8];
uint8_t change;


/**
  * @name          Motor_PID_Init
  * @brief         1.用于电机初始化———>给定内外环的kp|ki|kd
  * @brief         2.运用PID_motoinit将设定的pos值，speed值初始化
  * @param        
  * @retval     
  * 
  */
void Motor_PID_Init(void)
{
  
    static uint8_t i[2];//不懂有什么用
		/* M2006 参数 */
		static float PID_2006[7] ={1000.0f, 6.0f, 0.0f, 0.2f, 0.0f, 0.006f, 0.0f}; //前三个/* 速度环 Kp Ki Kd */    后三个/* 位置环 Kp Ki Kd */
		static float max_out_2006[2] ={10000.0f, 400.0f};//分别限幅速度环最大电压输出和角度环的最大外环速度
		static float max_I_D_out_2006[2] ={2000.0f, 100.0f};
    /* GM6020 参数，初次测试限制较小输出 */
		static float PID_6020[7] ={1000.0f, 3.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f};//前三个/* 速度环 Kp Ki Kd */    后三个/* 位置环 Kp Ki Kd */
		static float max_out_6020[2] ={10000.0f, 180.0f};//分别限幅速度环最大电压输出和角度环的最大外环速度
		static float max_I_D_out_6020[2] ={1000.0f, 100.0f};

    // for(i[0] = 0; i[0] < 4; i[0]++)
    // {
        PID_motorinit(&Motor_PID[0], 0, PID_6020, max_out_6020, max_I_D_out_6020, 0);
        PID_motorinit(&Motor_PID[1], 0, PID_6020, max_out_6020, max_I_D_out_6020, 0);
        PID_motorinit(&Motor_PID[2], 0, PID_6020, max_out_6020, max_I_D_out_6020, 0);
        PID_motorinit(&Motor_PID[3], 0, PID_6020, max_out_6020, max_I_D_out_6020, 0);
		
        PID_motorinit(&Motor_PID[4], 1, PID_6020, max_out_6020, max_I_D_out_6020, 0);
        PID_motorinit(&Motor_PID[5], 1, PID_6020, max_out_6020, max_I_D_out_6020, 0);
        PID_motorinit(&Motor_PID[6], 1, PID_6020, max_out_6020, max_I_D_out_6020, 0);
        PID_motorinit(&Motor_PID[7], 1, PID_6020, max_out_6020, max_I_D_out_6020, 0);
				
				PID_motorinit(&Motor_PID[8], 0, PID_2006, max_out_2006, max_I_D_out_2006, 0);
				PID_motorinit(&Motor_PID[9], 0, PID_2006, max_out_2006, max_I_D_out_2006, 0);
				PID_motorinit(&Motor_PID[10],0, PID_2006, max_out_2006, max_I_D_out_2006, 0);
				PID_motorinit(&Motor_PID[11],0, PID_2006, max_out_2006, max_I_D_out_2006, 0);
		
				PID_motorinit(&Motor_PID[12],1, PID_2006, max_out_2006, max_I_D_out_2006, 0);
				PID_motorinit(&Motor_PID[13],1, PID_2006, max_out_2006, max_I_D_out_2006, 0);
				PID_motorinit(&Motor_PID[14],1, PID_2006, max_out_2006, max_I_D_out_2006, 0);
				PID_motorinit(&Motor_PID[15],1, PID_2006, max_out_2006, max_I_D_out_2006, 0);
//    }

    for(i[1] = 0; i[1] < 4; i[1]++)
    {
        m6020set[i[1]].setpos = 0;
        m6020set[i[1]].setspeed = 0;
        m6020set[i[1] + 4].setpos = 0;
        m6020set[i[1] + 4].setspeed = 0;
        m2006set[i[1]].setpos = 0;
        m2006set[i[1]].setspeed = 0;
        m2006set[i[1] + 4].setpos = 0;
        m2006set[i[1] + 4].setspeed = 0;
    }
}


/**
  * @name          中间函数，只在Motor_PID_Init里运用
  * @brief         将PID_init设置的内外环值kp|i|d放入
  * @param         mode: 0:速度环PID  1:位置环PID
  * @param         PID: 0: in_kp, 1: in_ki, 2:in_kd | 3: out_kp, 4: out_ki, 5:out_kd
  * @param         max_out: 0: 内环pid最大输出 1：外环pid最大输出
  * @param         max_ID_out: 0：内环pid最大积分输出 1：外环最大微分输出
	* @param         deadband: 外环死区
  * @retval         none
  */
void PID_motorinit(PID_typedef *pid,uint8_t mode,float PID[7], float max_out[2], float max_I_D_out[2], float deadband)
{
    //内环初始化
    pid->mode = mode;//原本模仿代码用来选择内外环的，目前不懂有什么用，先注释
    pid->in_Kp = PID[0];//将Motor_PID_Init里设置的PID[]丢入
    pid->in_Ki = PID[1];
    pid->in_Kd = PID[2];

    pid->in_max_out = max_out[0];//内环最大输出
    pid->in_max_I_out = max_I_D_out[0];//0为内环最大积分输出

    pid->in_D_buf[0] = pid->in_D_buf[1] = pid->in_D_buf[2] = 0.0f;//初始化微分项的
    pid->in_error[0] = pid->in_error[1] = pid->in_error[2] = pid->in_P_out = pid->in_I_out = pid->in_D_out = pid->in_out = 0.0f;


    //外环初始化
     pid->out_Kp = PID[3];//同理如上
    pid->out_Ki = PID[4];
    pid->out_Kd = PID[5];
    pid->out_max_out = max_out[1];
    pid->out_max_D_out = max_I_D_out[1];
    pid->deadband = deadband;
    pid->out_D_buf[0] = pid->out_D_buf[1] = pid->out_D_buf[2] = 0.0f;
    pid->out_error[0] = pid->out_error[1] = pid->out_error[2] = pid->out_P_out = pid->out_I_out = pid->out_D_out = pid->out_out = 0.0f;


    //angle
    pid->ag_Kp = PID[6];
    pid->ag_P_out=pid->ag_error[0]=pid->ag_error[1]=pid->ag_error[2]= 0.0f;
}

/**
  * @name  PID_motoloop
  * @brief  PID闭环 PID公式
  * @param	speed:期望速度（是尾转转速，要换算成rad/s，之前用的都是rad/min）
  * @param	Pos:期望位置（一定要是角度值，如要转两圈，值就是720）	  
  * @param	deadband:死区（只有用位置环才需要死区，该死区指的是度数，如死区为2，即在2度误差内不做调整）
  * @param   mode；速度环0/位置环1
  * @retval 串级PID计算后的值
*/
float PID_motoloop(PID_typedef *pid,Motor_Msg*msg, float setspeed, float setpos)
{
        // 保存当前模式，以便下次调用时可以检查是否发生了模式切换
        // static uint8_t last_mode[16] = {0};
        // uint8_t motor_index = pid - Motor_PID; // 计算当前电机索引
        
        // 检查模式是否发生变化
        // if(last_mode[motor_index] != mode)
        // {
        //     // 模式发生变化，重置PID控制器状态
        //     pid->in_I_out = 0;
        //     pid->out_I_out = 0;
        //     pid->in_error[0] = pid->in_error[1] = pid->in_error[2] = 0;
        //     pid->out_error[0] = pid->out_error[1] = pid->out_error[2] = 0;
            
        //     // 更新模式
        //     last_mode[motor_index] = mode;
        // }
    if(pid ->mode == 1)//位置环PID
    {
        pid->out_max_out = setspeed;//将外环总输出限制设置为期望速度，因为外环计算出的就是速度值
        pid->out_error[2] = pid->out_error[1];//增量式PID要用到上上次的误差值，位置式PID不用，此参数是无用的；
        pid->out_error[1] = pid->out_error[0];//外环上一次误差
        pid->out_set = setpos;          //期望位置
        pid->out_get = msg->real_angle; //实际位置（要将编码器值8192换算成360°，方便计算）
        pid->out_error[0] = pid->out_set - pid->out_get;

        if(fabs(pid->out_error[0]) < pid->deadband) //死区内，不做计算
            return 0;

        pid->out_P_out = pid->out_Kp * pid->out_error[0];                   //Pout
				LimitMax(pid->out_P_out, pid->out_max_out);
        pid->out_I_out += pid->out_Ki * pid->out_error[0];                   //Iout
        pid->out_D_out = pid->out_Kd * (pid->out_error[0] - pid->out_error[1]); //Dout
        LimitMax(pid->out_D_out, pid->out_max_D_out);
        pid->out_out = pid->out_P_out + pid->out_I_out + pid->out_D_out;
				
        pid->in_error[2] = pid->in_error[1];                                    //内环的上上次误差
        pid->in_error[1] = pid->in_error[0];                                    //内环上一次误差
        pid->in_set = pid->out_out;                                      //外环的输出值 = 内环的输入值 = 期望速度
        pid->in_get = msg->speed / 60.0f;                /*有改动*/       //实际速度(注意这里将转速值处理成了rad/s,之前是rad/min)
        pid->in_error[0] = pid->in_set - pid->in_get;                         //速度误差
        pid->in_P_out =  pid->in_Kp * pid->in_error[0];
        pid->in_I_out += pid->in_Ki * pid->in_error[0];
        pid->in_D_out =  pid->in_Kd * (pid->in_error[0] - pid->in_error[1]);
        LimitMax(pid->in_I_out, pid->in_max_I_out);
        pid->in_out = pid->in_P_out + pid->in_I_out + pid->in_D_out;
        LimitMax(pid->in_out, pid->in_max_out);
    }
    else//速度环PID
    {
        pid->in_error[2] = pid->in_error[1];                                  //内环的上上次误差
        pid->in_error[1] = pid->in_error[0];                                  //内环上一次误差
        pid->in_set = setspeed ;                                    //外环的输出值 = 内环的输入值 = 期望速度
        pid->in_get = msg->speed / 60.0f;                /*有改动*/       //实际速度(注意这里将转速值处理成了rad/s,之前是rad/min)
        pid->in_error[0] = pid->in_set - pid->in_get;                         //速度误差
        pid->in_P_out =  pid->in_Kp * pid->in_error[0];
        pid->in_I_out += pid->in_Ki * pid->in_error[0];
        pid->in_D_out =  pid->in_Kd * (pid->in_error[0] - pid->in_error[1]);
        LimitMax(pid->in_I_out, pid->in_max_I_out);
        pid->in_out = pid->in_P_out + pid->in_I_out + pid->in_D_out;
        LimitMax(pid->in_out, pid->in_max_out);
		}
    return pid->in_out;
 }

 float PID_Angle(PID_typedef *pid,Motor_Msg *msg,float setangle)
 {
    pid->ag_error[2] = pid->ag_error[1];                                 
    pid->ag_error[1] = pid->ag_error[0]; 
    pid->ag_set      = setangle;
    pid->ag_get      = msg->real_angle*PI/180;
    pid->ag_error[0] = pid->ag_set - pid->ag_get;
    pid->ag_P_out =  pid->ag_Kp * pid->ag_error[0]; 
    pid->ag_out = pid->ag_P_out;

    return pid->ag_out;

 }

 
