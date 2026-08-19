#include "Mission.h"
#include "mytim.h"
#include "Motor_can.h"
#include "X_V2_can.h"
#include "Emm_V5.h"
#include "bsp_servo.h"

/**
  * @brief    
  * @param    addr：电机地址
  * @param    dir ：方向，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)，范围0?5000RPM
  * @retval   返回值含义
  */
void motor_pos_Init(void)
{
  change_speed_flag = 4;  //使用 Motor_PID[4..7]，对应GM6020位置环PID
	moter6020_test(50.0f);   //设置位置环最大转速，单位：转每秒(rev/s)
}

/**
  * @brief    设置速度环最大转速，单位：转每秒(rev/s)
  * @param    speed ：速度，单位：转每秒(rev/s)
  * @retval   返回值含义
  */
void moter2006_test(float speed)//设置速度环最大转速，单位：转每秒(rev/s)
{
	speeds[0]=speed;
	speeds[1]=speed;
	speeds[2]=speed;
	speeds[3]=speed;
	speeds[4]=speed;
	speeds[5]=speed;
	speeds[6]=speed;
	speeds[7]=speed;
}

/**
  * @brief    设置位置环最大转速，单位：转每秒(rev/s)
  * @param    speed ：速度，单位：转每秒(rev/s)
  * @retval   返回值含义
  */
void moter6020_test(float speed)//设置位置环最大转速，单位：转每秒(rev/s)
{
	speeds1[0]=speed;
	speeds1[1]=speed;
	speeds1[2]=speed;
	speeds1[3]=speed;
	speeds1[4]=speed;
	speeds1[5]=speed;
	speeds1[6]=speed;
	speeds1[7]=speed;
}

/**
  * @brief    初始化42电机
  * @param    无
  * @retval   无
  */
void motor42_Init(void)
{
	X_V2_SelectCAN(&hcan2);//选择42电机CAN口
	Emm_V5_Reset_CurPos_To_Zero(1);//上电后最好先清零，实测用处不大
	Emm_V5_En_Control(1, true, false);//使能电机，关闭电机自锁
}

/**
  * @brief    计算X方向坐标对应的角度
  * @param    x ：X方向位移
  * @retval   角度
  */
float zuobiao_x_pos(float x)
{
    /* 保留符号：负的位移代表反向旋转 */
    return x * (360.0f / 150.0f);
}

/**
  * @brief    计算Z方向坐标对应的角度
  * @param    z ：Z方向位移
  * @retval   角度
  */
float zuobiao_z_pos(float z)
{
    /* 齿条行程：每转行程 = 17 齿 × π × 1.5?mm */
    return z * (360.0f / (17.0f * 1.5f * (float)pia));
}

/**
  * @brief    角度转脉冲数
  * @param    pos ：角度
  * @retval   pulse : 脉冲数
  */
//脉冲数 = 角度 ÷ 360 × 每圈脉冲数//
////每圈脉冲数 = 200 × 16 = 3200////
uint32_t pos_pulse(float pos)
{
    if (pos <= 0.0f)
    {
        return 0;
    }

    return (uint32_t)(pos * 3200.0f / 360.0f + 0.5f);
}

/**
  * @brief    设置42电机位置
  * @param    addr：电机地址
  * @param    dir ：方向        //0为CW，其余值为CCW
	* @param    pos ：设定角度    //角度通过计算转化成脉冲数
  * @param    vel ：速度(RPM)   //范围0 - 5000RPM
  * @param    acc ：加速度      //范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      //范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void motor42_SetPos(uint8_t addr, int32_t pos, uint16_t vel, uint8_t acc, bool raF, bool snF)//设置42电机位置
{
	static u8 dir;
	static uint32_t angle;
	if(pos >= 0)
	{
		dir = 0;
		angle = (u32)pos;
	}
	else
	{
		angle = (u32)(-(int64_t)pos);
		dir = 1;
	}
	Emm_V5_Pos_Control(addr, dir, vel, acc, pos_pulse(angle), raF, snF);
}

/**
  * @brief    设置42电机位置（绝对位置）
  * @param    addr：电机地址
  * @param    dir ：方向        //0为CW，其余值为CCW
	* @param    pos ：设定角度    //角度通过计算转化成脉冲数
  * @param    vel ：速度(RPM)   //范围0 - 5000RPM
  * @param    acc ：加速度      //范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      //范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void motor42_SetPos2(uint8_t addr, int32_t pos, uint16_t vel, uint8_t acc, bool snF)
{
    static int32_t last_pos[8];
    static bool pos_initialized[8];

    uint8_t dir;
    uint32_t angle;

    // 该地址第一次调用一定发送；以后只有角度变化才发送
    if (pos_initialized[addr] && pos == last_pos[addr])
    {
        return;
    }

    pos_initialized[addr] = true;
    last_pos[addr] = pos;

    if (pos >= 0)
    {
        dir = 0;
        angle = (uint32_t)pos;
    }
    else
    {
        dir = 1;
        angle = (uint32_t)(-(int64_t)pos);
    }

    Emm_V5_Pos_Control( addr, dir, vel, acc, pos_pulse(angle), true/*绝对位置模式*/, snF);
}

/**
  * @brief    设置42电机速度
  * @param    addr：电机地址
  * @param    dir ：方向，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   //范围0 - 5000RPM
  * @param    acc ：加速度      //范围0 - 255，注意：0是直接启动
  * @param    snF ：是否发送指令，true为发送，false为不发送
  * @retval   返回值含义
  */
void motor42_SetVel(uint8_t addr, int16_t vel, uint8_t acc, bool snF)//设置42电机速度
{
	static u8 dir;
	static u16 speed;
	if(vel >= 0)
	{
		dir = 0;
		speed = (uint16_t)vel;
	}
	else
	{
		dir = 1;
		speed = (uint16_t)(-(int64_t)vel);
	}
	Emm_V5_Vel_Control( addr, dir, speed, acc, snF);
}

void find_materiel(void)
{
		Servo_SetAngle(&htim2, 0, TIM_CHANNEL_1, 70);

}