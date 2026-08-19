#include "main.h"//包含头文件#include "stm32f4xx_hal.h"
#include "can.h"//包含用cube配置的函数
#include "Motor_can.h"//can通信编写
#include "Motor.h"//存放motor结构体
#include "dji_pid.h"

Motor_Msg m6020[8];//管理8个 M6020 型号电机的测量数据
Motor_Msg m2006[8];//管理8个 M2006 型号电机的测量数据

CAN_TxHeaderTypeDef Can1_tx_msg;  //管理Can1发送配置
CAN_TxHeaderTypeDef Can2_tx_msg;  //管理Can2发送配置
uint8_t             can1_send_data[8];//存放要发的can1的数据
uint8_t             can2_send_data[8];//存放要发的can2的数据

int change_speed_flag=0;
float pos[8];
float speeds[8];
float speeds1[8];

//初始化滤波器
void Can_Filter_Init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;//使能滤波器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32位宽
    can_filter_st.FilterIdHigh = 0x0000;//下四行共同设置0000为报文全通模式
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//选择队列
	// CAN1 滤波器
    can_filter_st.FilterBank = 0;//0号滤波器
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//将设置的值回传
    HAL_CAN_Start(&hcan1);//开启can
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能RX中断
	// CAN2 滤波器
    can_filter_st.SlaveStartFilterBank = 14; //从属节点从14号开始
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * 
 * @brief    接收电机的信息（具体是通过回调函数接收的，这更像一个中间函数）
 * @brief    具体作用为将电机信息放入Motor_Msg结构体中
*/
void Get_msg(Motor_Msg *msg,uint8_t Date[] )
{
    msg->last_angle = msg->angle;
    msg->angle = (uint16_t)(Date[0] << 8 | Date[1]);
    msg->speed = (uint16_t)(Date[2] << 8 | Date[3]);
    msg->torque = (Date[4] << 8 | Date[5]);
    msg->temperate = Date[6];

    if(msg->angle - msg->last_angle > 4096)//脉冲4096=1圈
        msg->round_cnt --;
    else if (msg->angle - msg->last_angle < -4096) 
        msg->round_cnt ++;
    //u1s1 ?    
    //计算总角度，考虑了多圈旋转和初始偏移
    msg->total_angle = msg->round_cnt * 8192 + msg->angle - msg->offset_angle;//？
    //将总角度转换为实际角度
    msg->real_angle = msg->total_angle / 22.75555556f;//？
}

/**
 * @Brief HAL库自带的弱定义中断回调函数，用于处理 CAN 接收 FIFO  中的消息待处理事件
 * //前面使能了中断回调，当中断时在此编辑接收函数，没使能一定需要先使能!
 * @brief 回调函数，接收电机数据（正宫）
 * @param[in]      hcan, 句柄 
*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RX_1;//can1 接6020
    CAN_RxHeaderTypeDef RX_2;//can2 接2006

    uint8_t Rx1_Date[8];//放can1的数据
    uint8_t Rx2_Date[8];//放can2的数据

    if(hcan->Instance == CAN1)//can1时
    {
        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RX_1,Rx1_Date);
            switch (RX_1.StdId)
                {
                 case 0x205://对应电调 ID 为 1 的电机.下同
                 case 0x206:
                 case 0x207:
                 case 0x208:
                 case 0x209:
                 case 0x20A:
                 case 0x20B:
                 case 0x20C:
                    {
                        static uint8_t i = 0;
                        static uint8_t offset_sign[8] = {0}; //给八个电机上电角度标志位
                        //?
                        i = RX_1.StdId - 0x205;//获取电机的ID
                        Get_msg(&m6020[i],Rx1_Date);

                        if(offset_sign[i] == 0)  //只记一次上电角度
                        {
                            get_moto_offset(&m6020[i]);//获取上电角度？
                            offset_sign[i]++;
                        }
                    
                        break;
                    }
                    default:
                    {
                        break;
                    }

                }

    }           

    if(hcan->Instance == CAN2)//can2      时
    {
			HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RX_2,Rx2_Date);
            switch (RX_2.StdId)
                {
                 case 0x201://对应电调 ID 为 1 的电机.下同
                 case 0x202:
                 case 0x203:
                 case 0x204:
                 case 0x205:
                 case 0x206:
                 case 0x207:
                 case 0x208:
                    {
                        static uint8_t i = 0;
                        static uint8_t offset_sign[8] = {0}; //给八个电机上电角度标志位
                        //?
                        i = RX_2.StdId - 0x201;//获取电机的ID
                        Get_msg(&m2006[i],Rx2_Date);
                        if(offset_sign[i] == 0)  //只记一次上电角度
                        {
                            get_moto_offset(&m2006[i]);
                            offset_sign[i]++;
                        }
                    
                        break;
                    }
                    default:
                    {
                        break;
                    }

                }
    }    
}

//发送部分编写

void Motor_Control_Update(void)
{		
	  uint8_t i;

    // M6020 电机控制
    for(i = 0; i < 8; i++)
    {
        m6020set[i].current = PID_motoloop(&Motor_PID[i + change_speed_flag],&m6020[i],speeds1[i],pos[i]);
				m2006set[i].current = PID_motoloop(&Motor_PID[i + 8 + change_speed_flag],&m2006[i],speeds1[i],pos[i]);
    }
    Send_Can1_Msg1(m6020set[0].current, m6020set[1].current, m6020set[2].current, m6020set[3].current);
		Send_Can1_Msg2006(m2006set[0].current, m2006set[1].current, m2006set[2].current, m2006set[3].current);
//		for(i = 0; i < 8; i++)
//    {
//        m2006set[i].current = PID_motoloop(&Motor_PID[i +8 + change_speed_flag],&m2006[i],speeds[i],pos[i]);
//    }
//    Send_Can2_Msg1(m2006set[0].current, m2006set[1].current, m2006set[2].current, m2006set[3].current);
//		Send_Can2_Msg1(m2006set[4].current, m2006set[5].current, m2006set[6].current, m2006set[7].current);
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 6020电机控制电压, 范围 [-30000,30000] 
  * @param[in]      motor2: (0x202) 6020电机控制电压, 范围 [-30000,30000] 
  * @param[in]      motor3: (0x203) 6020电机控制电压, 范围 [-30000,30000] 
  * @param[in]      motor4: (0x204) 6020电机控制电压, 范围 [-30000,30000]
  * @retval         none
  */
void Send_Can1_Msg1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Can1_tx_msg.StdId = 0x1FF;      //发送给的电机ID
    Can1_tx_msg.IDE = CAN_ID_STD;   //表示发送为标准帧
    Can1_tx_msg.RTR = CAN_RTR_DATA; //表示发送为数据帧
    Can1_tx_msg.DLC = 0x08;         //data长度

    can1_send_data[0] = motor1 >> 8;//发送为16位数据。所以2段为一个数据
    can1_send_data[1] = motor1;
    can1_send_data[2] = motor2 >> 8;//
    can1_send_data[3] = motor2;     
    can1_send_data[4] = motor3 >> 8;
    can1_send_data[5] = motor3;
    can1_send_data[6] = motor4 >> 8;
    can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &Can1_tx_msg, can1_send_data, &send_mail_box);
}
void Send_Can1_Msg2006(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Can1_tx_msg.StdId = 0x200;      //发送给的电机ID
    Can1_tx_msg.IDE = CAN_ID_STD;   //表示发送为标准帧
    Can1_tx_msg.RTR = CAN_RTR_DATA; //表示发送为数据帧
    Can1_tx_msg.DLC = 0x08;         //data长度

    can1_send_data[0] = motor1 >> 8;//发送为16位数据。所以2段为一个数据
    can1_send_data[1] = motor1;
    can1_send_data[2] = motor2 >> 8;//
    can1_send_data[3] = motor2;     
    can1_send_data[4] = motor3 >> 8;
    can1_send_data[5] = motor3;
    can1_send_data[6] = motor4 >> 8;
    can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &Can1_tx_msg, can1_send_data, &send_mail_box);
}
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x205) 6020电机控制电流, 范围 [-30000,30000] 
  * @param[in]      motor2: (0x206) 6020电机控制电流, 范围 [-30000,30000] 
  * @param[in]      motor3: (0x207) 6020电机控制电流, 范围 [-30000,30000] 
  * @param[in]      motor4: (0x208) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
void Send_Can1_Msg2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Can1_tx_msg.StdId = 0x2FF;      //发送给的电机ID
    Can1_tx_msg.IDE = CAN_ID_STD;   //表示发送为标准帧
    Can1_tx_msg.RTR = CAN_RTR_DATA; //表示发送为数据帧
    Can1_tx_msg.DLC = 0x08;         //data长度

    can1_send_data[0] = motor1 >> 8;//发送为16位数据。所以2段为一个数据
    can1_send_data[1] = motor1;
    can1_send_data[2] = motor2 >> 8;//
    can1_send_data[3] = motor2;     
    can1_send_data[4] = motor3 >> 8;
    can1_send_data[5] = motor3;
    can1_send_data[6] = motor4 >> 8;
    can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &Can1_tx_msg, can1_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: (0x202) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor3: (0x203) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor4: (0x204) 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void Send_Can2_Msg1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Can2_tx_msg.StdId = 0x200;      //发送给的电机ID
    Can2_tx_msg.IDE = CAN_ID_STD;   //表示发送为标准帧
    Can2_tx_msg.RTR = CAN_RTR_DATA; //表示发送为数据帧
    Can2_tx_msg.DLC = 0x08;         //data长度

    can2_send_data[0] = motor1 >> 8;//发送为16位数据。所以2段为一个数据
    can2_send_data[1] = motor1;
    can2_send_data[2] = motor2 >> 8;//
    can2_send_data[3] = motor2;     
    can2_send_data[4] = motor3 >> 8;
    can2_send_data[5] = motor3;
    can2_send_data[6] = motor4 >> 8;
    can2_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &Can2_tx_msg, can2_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x205) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: (0x206) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor3: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor4: (0x208) 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void Send_Can2_Msg2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Can2_tx_msg.StdId = 0x1FF;      //发送给的电机ID
    Can2_tx_msg.IDE = CAN_ID_STD;   //表示发送为标准帧
    Can2_tx_msg.RTR = CAN_RTR_DATA; //表示发送为数据帧
    Can2_tx_msg.DLC = 0x08;         //data长度

    can2_send_data[0] = motor1 >> 8;//发送为16位数据。所以2段为一个数据
    can2_send_data[1] = motor1;
    can2_send_data[2] = motor2 >> 8;//
    can2_send_data[3] = motor2;     
    can2_send_data[4] = motor3 >> 8;
    can2_send_data[5] = motor3;
    can2_send_data[6] = motor4 >> 8;
    can2_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &Can2_tx_msg, can2_send_data, &send_mail_box);
}

/*this function should be called after system+can init */
void get_moto_offset(Motor_Msg *msg)
{
    msg->offset_angle = msg->angle;
    msg->round_cnt = 0;//防止角度差8192即一圈
    msg->total_angle = 0.0f;
    msg->real_angle = 0.0f;
}
