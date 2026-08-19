#include "my_usart.h"
#include "usart.h"
#include "system.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "tjc_usart_hmi.h"

u8 rec_data1;
u8 rec_data3;
u8 rec_data4;
u8 rec_data5;
u8 rec_data6;
u8 data1;
volatile u16 data[3] = {0};
volatile u8 servoID;
volatile u8 motor42ID;
volatile int16_t motor42_target_angle = 0;
volatile uint16_t vision_values[4] = {0};
volatile uint8_t vision_data_ready = 0;

void jieshou_mission(u8 ucData)
{
    static uint8_t frame[10];
    static uint8_t count = 0;
    uint8_t i;

    if (count == 0)
    {
        if (ucData == 0x51) frame[count++] = ucData;
        return;
    }

    if (ucData == 0x51)
    {
        count = 1;
        frame[0] = ucData;
        return;
    }

    frame[count++] = ucData;
    if (count < sizeof(frame)) return;

    if (frame[9] == 0x52)
    {
        for (i = 0; i < 4; i++)
        {
            vision_values[i] = ((uint16_t)frame[2 + i * 2] << 8)
                             | frame[1 + i * 2];
        }
        vision_data_ready = 1;
    }
    count = 0;
}

void vision_uart_process_byte(uint8_t byte)
{
    jieshou_mission(byte);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(huart->Instance == TJC_UART_INS)//判断是由哪个串口触发的中断
    {
			uint8_t byte = RxBuffer[0];

      osMessageQueuePut(UartRxQueueHandle, &byte, 0, 0);

      HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);// 重新使能串口1接收中断
	  }
		
		if(huart->Instance == USART3)
		{
			jieshou(rec_data3);
			HAL_UART_Receive_IT(&huart3, (u8 *)&rec_data3, 1);//再次开启串口接收数据
		}                                                  
																											 
    if(huart->Instance == UART4)                       
		{                                                  
																											 
			HAL_UART_Receive_IT(&huart4, (u8 *)&rec_data4, 1);//再次开启串口接收数据
		}                                                  
																											 
		if(huart->Instance == UART5)                       
		{                                                  
																											 
			HAL_UART_Receive_IT(&huart5, (u8 *)&rec_data5, 1);//再次开启串口接收数据
		}                                                  
																											 
		if(huart->Instance == USART6)                      
		{                                                  
																											 
			HAL_UART_Receive_IT(&huart6, (u8 *)&rec_data6, 1);//再次开启串口接收数据
		}

}


void jieshou(u8 ucData)         // 接收串口发来数据
{
    static u8 ucRxBuffer[20];           // 缓冲数组 (10就够了，写20防溢出)
    static u8 ucRxCnt = 0;              // 接收计数器
    ucRxBuffer[ucRxCnt++] = ucData;	    // 将收到的数据存入缓冲区中
    
    if (ucRxBuffer[0] != 0x19)          // 数据头不对，则重新开始寻找 0x34
    {
        ucRxCnt = 0;
        return;
    }
    
    if(ucRxCnt < 4)
    {
        return; 
    }
	
   
    if(ucRxBuffer[3] == 0x34)
    {
			data1 = (ucRxBuffer[1] << 8) | ucRxBuffer[2];
      ucRxCnt = 0; 
    }
    else
    {
      
        ucRxCnt = 0; 
    }
}

//void jieshou_hmi(u8 ucData)
//{
//    static u8 hmiRxBuffer[4];
//    static u8 hmiRxCnt = 0;
//    uint16_t angle;

//    if (hmiRxCnt == 0)
//    {
//        if (ucData == 0xA5)
//        {
//            hmiRxBuffer[hmiRxCnt++] = ucData;
//        }
//        return;
//    }

//    hmiRxBuffer[hmiRxCnt++] = ucData;

//    if (hmiRxCnt < 4)
//    {
//        return;
//    }

//    if (hmiRxBuffer[3] == 0x5A)
//    {
//        angle = (uint16_t)hmiRxBuffer[1]
//              | ((uint16_t)hmiRxBuffer[2] << 8);

//        if (angle <= 270)
//        {
//            data = (float)angle;
//        }
//    }

//    hmiRxCnt = 0;
//}

void jieshou_hmi(u8 ucData)
{
    static u8 hmiRxBuffer[5];
    static u8 hmiRxCnt = 0;

    uint16_t servoAngle[3];
    int16_t motorAngle;

    /* 等待帧头 */
    if (hmiRxCnt == 0)
    {
        if ((ucData == 0xA5) || (ucData == 0xB5))
        {
            hmiRxBuffer[0] = ucData;
            hmiRxCnt = 1;
        }

        return;
    }

    hmiRxBuffer[hmiRxCnt++] = ucData;

    if (hmiRxCnt < 5)
    {
        return;
    }

    /* 舵机帧：A5 数据低字节 数据高字节 5A */
    if ((hmiRxBuffer[0] == 0xA5) && (hmiRxBuffer[4] == 0x5A))
    {
				servoID = hmiRxBuffer[1];
        servoAngle[servoID-1] = (uint16_t)hmiRxBuffer[2]	| ((uint16_t)hmiRxBuffer[3] << 8);

        if (servoAngle[servoID-1] <= 270)
        {
            data[servoID-1] = (float)servoAngle[servoID-1];
        }
    }

    /* 步进电机帧：B5 数据低字节 数据高字节 5B */
    else if ((hmiRxBuffer[0] == 0xB5) && (hmiRxBuffer[4] == 0x5B))
    {
				motor42ID = hmiRxBuffer[1];
        motorAngle = (int16_t)((uint16_t)hmiRxBuffer[2] | ((uint16_t)hmiRxBuffer[3] << 8));

        motor42_target_angle = motorAngle;
    }

    hmiRxCnt = 0;
}
