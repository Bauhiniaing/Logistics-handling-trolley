#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"
#include "MOTOR.h"
#include "Motor_can.h"
#include "mytim.h"
#include "math.h"
#include "lcd.h"
#include "dji_pid.h"

#define DLY_TIM_Handle (&htim14)                      
int TIM2_mode = 1, TIM6_mode = 1, TIM7_mode = 1;
int tim4_delay = 0; // 0-->2000 = 1s
int delay_flag = 0; //延时完成标志位
uint8_t lcd_refresh_flag = 0;

/**
  * @brief  微秒级延时
  * @param  nus: 延时时间（微秒）
  * @retval 无
  */
void delay_us(uint16_t nus)
{
  __HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
  __HAL_TIM_ENABLE(DLY_TIM_Handle);

  while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
  {
  }

  __HAL_TIM_DISABLE(DLY_TIM_Handle);
}

/**
  * @brief  毫秒级延时
  * @param  nms: 延时时间（毫秒）
  * @retval 无
  */
void delay_ms(uint16_t nms)
{
  uint32_t i;

  for (i = 0; i < nms; i++)
    delay_us(1000);
}

/**
  * @brief  TIM周期Elapsed回调函数
  * @param  htim: TIM句柄
  * @retval 无
  */
void My_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM4)//1ms
  {
//    TIM4_int();
  }

  if (htim->Instance == TIM3)//1ms
  {
		
		Motor_Control_Update();
			
  } // htim->Instance == TIM3

  if (htim->Instance == TIM2)
  {

    TIM2_int();
  }
}

void TIM2_int(void)
{
}

void TIM4_int(void)
{

//  Buffer_Tick();
//  Time4_delay();

  if (tim4_delay > 0)
  {
    tim4_delay--;
    if (tim4_delay == 0)
    {
      delay_flag = 1; 
    }
  }
  static uint16_t lcd_cnt = 0;
    lcd_cnt++;
    if(lcd_cnt >= 100)
    {
        lcd_cnt = 0;
        lcd_refresh_flag = 1;
    }
}

