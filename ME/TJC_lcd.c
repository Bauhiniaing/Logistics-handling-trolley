#include "TJC_lcd.h"
#include "tjc_usart_hmi.h"
#include "Motor_can.h"
#include "my_usart.h"

/**
  * @brief  获取速度的绝对值
  * @param  speed: 输入速度值
  * @retval speed的绝对值
  */    
static int speed_magnitude(int16_t speed)
{
    return speed < 0 ? -(int)speed : (int)speed;
}

/**
  * @brief  主界面数据更新函数
  * @param  无
  * @retval 无
  */
void TJC_LCD_Main(void)
{
		tjc_send_val("main.num1", "val", vision_values[0]);
    tjc_send_val("main.num2", "val", vision_values[1]);
    tjc_send_val("main.num3", "val", vision_values[2]);
    tjc_send_val("main.num4", "val", vision_values[3]);
}

/**
  * @brief  测试主界面数据更新函数
  * @param  无
  * @retval 无
  */
void TJC_LCD_TestMain(void)
{
    static int test_value = 0;

    tjc_send_val("main.num1", "val", test_value);
    tjc_send_val("main.num2", "val", test_value + 100);
    tjc_send_val("main.num3", "val", test_value + 200);
    tjc_send_val("main.num4", "val", test_value + 300);

    test_value++;
    if (test_value > 999)
    {
        test_value = 0;
    }
}

/**
  * @brief  LCD数据更新函数
  * @param  heading_deg: 当前航向角度，单位为度
  * @retval 无
  */
void TJC_LCD_Update(int heading_deg)
{
    heading_deg %= 360;
    if (heading_deg < 0)
    {
        heading_deg += 360;
    }

    tjc_send_val("wheel.wheel1", "val", speed_magnitude(m2006[0].speed));
    tjc_send_val("wheel.wheel2", "val", speed_magnitude(m2006[1].speed));
    tjc_send_val("wheel.wheel3", "val", speed_magnitude(m2006[2].speed));
    tjc_send_val("wheel.wheel4", "val", speed_magnitude(m2006[3].speed));
    tjc_send_val("wheel.heading", "val", heading_deg);
}

/**
  * @brief  LCD测试数据更新函数
  * @param  无
  * @retval 无
  */
void TJC_LCD_TestUpdate(void)
{
    static int test_step = 0;
    int phase = test_step % 200;
    int base_speed;

    if (phase <= 100)
    {
        base_speed = phase * 20;
    }
    else
    {
        base_speed = (200 - phase) * 20;
    }

    tjc_send_val("wheel.wheel1", "val", base_speed);
    tjc_send_val("wheel.wheel2", "val", base_speed + 120);
    tjc_send_val("wheel.wheel3", "val", base_speed > 80 ? base_speed - 80 : 0);
    tjc_send_val("wheel.wheel4", "val", base_speed + 40);
    tjc_send_val("wheel.heading", "val", (test_step * 3) % 360);

    test_step++;
}
