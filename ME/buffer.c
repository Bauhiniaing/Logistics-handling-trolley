#include "buffer.h"

float percent[8] = {0}; 
uint8_t state[8] = {0}; 
float startPos[8] = {0};
volatile u32 buffer_tick_ms = 0;

/**
  * @brief  舵机位置缓启动插值函数
  * @param  id         舵机编号，对应数组下标
  * @param  duration   插值持续时间，单位 ms
  * @param  targetPos  目标角度/目标位置（角度变化量）
  * @retval realPos[id] 当前插值计算出来的位置
  */

void Buffer_Tick(void)
{
    buffer_tick_ms++;
}

float Slow_Get_pos(int id, float duration, float targetPos)
{
    static float realPos[8] = {0};
    static float lastTarget[8] = {0};
    static u32 lastTick[8] = {0};
    u32 nowTick;
    u32 elapsed;
    
    if(id < 0 || id >= 8)
    {
        return targetPos;
    }

    if(duration <= 0.0f)
    {
        percent[id] = 1.0f;
        state[id] = 2;
        realPos[id] = targetPos;
        return realPos[id];
    }
    
    nowTick = buffer_tick_ms;

    if(state[id] != 1 && ((targetPos - lastTarget[id]) > 0.001f || (lastTarget[id] - targetPos) > 0.001f))// 检测目标位置是否发生变化
    {
        state[id] = 1;
    }

    switch(state[id])
    {
        case 0:
            // 给定默认起始位置为0，如果需要可以修改为实际的舵机当前位置
            startPos[id] = 0.0f; 
            state[id] = 1;
        case 1:
            percent[id] = 0.0f;
            lastTarget[id] = targetPos;
            lastTick[id] = nowTick;
            realPos[id] = startPos[id];
            state[id] = 2;
            startPos[id] = realPos[id];
            break;

        case 2:
            elapsed = nowTick - lastTick[id];
            if(elapsed > 0U)
            {
                lastTick[id] = nowTick;
                // duration（时长）单位为毫秒，因为 Buffer_Tick() 由 TIM4 定时器每 1ms 调用一次。
                percent[id] += (float)elapsed / duration;

                if(percent[id] >= 1.0f)
                {
                    percent[id] = 1.0f;
                    state[id] = 3;
                }

                realPos[id] = (1.0f - percent[id]) * startPos[id] + percent[id] * targetPos;
            }
            break;

        case 3:
            // 到达目标位置
            realPos[id] = targetPos;
            startPos[id] = targetPos; // 更新起始位置为目标位置，以便下一次插值使用
            break;

        default:
            state[id] = 0;
            break;
    }

    return realPos[id];
}

void Clean_parameter(void)
{
	for(int i=0;i<8;i++)
    {
        percent[i]=0;
        state[i] = 1;// 设置为1表示跳过case 0，直接进入case 1，避免重新记录起始位置
        startPos[i]=0;
    }
    buffer_tick_ms=0;
}
