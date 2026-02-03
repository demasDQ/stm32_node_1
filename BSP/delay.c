#include "delay.h"

// DWT相关寄存器定义
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEM_CR      (*(volatile uint32_t*)0xE000EDFC)
#define DBGMCU_CR   (*(volatile uint32_t*)0xE0042004)

static uint32_t fac_us = 0;  // us延时倍乘数

// DWT延时初始化函数
void delay_init(void)
{
    // 使能跟踪时钟
    DEM_CR |= 0x01000000;
    
    // 使能DWT
    DWT_CTRL |= 0x01;
    
    // 重置CYCCNT寄存器
    DWT_CYCCNT = 0;
    
    // 计算每微秒的时钟周期数
    fac_us = SystemCoreClock / 1000000;
}

// 微秒级延时函数 - 基于DWT实现
void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    
    // 如果fac_us未初始化，则先初始化
    if(fac_us == 0)
    {
        delay_init();
    }
    
    ticks = us * fac_us;            // 需要的节拍数
    told = DWT_CYCCNT;              // 刚进入时的计数器值
    
    while(1)
    {
        tnow = DWT_CYCCNT;
        if(tnow != told)
        {
            if(tnow > told)
            {
                tcnt += tnow - told;
            }
            else
            {
                // 计数器溢出情况
                tcnt += (0xFFFFFFFF - told) + tnow;
            }
            told = tnow;
            
            if(tcnt >= ticks)
            {
                break;
            }
        }
    }
}

// 毫秒级延时函数 - 基于DWT实现
void delay_ms(uint32_t ms)
{
    uint32_t i;
    for(i = 0; i < ms; i++)
    {
        delay_us(1000);  // 调用微秒延时函数
    }
}