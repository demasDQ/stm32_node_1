#ifndef __DELAY_H
#define __DELAY_H

#include "../../Core/Inc/main.h"

// DWT延时初始化函数
void delay_init(void);

// 微秒级延时函数 - 基于DWT实现
void delay_us(uint32_t us);

// 毫秒级延时函数 - 基于DWT实现
void delay_ms(uint32_t ms);

#endif