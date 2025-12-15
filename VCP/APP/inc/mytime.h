#ifndef MYTIME_H
#define MYTIME_H


#include "main.h"
#include "mytime.h"

/* 启动 TIM14（使能定时器中断） */
void mytime_start(TIM_HandleTypeDef *htim);

/* 停止 TIM14（关闭定时器中断） */
void mytime_stop(TIM_HandleTypeDef *htim);


#endif
