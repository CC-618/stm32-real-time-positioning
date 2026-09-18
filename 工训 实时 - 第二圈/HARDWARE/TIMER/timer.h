#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"



void tim2_init(int ms); //TIM2定时器初始化
void tim3_init(u16 arr,u16 psc); //TIM3定时器初始化
void tim1_init(u16 arr,u16 psc);//TIM4定时器初始化
#endif



