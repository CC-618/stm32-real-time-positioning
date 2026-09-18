#ifndef __PWM_H
#define __PWM_H
#include "stm32f10x.h"
#include "def.h"
#include "sys.h"


#define  PWMch1  1		//C6
#define  PWMch2  2		//C7
#define  PWMch3  3		//C8
#define  PWMch4  4		//C9
#define  PWMch5  5		//A8
#define  PWMch6  6		//A11
#define  PWMch7  7		//A11
#define  PWMch8  8		//A11

#define  PulsePeriodA    1000		//PWM 通道1 arr值，输出周期，1ms(1000Hz)
#define  PulsePeriodC    100		//PWM 通道1 arr值，输出周期，100us(10KHz)



//-----------------------------------------------------------------------------
//函数名称：TIM1_PWM_Init
//函数功能：TIM1 PWM 初始化函数，采用 PA8  PA9 引脚输出
//输入参数：arr：自动重装值
//          psc：时钟预分频数
//-----------------------------------------------------------------------------
void TIM3_PWM_Init(u16 arr,u16 psc);

//-----------------------------------------------------------------------------
//函数名称：PWM_out
//函数功能：PWM 输出函数
//输入参数：channel：PWM 输出通道，通过 PWMch1、PWMch2 进行参数传递
//          pulsePer：PWM 高电平的百分比 
//----------------------------------------------------------------------------- 	
void PWM_out(uint8_t channel, int pulsePer);


#endif
