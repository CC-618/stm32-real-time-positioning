#include "pwm.h"



//-----------------------------------------------------------------------------
//函数名称：TIM1_PWM_Init
//函数功能：TIM1 PWM 初始化函数，采用 PA8 PA11引脚输出，
//
//**********本程序使用TIM1_CH1N、TIM1_CH2N、TIM1_CH3N******
//
//输入参数：arr：自动重装值
//          psc：时钟预分频数
//-----------------------------------------------------------------------------

void TIM1_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//使能定时器1
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIOA外设和AFIO复用功能模块时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;           //GPIOA8, GPIOA11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化PA8,PA9
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); //Timer1重映射PA8\PA9，在使用TIM1_CH1N、TIM1_CH2N、TIM1_CH3N时，不能重映射
	
	TIM_TimeBaseStructure.TIM_Period = arr - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;  //定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
	
	//初始化TIM1 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低

	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC1
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC4
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1	
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);        //MOE 主输出使能,高级定时器必须开启这个

}  


//-----------------------------------------------------------------------------
//函数名称：TIM1_PWM_Init
//函数功能：TIM1 PWM 初始化函数，采用 PA8 PA11引脚输出，
//
//**********本程序使用TIM1_CH1N、TIM1_CH2N、TIM1_CH3N******
//
//输入参数：arr：自动重装值
//          psc：时钟预分频数
//-----------------------------------------------------------------------------

void TIM3_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能定时器3
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIOC外设和AFIO复用功能模块时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;           //GPIOC6, GPIOC7, GPIOC8, GPIOC9,
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);              //初始化PC6,PC7,PC8,PC9
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer1重映射PA8\PA9，在使用TIM1_CH1N、TIM1_CH2N、TIM1_CH3N时，不能重映射
	
	TIM_TimeBaseStructure.TIM_Period = arr - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;  //定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM1 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低

	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC2
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC4
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM1	
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);        //MOE 主输出使能,高级定时器必须开启这个

}  


//-----------------------------------------------------------------------------
//函数名称：PWM_out
//函数功能：PWM 输出函数
//输入参数：channel：PWM 输出通道，通过 PWMch1、PWMch2 、PWMch3 、PWMch4 、PWMch5 、PWMch6 进行参数传递
//          pulsePer：PWM 高电平的百分比 ，两位小数，整型变量
//----------------------------------------------------------------------------- 	

void PWM_out(uint8_t channel, int pulsePer)
{
	switch(channel)
	{
		case 1:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare1(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 2:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare2(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 3:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare3(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 4:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare4(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 5:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare1(TIM1, (uint32_t) (pulsePer * PulsePeriodA / 10000) );break;
		case 6:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare4(TIM1, (uint32_t) (pulsePer * PulsePeriodA / 10000) );break;
		default:break;	
	}
}




