#include "pwm.h"

/**
 * @description: TIM3定时器初始化
 * @param N: 单位us，定时器时间间隔
 * @return None
 */

void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM3_OCInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能定时器3
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIOC外设和AFIO复用功能模块时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;           //GPIOC6, GPIOC7, GPIOC8, GPIOC9,
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);              //初始化PC6,PC7,PC8,PC9
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);			//结构体中元素初始化为默认值	
	
	TIM_TimeBaseStructure.TIM_Period = arr;     	//自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler = psc;   			//设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;  		//设置时钟分割:不分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 	//指定的参数初始化TIMx的时间基数单位																														
	
/* Output Compare Toggle Mode configuration: Channel1 */
	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;//输出比较翻转模式
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM3_OCInitStructure.TIM_Pulse = 0;//设置通道1的比较值（跳变值），也就是比较寄存器的初始值
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM3, &TIM3_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable); //使能或者失能TIMx在CCR1上的预装载寄存器

	/* Output Compare Toggle Mode configuration: Channel2 */
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM3_OCInitStructure.TIM_Pulse = 0;//设置通道2的比较值（跳变值），也就是比较寄存器的初始值
	TIM_OC2Init(TIM3, &TIM3_OCInitStructure);

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	/* Output Compare Toggle Mode configuration: Channel3 */
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM3_OCInitStructure.TIM_Pulse = 0;//设置通道2的比较值（跳变值），也就是比较寄存器的初始值
	TIM_OC3Init(TIM3, &TIM3_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	/* Output Compare Toggle Mode configuration: Channel4 */
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM3_OCInitStructure.TIM_Pulse = 0;//设置通道2的比较值（跳变值），也就是比较寄存器的初始值
	TIM_OC4Init(TIM3, &TIM3_OCInitStructure);

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	TIM_Cmd(TIM3,ENABLE);
//	TIM_ITConfig(TIM3,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,ENABLE);
	
	/*中断*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//-----------------------------------------------------------------------------
//函数名称：PWM_out
//函数功能：PWM 输出函数
//输入参数：channel：PWM 输出通道，通过 PWMch1、PWMch2 、PWMch3 、PWMch4 、PWMch5 、PWMch6 进行参数传递
//          pulsePer：PWM ，整型变量，高电平的百分比 ，两位小数
//----------------------------------------------------------------------------- 	
//PWM周期；72：为分频系数，分频后为 1000000Hz
void PWM_out(uint8_t channel, int pulsePer)
{
	switch(channel)
	{
		case 1:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare1(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 2:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare2(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 3:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare3(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 4:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare4(TIM3, (uint32_t) (pulsePer * PulsePeriodC / 10000) );break;
		case 5:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare1(TIM1, (uint32_t) (pulsePer * PulsePeriodA / 10000) );break;
		case 6:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare2(TIM1, (uint32_t) (pulsePer * PulsePeriodA / 10000) );break;		
		case 7:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare3(TIM1, (uint32_t) (pulsePer * PulsePeriodA / 10000) );break;
		case 8:if(pulsePer > 0 && pulsePer <= 10000) TIM_SetCompare4(TIM1, (uint32_t) (pulsePer * PulsePeriodA / 10000) );break;
		default:break;	
	}
}




