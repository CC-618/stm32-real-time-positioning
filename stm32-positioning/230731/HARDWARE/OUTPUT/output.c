#include "output.h"



void OUTPUT_Init(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 	//使能PC端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	//OUTPUT-->PC.10\PC.11 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 	//根据设定参数初始化GPIOC.10\GPIOC.11

		
	OUTPUT1 = OUTPUTOff;
	OUTPUT2 = OUTPUTOff;
	
}



