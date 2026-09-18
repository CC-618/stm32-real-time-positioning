#include "io.h"



void IO_Init(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;		//INPUT-->PB.3\PB.4\PB.5\PB.6\PB.7 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 				//输入上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 			//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 			//根据设定参数初始化GPIOB.3\GPIOB.4\GPIOB.5\GPIOB.6\GPIOB.7
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	 	//使能PD端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 	//INPUT-->PD.2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 				//输入上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 	//根据设定参数初始化GPIOD.2

}



