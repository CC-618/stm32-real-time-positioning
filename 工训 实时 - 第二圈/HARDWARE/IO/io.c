#include "io.h"



void Step_IO_Init(void)
{
	//方向-->PB3-->1号轮\PB4-->2号轮\PB5-->3号轮\PB6-->4号轮
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//使能AFIO端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB端口时钟
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);// 改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;		//INPUT-->PB.3\PB.4\PB.5\PB.6\PB.7 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 				//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 			//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 			//根据设定参数初始化GPIOB.3\GPIOB.4\GPIOB.5\GPIOB.6\GPIOB.7
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	 	//使能PD端口时钟
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 	//INPUT-->PD.2 端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 				//推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
//	GPIO_Init(GPIOD, &GPIO_InitStructure);					 	//根据设定参数初始化GPIOD.2	
	
	//方向-->PA4-->1号轮\PA5-->2号轮\PA6-->3号轮\PA7-->4号轮	
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;		//INPUT-->PA4\PA5\PB.6 端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 				//推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 			//IO口速度为50MHz
//	GPIO_Init(GPIOA, &GPIO_InitStructure);		//根据设定参数初始化
	
	//使能-->PC4-->1号轮\PC5-->2号轮\PB0-->3号轮\PB1-->4号轮
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 				//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 			//IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);   // PC
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 				//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 			//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);   // PB
	
	
}
