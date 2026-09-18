#include "key.h"



void Key_Init(void)
{ 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 	//使能PA端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;		//Key-->PA.0\PA.1 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 		//输入上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 	//根据设定参数初始化GPIOA.0\GPIOA.1
	
}



void DipSwitch_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;				 //DipSwitch-->PC0~PC3 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 		//输入上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 			//根据设定参数初始化GPIOA.0~7
}

u8 DipSwitch_Read()
{
	DipSwitchDef SwitchNum;
	SwitchNum.BITS.Bit0 = PCin(3);
	SwitchNum.BITS.Bit1 = PCin(2);
	SwitchNum.BITS.Bit2 = PCin(1);
	SwitchNum.BITS.Bit3 = PCin(0);
	return SwitchNum.Num;
}



