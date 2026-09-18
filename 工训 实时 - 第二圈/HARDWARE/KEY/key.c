#include "key.h"
#include "delay.h"
#include "OLED.h"
#include "Stepping.h"
#include "uart.h"
extern u8 uart_tx3[];
//按键变量
u8 Key1_Flag;
u8 Key1_Count;
u16 Key1_Counter;
u8 Key1_Time;

u8 Key2_Flag;
u8 Key2_Count;
u16 Key2_Counter;
u8 Key2_Time;

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

//按键识别程序
void Key_Scan(void)
{
	//KEY1
	if( Key1 == 0 && Key1_Flag == 0 )
	{
		Key1_Flag = 1;
	}
	else if( Key1 == 0 && Key1_Flag == 2 )
	{
		Key1_Flag = 3;
		Key1_Count++;
	}
	else if( Key1 == 1 )
	{
		Key1_Time = 0;
		Key1_Flag = 0;
	}
	//KEY2
	if( Key2 == 0 && Key2_Flag == 0 )
	{
		Key2_Flag = 1;
	}
	else if( Key2 == 0 && Key2_Flag == 2 )
	{
		Key2_Flag = 3;
		Key2_Count++;
	}
	else if( Key2 == 1 )
	{
		Key2_Time = 0;
		Key2_Flag = 0;
	}
}

//按键执行程序
/**
  * 函    数：按键执行程序
  * 参    数：无
  * 返 回 值：无
  * 注意事项：按键一控制任务起始
			  按键二控制OLED翻页
  */
extern u8 Sensor1_flag;
extern u8 Sensor2_flag;
extern u8 Posture_Clear;
extern u8 uart_tx1[];
void Key_execute(void)		//按键执行程序
{
	//KEY1
	if( Key1_Count == 1 )
	{
		Key1_Count = 0;
//		Start_flag =1;
//		go(2000/Wheel_Circum*SPR, g_step_accel, g_step_decel, g_set_speed);
//		go(20, 15, 15, 15);
//		go(150/Wheel_Circum*SPR, 50, 50, 100);		
//		back(30/Wheel_Circum*SPR, g_step_accel, g_step_decel, g_set_speed);
//		right(200, 15, 15, 15);
//		left(300/Wheel_Circum*Wheel_Left_Right*SPR, LR_step_accel, LR_step_decel, LR_set_speed);
//		turn_left(9/Wheel_Yaw*SPR, LR_step_accel,LR_step_decel, LR_set_speed);
//		turn_right(9/Wheel_Yaw*SPR, LR_step_accel, LR_step_decel, LR_set_speed);
	}
	//KEY2
	if(Key2_Count == 1)
	{
		Key2_Count = 0;		
//		Posture_Clear++;
//		back(2000/Wheel_Circum*SPR, 700, 700, 1000);
//		OLED_NUM++;
//		right(200/Wheel_Circum*SPR, 20, 20, 30);
//		turn_left(900/Wheel_Yaw*SPR, g_step_accel,g_step_decel, g_set_speed);		
//		uart_tx3[0]=66;
//		DMA1_USART3_TxData(uart_tx3,8);//串口发送数组uart_tx1[0]为数据第一位
	}
}

void Key_Control(void)
{
	//KEY1
		if( Key1_Flag == 1 ) 
		{
			if( Key1_Time++ >= 20 ) 	Key1_Flag = 2;//按下Key1 20*Time_Ms生效
		}
		if( Key2_Flag == 1 ) 
		{
			if( Key2_Time++ >= 20 ) 	Key2_Flag = 2;//按下Key2 20*Time_Ms生效
		}
}
