#ifndef _MAIN_H_
#define _MAIN_H_

#include "sys.h"
#include "def.h"
#include "delay.h"
#include "uart.h"
#include "led.h"
#include "buzzer.h"
#include "io.h"
#include "output.h"
#include "key.h"
#include "can.h"
#include "wdg.h"
#include "timer.h"
#include "adc.h"
#include "oled.h"
#include "spi.h"
#include "pwm.h"
#include "pid.h"
#include "kalanman.h"
#include "encoder.h"
//CAN通信波特率设置
//#define CanBrt  Baund100
//#define CanBrt  Baund125
//#define CanBrt  Baund250
#define CanBrt  Baund500
//#define CanBrt  Baund1000

//CAN总线本地ID
#define ID_Self 	0x08100000

//定时中断中时间值
u32 OS_Counter;
u32 LED_Counter;

//串口接收数组
u8	RX1_BUF[USART1_MAX_RX_LEN];
u8	RX2_BUF[USART2_MAX_RX_LEN];
u8	RX3_BUF[USART3_MAX_RX_LEN];

u8 uart_tx4[]={9,0,1,2,3,4,5,6};
//串口发送数组
u8 uart_tx1[]={9,0,1,2,3,4,5,6};
u8 uart_tx2[10]={8,0,1,2,3,4,5,6,7,8};
u8 uart_tx3[]={7,0,1,2,3,4,5,6};
u8 uart2_Tx_flag;

//CAN发送数组
u8 CAN_tx_buff[8]={0,1,2,3,4,5,6,7};

//拨码开关变量
u8 Switch_num;

//AD转换结果数组
u16	ADC_Value[9];
u8 AD_Read_flag;	//读取AD值标志位

//PID控制结构体变量
PID_TypeDef SpeedL,SpeedR;

//按键变量
u8 Key1_Flag;
u8 Key1_Count;
u8 Key1_Time;

u8 Key2_Flag;
u8 Key2_Count;
u8 Key2_Time;

//显示变量
u8	Display_flag;

//JY931
JY931_TypeDef uart2_tx_buf;





//PID参数初始化
void PID_init(void)
{
	SpeedL.P_set = 1.0;
	SpeedL.I_set = 0.0;
	SpeedL.D_set = 0.0;
	SpeedL.I_max = 100.0;
	SpeedL.Out_max = 100.0;
	
	SpeedR.P_set = 1.0;
	SpeedR.I_set = 0.0;
	SpeedR.D_set = 0.0;
	SpeedR.I_max = 100.0;
	SpeedR.Out_max = 100.0;
}

//系统初始化
void HardWare_init(void)
{
			//看门狗
	delay_init();			//延时函数初始化	  
	LED_Init();				//指示灯初始化
	Key_Init();				//按键初始化
	DipSwitch_Init();		//拨码开关初始化
	BUZZER_Init();			//蜂鸣器初始化
	tim2_init(2);			//2(ms)
//	//以上初始化不要动
//	
//	OUTPUT_Init();			//MOS输出初始化
	IO_Init();				//输入输出初始化
//	
//	ADC_Config(ADC_Value);			//ADC初始化
	
//	OLED_Init();			//OLED初始化
//	
//	TIM1_PWM_Init(PulsePeriodA,72);		//PWM输出初始化    PulsePeriodA：PWM周期；72：为分频系数，分频后为 1MHz
//	TIM3_PWM_Init(PulsePeriodC,72);		//PWM输出初始化    PulsePeriodC：PWM周期；72：为分频系数，分频后为 1MHz
//		
	UART1_init(921600);		//串口1初始化为115200
	UART2_init(115200);		//串口2初始化为115200
	UART3_init(921600);		//串口3初始化为115200
		
	CAN_Mode_Init(CanBrt, CAN_Mode_Normal);//CAN总线初始化，250kbps，回环模式  CAN_Mode_LoopBack  ，  CAN_Mode_Normal
	
//	PID_init();
	
	delay_ms(500);
}


//读取AD转换结果
void Read_ad(void)
{
//	OLED_ShowSignedNum(1, 1, ADC_Value[8], 5);
	
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
//键值处理
void Key_execute(void)		//按键执行程序
{
	u8 i;
	//KEY1
	if( Key1_Count == 1 )
	{
		Key1_Count = 0;
		OLED_ShowString(3, 12, "KEY1");
		CAN_tx_buff[0] = 0x05;
		Can_Send_Msg(ID_Self, CAN1, CAN_tx_buff, 1);
	}
	
	if( Key2_Count == 1 )
	{
		Key2_Count = 0;
		OLED_ShowString(3, 12, "KEY2");
		CAN_tx_buff[0] = 0x06;
		Can_Send_Msg(ID_Self, CAN1, CAN_tx_buff, 1);
	}
}

//解读接收到的JY931的数据
void TransJY931(void)
{
	if(RX1_BUF[0] == 44)
	{
		AccX = RX1_BUF[3] + (RX1_BUF[4] << 8) ;
		AccY = RX1_BUF[5] + (RX1_BUF[6] << 8) ;
		AccZ = RX1_BUF[7] + (RX1_BUF[8] << 8) ;
		
		WX = RX1_BUF[14] + (RX1_BUF[15] << 8) ;
		WY = RX1_BUF[16] + (RX1_BUF[17] << 8) ;
		WZ = RX1_BUF[18] + (RX1_BUF[19] << 8) ;
		
		Roll = RX1_BUF[25] + (RX1_BUF[26] << 8) ;
		Pitch = RX1_BUF[27] + (RX1_BUF[28] << 8) ;
		Yaw = RX1_BUF[29] + (RX1_BUF[30] << 8) ;
		
		HX = RX1_BUF[36] + (RX1_BUF[37] << 8) ;
		HY = RX1_BUF[38] + (RX1_BUF[39] << 8) ;
		HZ = RX1_BUF[40] + (RX1_BUF[41] << 8) ;
	}
}

//显示程序
void display(void)
{
//	OLED_ShowSignedNum(1, 1, AccX, 10);
//	OLED_ShowSignedNum(2, 1, AccY, 10);
//	OLED_ShowSignedNum(3, 1, AccZ, 10);
	
//	OLED_ShowSignedNum(2, 1, WX, 4);
//	OLED_ShowSignedNum(2, 6, WY, 4);
//	OLED_ShowSignedNum(2, 12, WZ, 4);
//	
	OLED_ShowSignedNum(1, 1, 5, 10);
	OLED_ShowSignedNum(2, 1, 6, 10);
	OLED_ShowSignedNum(3, 1, 7, 10);
//	
//	OLED_ShowSignedNum(4, 1, HX, 4);
//	OLED_ShowSignedNum(4, 6, HY, 4);
//	OLED_ShowSignedNum(4, 12, HZ, 4);
	
}


void car_data_uart(float x,float y,float Angle)
{
	
	uart2_tx_buf.byte.X = (long)(x * 100);
	uart2_tx_buf.byte.Y = (long)(y * 100);
	uart2_tx_buf.byte.Angle = (long)(Angle * 100);
	DMA1_USART2_TxData(uart2_tx_buf.array, 12);
}






#endif




