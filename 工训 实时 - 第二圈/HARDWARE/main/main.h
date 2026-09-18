#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x.h"                  // Device header
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
#include "WP_Ctrl.h"
#include "Encoder.h"
#include "Sensor.h"
#include "Stepping.h"
#include "Seria_Screen.h"
//#include "kalanman.h"

//CAN通信波特率设置
//#define CanBrt  Baund100
//#define CanBrt  Baund125
//#define CanBrt  Baund250
#define CanBrt  Baund500
//#define CanBrt  Baund1000



//定时中断中时间值
u32 OS_Counter;
u32 LED_Counter;

//串口接收数组
u8	RX1_BUF[USART1_MAX_RX_LEN];
u8	RX2_BUF[USART2_MAX_RX_LEN];
u8	RX3_BUF[USART3_MAX_RX_LEN];

//串口发送数组
u8 uart_tx1[]={0};
u8 uart_tx2[]={0,0,0,0,0,0,0,0};
u8 uart_tx3[]={0,0,0,0,0,0,0,0};

//CAN发送数组
u8 CAN_tx_buff[8]={0,0,0,0,0,0,0,0};

//拨码开关变量
u8 Switch_num;

//AD转换结果数组
u16	ADC_Value[9];
u8 AD_Read_flag;	//读取AD值标志位

u8 Time2_ms=1;	//TIM2中断时间=Time_Ms*1ms;
u8 Time3_us=50;	//TIM3中断时间=Time_Ms*1us;
extern u8 USART1_NUM;
extern u8 Incident_flag;
//short	AccX;	//加速度  /32768*16g
//short	AccY;
//short	AccZ; 
//short	WX;		//角速度
//short	WY;
//short	WZ;
//short	Roll;	//角度
//short	Pitch;
//short	Yaw;
//short	HX;		//磁场
//short	HY;
//short	HZ;

JY901_TypeDef   uart3_Rx_buf;


//系统初始化
void HardWare_init(void)
{
//	IWDG_Init(4,625);		//看门狗大概409.6(ms)
	delay_init();			//延时函数初始化	  
//	LED_Init();				//指示灯初始化
	Key_Init();				//按键初始化-->PA.0\PA.1
//	DipSwitch_Init();		//拨码开关初始化
//	BUZZER_Init();			//蜂鸣器初始化
	//以上初始化不要动
	
//	OUTPUT_Init();			//MOS输出初始化 PC.10\PC.11
	Step_IO_Init();			//电机正反转,使能引脚初始化 
//	Encoder_EXTIX_Init();			//旋转编码器初始化 -->PB3\PB4\PB5\PB6

//	ADC_Config(ADC_Value);			//ADC初始化
	
	OLED_Init();			//OLED初始化
	UART1_init(115200);		//串口1初始化为115200 PA9 TX PA10 RX
	UART2_init(115200);		//串口2初始化为115200 PA2 TX PA3 RX
	UART3_init(115200);		//串口3初始化为115200 PB10 TX PB11 RX
		
	CAN_Mode_Init(CanBrt, CAN_Mode_Normal);//CAN总线初始化，250kbps，回环模式  CAN_Mode_LoopBack  ，  CAN_Mode_Normal
	
	PID_init();
//  PA8,9,10,11	
//	TIM1_PWM_Init(PulsePeriodA,72);		//PWM输出初始化    PulsePeriodA：PWM周期；72：为分频系数，分频后为 1MHz
//  PC6,7,8	
//	TIM3_PWM_Init(PulsePeriodC,72);		//电机PWM输出初始化    PulsePeriodC：PWM周期；72：为分频系数，分频后为 1MHz
	TIM3_PWM_Init(0xFFFF,36-1);	//定时器3翻转模式,2MHZ C6-->1号轮 C7-->2号轮 C8-->3号轮 C9-->4号轮
	tim2_init(Time2_ms);	//定时器2中断时间(ms)
	delay_ms(500);
}


//读取AD转换结果
void Read_ad(void)
{
	OLED_ShowSignedNum(1, 1, ADC_Value[8], 5);	
}





#endif




