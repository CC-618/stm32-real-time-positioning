#include "TIM_GPIO.h"
//#include "output.h"
//u16 Freq_Num;
//u16 Freq_Flag;//频率
//u32 Stepz1_Flag=1;//布数计算控制
//u32 Stepz2_Flag=1;//布数计算控制
//u32 Stepz3_Flag=1;//布数计算控制
//u32 Step1_Num;//布数1
//u32 Step2_Num;//布数2
//u32 Step3_Num;//布数3
////Vehicle_Wheel Whee1,Whee2,Whee3;
//void PWM_GPIO(void)
//{
//	if(Whee1.Step>0)
//	{
//		Whee1.Freq_Control++;
//		if(Whee1.Freq_Control == Whee1.Freq_Expect)
//		{
//			Whee1.Step--;
//			PAout(4)=OUTPUTOff;//高电平
//		}
//		else if(Whee1.Freq_Control == 2*Whee1.Freq_Expect)
//		{
//			PAout(4)=OUTPUTOn;
//			Whee1.Freq_Control=0;
//		}
//	}
//	if(Whee2.Step>0)
//	{
//		Whee2.Freq_Control++;
//		if(Whee2.Freq_Control == Whee2.Freq_Expect)
//		{
//			Whee2.Step--;
//			PAout(5)=OUTPUTOff;//高电平
//		}
//		else if(Whee2.Freq_Control == 2*Whee2.Freq_Expect)
//		{
//			PAout(5)=OUTPUTOn;
//			Whee2.Freq_Control=0;
//		}
//	}
//	if(Whee3.Step>0)
//	{
//		Whee3.Freq_Control++;
//		if(Whee3.Freq_Control == Whee3.Freq_Expect)
//		{
//			Whee3.Step--;
//			PAout(6)=OUTPUTOff;//高电平
//		}
//		else if(Whee3.Freq_Control == 2*Whee3.Freq_Expect)
//		{
//			PAout(6)=OUTPUTOn;
//			Whee3.Freq_Control=0;
//		}
//	}
//}
