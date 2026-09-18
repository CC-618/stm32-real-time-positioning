#include "Encoder.h"

int32_t Encoder1_Count;					//用于计数旋转编码器的脉冲值
int32_t Encoder2_Count;					//用于计数旋转编码器的脉冲值
int32_t Last_Encoder1_Count;			//用于计数旋转编码器的上一次增量值
int32_t Last_Encoder2_Count;			//用于计数旋转编码器的上一次增量值
float Diff_Distance1;					//编码器1的微分路程
float Diff_Distance2;					//编码器2的微分路程
float Dis_Journey;						//单位时间内的位移	
Vector2f Displacement,Displacement_expect,Displacement_err,MV_Displacement,Isotopic_err,Colour_Displacement;					//坐标
float Coord_err;						//实时位移误差
float Coord_expect;						//期望误差

float TCoord_Radian;					//单位时间内的位移与X轴夹角弧度	
float Diff_Distance[][2];				//Diff_Distance[][0]:1号编码器位移，Diff_Distance[][0]:2号编码器位移
float Special_Sec;						//特殊区间
/**
  * 函    数：旋转编码器初始化
  * 参    数：无
  * 返 回 值：无
  */
int Encoder_L_1_EXTI;
int Encoder_R_1_EXTI;
int Encoder_L,Encoder_R,encoder_total;
void Encoder_EXTIX_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB,&GPIO_InitStruct);	
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
	

	
	EXTI_InitStruct.EXTI_Line = EXTI_Line4 | EXTI_Line6;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;//跳变沿触发
	EXTI_Init(&EXTI_InitStruct);
	
	

	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
 
}
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line4);  
		if(PBin(3)==0)   
		{
			Encoder_L_1_EXTI--;
		}
		else                  //上升沿
		{ 
			Encoder_L_1_EXTI++; 
		}
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);  
		if(PBin(5)==0)   
		{
			Encoder_R_1_EXTI--; 
			
		}
		else                  //上升沿
		{ 
			Encoder_R_1_EXTI++;
		}
	}
}

int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
	{
		case 1:Encoder_TIM=Encoder_R_1_EXTI;  break;
		case 2:Encoder_TIM=Encoder_L_1_EXTI;  break;
	    default:
			break;
	}
		return Encoder_TIM;
}


/**
  * 函    数：旋转编码器获取增量值
  * 参    数：Multiple  :返回数据倍数
			  Pulse		:编码器单圈脉冲数
  * 返 回 值：无
  * 注意事项：无
  */
void Encoder1_Get(float Multiple,u16 Pulse)
{
	int16_t Temp;
	Encoder1_Count=Read_Encoder(1);
	Temp = Encoder1_Count-Last_Encoder1_Count;
	Last_Encoder1_Count=Encoder1_Count;
	Diff_Distance1 = Multiple*(Diameter1*M_PI*Temp)/Pulse*1;//单位转化倍数*(周长*单位周期脉冲数)/单圈周期脉冲数*误差系数+误差角度
															 //编码器1,1个周期内的路程
	Diff_Distance[0][0] +=Diff_Distance1;
}

/**
  * 函    数：旋转编码器获取增量值
  * 参    数：Multiple  :返回数据倍数
			  Pulse		:编码器单圈脉冲数
  * 返 回 值：无
  * 注意事项：无
  */
void Encoder2_Get(float Multiple,u16 Pulse)
{
	int16_t Temp;
	Encoder2_Count=Read_Encoder(2);
	Temp = Encoder2_Count-Last_Encoder2_Count;
	Last_Encoder2_Count=Encoder2_Count;
	Diff_Distance2 = Multiple*(Diameter2*M_PI*Temp)/Pulse*1;//单位转化倍数*(周长*单位周期脉冲数)/单圈周期脉冲数*误差系数
															 //编码器2,1个周期内的路程
	Diff_Distance[0][1] +=Diff_Distance2;
}

/**
  * 函    数：两个编码器数据融合函数
  * 参    数：无
  * 返 回 值：无
* 注意事项：主要任务为计算坐标,编码器坐标系应与车体坐标系重合，不然会有误差
               /\
             X  |
                |
                |
     Y          |
	<—————车头向前
  */
void Velocity_Fusion(void)
{
	Encoder1_Get(1,1024);//更新编码器1路程	
	Encoder2_Get(1,1024);//更新编码器2路程	
	Dis_Journey=sqrt(pow(Diff_Distance1, 2)+pow(Diff_Distance2, 2));//编码器合路程
	TCoord_Radian=Angle_z_temp*M_PI/180-M_PI/4+atan2(Diff_Distance1,Diff_Distance2);//计算单位时间内的位移与坐标轴夹角弧度
	Displacement.x+=Dis_Journey*cos(TCoord_Radian);//x坐标
	Displacement.y+=Dis_Journey*sin(TCoord_Radian);//y坐标
	Special_Sec=fabs(sqrt(pow(Displacement.x, 2)+pow(Displacement.y, 2)));//取圆形特殊区域
}
