#include "pid.h"
#include "pwm.h"
#include "output.h"
#include "WP_Ctrl.h"
#include "Encoder.h"
#include "math.h"
#include "uart.h"
#include "Stepping.h"
extern u8 RX2_BUF[USART2_MAX_RX_LEN];
extern u8 uart_tx2[];
extern u16 Order1,Order2;
u8 Sensor1_flag=1;
u8 Sensor2_flag=1;
u8 Sensor3_flag=1;
u8 Start_flag;
u8 Error_flag;

u8 Angle_Pattern;
u8 x_axis_Pattern;
u8 y_axis_Pattern;
u8 xy_axis_Pattern;
u8 right_left_Pattern;
u8 right_left_Angle;

u8 Incident_flag=1;
u8 Last_Incident_flag=1;
u8 Str_Spi_flag;

u32 TIM2_Controler;
//u16 Sensor_Controler;

u32 Last_TIM2_Controler;
u32 Wheel_Step;

float Angle_er;//角度误差
float loca_Angle_err;//位移角度误差
float Angle_Immob;//车头朝向固定角度值
float PWM1_out;			//1号轮输出
float PWM2_out;			//2号轮输出
float PWM3_out;			//3号轮输出
float disp1_err;	//1号轮位移误差
float disp2_err;	//2号轮位移误差
float disp3_err;	//3号轮位移误差
float Speed1_out;	//1号轮速度输出
float Speed2_out;	//2号轮速度输出
float Speed3_out;	//3号轮速度输出
float Angle1_out;	//1号轮角度输出
float Angle2_out;	//2号轮角度输出
float Angle3_out;	//3号轮角度输出
Carrier Isotopic;
//Displacement_err.x,Displacement_err.y

//任务序号，       期望角度,x轴期望,,y轴期望			,角度输出控制，x轴输出控制，y轴输出控制
float Pos_x_y[][7]={     0,        0,0,0,		  0,0,0,
					     1,        0,0,110,      0,0,1,
					     2,        0,567,0,  	  1,1,0,
					     3,        0,1500,0,	  1,1,0,
					     4,        0,0,150,	      1,0,1,
					     5,        0,0,194,		  1,0,1,
					     6,        0,1140,0,	  1,1,0,
	
					     7,        90,0,1020,	  1,1,0,
						 8,		   90,0,0,	      1,0,0,
					     9,        90,0,1870,	  1,1,0,

//					     10,        178,0,0,	   1,0,0,
//					     11,        178,1120,1980, 1,1,1,
//					     12,       178,0,1960,	  1,0,1,
//					     13,       178,1930,0,	  	1,1,0,
	
					     10,        180,0,0,	   1,0,0,
					     11,        180,1140,1980, 1,1,1,
					     12,       180,0,1960,	  1,0,1,
					     13,       180,1930,0,	  	1,1,0,

					     14,       90,0,0,	      1,0,0,
					     15,       90,0,1070,	  1,1,0,
					     16,       90,2090,0,	  1,0,1,
					     17,       90,2010,0,	  1,0,1,
					     18,       90,0,200,	 	 1,1,0,
	
					     19,       0,0,0,	  	  1,0,0,
					     20,       0,1580,0,	  1,1,0,		
					     21,        0,0,140,	      1,0,1,
					     22,        0,0,170,		  1,0,1,
					     23,        0,1180,0,	  1,1,0,
						 
					     24,        90,0,1020,	  1,1,0,
						 25,		 90,0,0,	      1,0,0,
					     26,        90,0,1900,	  1,1,0,
						 
					     26,        180,0,0,	      1,0,0,
					     27,        180,1200,1970, 1,1,1,
					     28,       180,0,1950,	  1,0,1,
					     29,       180,1930,0,	  1,1,0,
						 
					     30,       90,0,950,	  1,1,0,
					     31,       90,2050,0,	  1,0,1,
					     32,       90,1950,0,	  1,0,1,
					     33,       90,0,100,	  1,1,0,
						 
					     34,       2.4,0,0,	  	  1,0,0,
					     35,       2.4,110,0,	  1,1,0,
					     36,       2.4,0,-160,	  1,0,1,						 
					};//任务序号，期望角度，x轴位移期望(cm)，y轴位移期望(cm)
float flag[]={0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
float Uart2_flag[]={0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
float out_flag[]={0,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};	
/*
 * @brief       误差计算
 * @param       Angle_flag：角度模式标志位，0为锁头，期望角度由坐标与陀螺仪给定;1为车头朝向固定，期望角度Angle_Immob需人工赋值
 * @retval      无
 */
void Error_calcula(void)
{	
	/*取期望*/
	Angle_Immob=Pos_x_y[Incident_flag][1];//头固定角度
	Displacement_expect.x=Pos_x_y[Incident_flag][2];//期望x
	Displacement_expect.y=Pos_x_y[Incident_flag][3];//期望y
	
	/*角度，位移计算*/
	Displacement_err.x=Displacement_expect.x-Displacement.x;//x轴位移误差
	Displacement_err.y=Displacement_expect.y-Displacement.y;//y轴位移误差
	
//	Coord_expect=sqrt(pow(Displacement_expect.x, 2)+pow(Displacement_expect.y,2));//计算期望直线位移误差
	Coord_err=sqrt(pow(Displacement_err.x, 2)+pow(Displacement_err.y,2));//计算直线位移误差

	Angle_er=Angle_Immob-Angle_z_temp;//车头朝向固定角度值-陀螺仪角度(角度)
	loca_Angle_err=atan2(Displacement_err.y,Displacement_err.x)-Angle_z_temp*M_PI/180;//计算位移夹角(弧度)
	
	Isotopic_err.x=Coord_err*cos(loca_Angle_err);//载体坐标系x轴误差
	Isotopic_err.y=Coord_err*sin(loca_Angle_err);//载体坐标系y轴误差
	//优化Angle_err
	if(Angle_er>180)
	{
		JY_Yaw=Angle_er-360;
	}
	else if(Angle_er<-180)
	{
		JY_Yaw=Angle_er+180;
	}
	else
	{
		JY_Yaw=Angle_er;
	}
//	Isotopic.JY_YAW=JY_Yaw;
//	Isotopic.ZX=Displacement_err.x;
//	Isotopic.ZY=Displacement_err.y;
	
	Isotopic.JY_YAW=JY_Yaw;
	Isotopic.ZX=Isotopic_err.x;
	Isotopic.ZY=Isotopic_err.y;
	//优化loca_Angle_err
//	if(loca_Angle_err>180)
//	{
//		loca_JY_Yaw=loca_Angle_err-360;
//	}
//	else if(loca_Angle_err<-180)
//	{
//		loca_JY_Yaw=loca_Angle_err+180;
//	}
//	else
//	{
//		loca_JY_Yaw=loca_Angle_err;
//	}
}

void Incident1(float err1 , float err2 , float err3)
{
	/*事件判断*/
	if(fabs(JY_Yaw)<err1 && fabs(Displacement_err.x)<err2 && fabs(Displacement_err.y)<err3)
	{
		Str_Spi_flag=0;
		Error_flag=0;
	}
	else
	{
		Str_Spi_flag=1;
	}
	if(g_motion_sta == STOP && Str_Spi_flag == 0)
	{
		Incident_flag++;
		TIM2_Controler=0;
		Angle_Pattern=0;//角度模式归0
		x_axis_Pattern=0;//X轴模式归0
		y_axis_Pattern=0;//Y轴模式归0
	}
	if(Angle_Pattern==0 &&x_axis_Pattern==0 && y_axis_Pattern==0 && Str_Spi_flag == 1)//可确保一轮回刷新一次
	{
		Error_flag = 1;
	}
	/*运动分解*/
	if(Error_flag == 1)
	{
		//角度
		if(JY_Yaw>=0)
		{
			Angle_Pattern=1;
		}
		else
		{
			Angle_Pattern=2;
		}
		//x轴
		if(Displacement_err.x>=0)
		{
			x_axis_Pattern=1;
		}
		else
		{
			x_axis_Pattern=2;
		}
		//y轴
		if(Displacement_err.y>=0)
		{
			y_axis_Pattern=1;
		}
		else
		{
			y_axis_Pattern=2;
		}
	}
	else
	{
		Angle_Pattern=0;
		x_axis_Pattern=0;
		y_axis_Pattern=0;
	}
}


/**
  * 函    数：步数计算，输出
  * 参    数：无
  * 返 回 值：无
  * 注意事项：无
  */
void Control_Output1(float err1 , float err2 , float err3)
{
	//角度
	if(Angle_Pattern!=0 && g_motion_sta == STOP)
	{
		if(fabs(Angle_er)<err1)
		{
			Angle_Pattern=0;
		}
		if(Angle_Pattern != 0)
		{
			Wheel_Step=Angle_er/Wheel_Circum*SPR;//误差/单圈周长*1圈步数		
			switch(Angle_Pattern)
			{
				case 1:
						turn_left(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
					break;
				case 2:
						turn_right(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
					break;
				default:
					break;
			}			
		}
	}
	
	//x轴
	if(Angle_Pattern == 0 && x_axis_Pattern!=0 && g_motion_sta == STOP)
	{
		if(fabs(Displacement_err.x)<err1)
		{
			x_axis_Pattern=0;
		}
		if(x_axis_Pattern != 0)
		{		
			Wheel_Step=Displacement_err.x/Wheel_Circum*SPR;//误差/单圈周长*1圈步数		
			switch(x_axis_Pattern)
			{
				case 1:
						go(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
					break;
				case 2:
						back(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
					break;
				default:
					break;
			}
		}
	}
	//y轴
	if(Angle_Pattern == 0 && x_axis_Pattern ==0 && x_axis_Pattern !=0 && g_motion_sta == STOP)
	{
		if(fabs(Displacement_err.x)<err1)
		{
			y_axis_Pattern=0;
		}
		if(y_axis_Pattern != 0)
		{		
			Wheel_Step=Displacement_err.y/Wheel_Circum*SPR;//误差/单圈周长*1圈步数		
			switch(y_axis_Pattern)
			{
				case 1:
						left(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
					break;
				case 2:
						right(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
					break;
				default:
					break;
			}
		}
	}	
}

//事件判断			
void Incident2(void)
{
	if(flag[Incident_flag]==1 && g_motion_sta==STOP)
	{
		Angle_Pattern=0;
		xy_axis_Pattern=0;
		right_left_Pattern=0;
		flag[Incident_flag]=0;
	}	
	if(Angle_Pattern==1 && xy_axis_Pattern==1 && right_left_Pattern==1 && flag[Incident_flag] == 0 && g_motion_sta==STOP)
	{
		flag[Incident_flag]=-1;
		Incident_flag++;
		TIM2_Controler=0;

	}
}

/**
  * 函    数：步数计算，输出
  * 参    数：无
  * 返 回 值：无
  * 注意事项：无
  */
void Control_Output2(float Angl_Err,float Pattern,float right_left_Err)
{
	/*事件判断*/
	Incident2();
	
	/*角度*/
	if(Angle_Pattern==0 && g_motion_sta==STOP)
	{
		if(Pos_x_y[Incident_flag][4] == 1 && g_motion_sta==STOP)
		{
			Wheel_Step=Angl_Err/Wheel_Yaw*SPR;//角度误差/每圈角度*1圈步数
			if(out_flag[Incident_flag]==3)
			{
				out_flag[Incident_flag]=2;
				if(Angl_Err>0 && g_motion_sta==STOP)
				{
					turn_left(Wheel_Step, LR_step_accel, LR_step_decel, LR_set_speed);
				}
				else if(Angl_Err<0 && g_motion_sta==STOP)
				{
					turn_right(Wheel_Step, LR_step_accel, LR_step_decel, LR_set_speed);
				}	
				Wheel_Step=0;
			}
			if(g_motion_sta==STOP && out_flag[Incident_flag]==2)
			{
				Angle_Pattern=1;
			}
		}
		if(Pos_x_y[Incident_flag][4] == 0 && g_motion_sta==STOP)
		{
			out_flag[Incident_flag]=2;	
			Angle_Pattern=1;
		}	
	}
	
	/*x轴*/
	if(Angle_Pattern==1 && xy_axis_Pattern==0 && g_motion_sta==STOP)
	{
		if(Pos_x_y[Incident_flag][5] == 1)
		{
			Wheel_Step=Pattern/Wheel_Circum*SPR;//误差/单圈周长*1圈步数
			if(out_flag[Incident_flag]==2)
			{
				out_flag[Incident_flag]=1;				
				if(Pattern>0 && g_motion_sta==STOP)
				{
					
					go(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
				}
				else if(Pattern<0 && g_motion_sta==STOP)
				{
					back(Wheel_Step, g_step_accel, g_step_decel, g_set_speed);
				}
				Wheel_Step=0;
			}
			if(g_motion_sta==STOP && out_flag[Incident_flag]==1)
			{
				xy_axis_Pattern=1;
			}
		}
		if(Pos_x_y[Incident_flag][5] == 0 && g_motion_sta==STOP)
		{
			out_flag[Incident_flag]=1;	
			xy_axis_Pattern=1;
		}	
	}

	/*y轴*/	
	if(Angle_Pattern==1 && xy_axis_Pattern==1 && right_left_Pattern==0 && g_motion_sta==STOP)
	{
		if(Pos_x_y[Incident_flag][6] == 1)
		{
			Wheel_Step=right_left_Err/Wheel_Circum*SPR;//误差/单圈周长*1圈步数
			if(out_flag[Incident_flag]==1)
			{
				out_flag[Incident_flag]=0;				
				if(right_left_Err>0 && g_motion_sta==STOP)
				{
					left(Wheel_Step, LR_step_accel, LR_step_decel, LR_set_speed);
				}
				else if(right_left_Err<0 && g_motion_sta==STOP)
				{
					right(Wheel_Step, LR_step_accel, LR_step_decel, LR_set_speed);
				}
				Wheel_Step=0;
			}
			if(g_motion_sta==STOP && out_flag[Incident_flag]==0)
			{
				right_left_Pattern=1;
			}
		}
		if(Pos_x_y[Incident_flag][6] == 0 && g_motion_sta==STOP)
		{
			out_flag[Incident_flag]=0;	
			right_left_Pattern=1;
		}		
	}	

}



/**
  * 函    数：控制器模式选择函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：无
  */					
void Controler_Mode_Select(void)
{
	if(Incident_flag <= 2)
	{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);		
	}
	else if(Incident_flag == 3)
	{
		if(Order_Ctrl==1)//扫完二维码才走
		{
			/*角度，位移计算*/
			Error_calcula();
			/*输出*/
			Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);			
		}	
	}
	else if(Incident_flag == 4)
	{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);		
	}	
	else if(Incident_flag == 5)
	{
		if(Sensor1_flag ==4 )//抓完物料才走
		{
			/*角度，位移计算*/
			Error_calcula();
			/*输出*/
			Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);				
		}	
	}
	else if(Incident_flag >= 6 && Incident_flag <= 11)
	{		
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);		
	}
	else if(Incident_flag == 12)
	{	
		if(Co_Trim[Incident_flag]==6)
		{
			/*角度，位移计算*/
			Error_calcula();
			/*输出*/
			Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);	
		}
	}
	else if(Incident_flag >= 13 && Incident_flag <= 16)
	{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);	
	}
	else if(Incident_flag == 17)
	{
		if(Co_Trim[Incident_flag]==3)
		{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);			
		}
	
	}
	else if(Incident_flag >= 18 && Incident_flag <= 21)
	{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);	
	}
	else if(Incident_flag == 22)
	{
		if(Sensor2_flag==4)
		{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);			
		}	
	}
	else if(Incident_flag >= 23 && Incident_flag <= 36)
	{
		/*角度，位移计算*/
		Error_calcula();
		/*输出*/
		Control_Output2(JY_Yaw,Isotopic.ZX,Isotopic.ZY);	
	}	
}



/**
 * @brief   总控制器
 * @param   None
 * @return  None
 */
void Motor_Control(void)
{ 
	if(Start_flag == 1)
	{
		TIM2_Controler++;//任务调度间隔
		Controler_Mode_Select();//控制器模式选择
	}
}
