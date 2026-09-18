#include "pid.h"

//PID控制结构体变量
PID_TypeDef Yaw_Spi_Control,Yaw_Str_Control,location1_Control,location2_Control,Speed2_Control,Speed3_Control;

//PID参数初始化
void PID_init(void)
{
	Yaw_Str_Control.P_set = 50;//100
	Yaw_Str_Control.I_set = 0;//0.1
	Yaw_Str_Control.D_set = 300;
	Yaw_Str_Control.I_max = 100.0;
	Yaw_Str_Control.Out_max = 3000.0;
	
	Yaw_Spi_Control.P_set = 45;//100
	Yaw_Spi_Control.I_set = 0;//0.1
	Yaw_Spi_Control.D_set = 100;
	Yaw_Spi_Control.I_max = 0.0;
	Yaw_Spi_Control.Out_max = 2500.0;
	
	location1_Control.P_set = 0.1;
	location1_Control.I_set = 0;
	location1_Control.D_set = 0;
	location1_Control.I_max = 8.0;
	location1_Control.Out_max = 20.0;
	
	location2_Control.P_set = 0.1;
	location2_Control.I_set = 0;
	location2_Control.D_set = 0;
	location2_Control.I_max = 8.0;
	location2_Control.Out_max = 20.0;
	
	Speed2_Control.P_set = 35;
	Speed2_Control.I_set = 20;//20
	Speed2_Control.D_set = 0;
	Speed2_Control.I_max = 2000.0;
	Speed2_Control.Out_max = 5000.0;
	
	Speed3_Control.P_set = 35;
	Speed3_Control.I_set = 20;//20
	Speed3_Control.D_set = 0;
	Speed3_Control.I_max = 2000.0;
	Speed3_Control.Out_max = 5000.0;
}

//位置式PID
void PID_Positional(PID_TypeDef *Data)
{
	Data->P_sum = Data->P_set * Data->err1;
	Data->I_sum += Data->I_set * Data->err1;
	if		(Data->I_sum >= Data->I_max) 	Data->I_sum = Data->I_max;
	else if	(Data->I_sum <= -Data->I_max) Data->I_sum = -Data->I_max;
	Data->D_sum = Data->D_set * (Data->err1 - Data->err2);
	
	Data->Out = Data->P_sum + Data->I_sum + Data->D_sum;
	if		(Data->Out >= Data->Out_max) 	Data->Out = Data->Out_max;
	else if	(Data->Out <= -Data->Out_max) Data->Out = -Data->Out_max;
	
	Data->err2 = Data->err1;
}


//增量式PID
void PID_Incremental(PID_TypeDef *Data)
{
	Data->P_sum = Data->P_set * (Data->err1 - Data->err2);
	Data->I_sum = Data->I_set * Data->err1;
	Data->D_sum = Data->D_set * (Data->err1 - 2 * Data->err2 + Data->err3);
	
	Data->Out = Data->P_sum + Data->I_sum + Data->D_sum;
	if		(Data->Out >= Data->Out_max) 	Data->Out = Data->Out_max;
	else if	(Data->Out <= -Data->Out_max) Data->Out = -Data->Out_max;
	
	Data->err3 = Data->err2;
	Data->err2 = Data->err1;
}


