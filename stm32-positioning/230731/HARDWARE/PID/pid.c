#include "pid.h"


//增量式PID
void PID_Incremental(PID_TypeDef *Data)
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


//位置式PID
void PID_Positional(PID_TypeDef *Data)
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



