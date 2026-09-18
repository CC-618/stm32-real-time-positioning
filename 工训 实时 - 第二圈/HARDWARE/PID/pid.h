#ifndef _PID_H_
#define _PID_H_

#include "sys.h"
#include "def.h"

extern PID_TypeDef Yaw_Spi_Control,Yaw_Str_Control,location1_Control,location2_Control,Speed2_Control,Speed3_Control;

//PID初始化
void PID_init(void);

//增量式PID
void PID_Positional(PID_TypeDef *Data);

//位置式PID
void PID_Incremental(PID_TypeDef *Data);





#endif










