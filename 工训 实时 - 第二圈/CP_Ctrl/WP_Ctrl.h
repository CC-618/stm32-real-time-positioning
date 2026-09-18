#ifndef _WP_CTRL_H_
#define _WP_CTRL_H_

#include "sys.h"
#include "def.h"

#define Loop_Distance1 			150			
#define Loop_Distance2 			300

extern u8 Sensor1_flag;
extern u8 Sensor2_flag;
extern u8 Sensor3_flag;

typedef struct
{
    __IO float  JY_YAW;                                /* Ω«∂» */
    __IO float  ZX;                                      /* x÷· */
    __IO float  ZY;                               /* y÷· */
 
} Carrier;


void Error_calcula(void);
void Incident1(float err1 , float err2 , float err3);
void Incident2(void);
void Controler_Mode_Select(void);
void Motor_Control(void);
void Str_Set_Pwm(float one,float two,float three);
void Control_Output1(float err1 , float err2 , float err3);
void Control_Output2(float Angl_Err,float Pattern,float right_left_Err);

void Points_Cleared(void);

#endif

