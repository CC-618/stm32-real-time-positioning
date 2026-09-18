#ifndef __KALANMAN_H
#define __KALANMAN_H

#include "sys.h"
#include "def.h"


float Kalman_Filter_x(float Accel,float Gyro);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_y(float angle_m, float gyro_m);
void Angle_Calcu(void);
float Complementary_Filter_x(void);
#endif
