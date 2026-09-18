#ifndef __KALANMAN_H
#define __KALANMAN_H
#include "sys.h"
extern int expect_angle;
extern short int temperature;          //陀螺仪温度数据
extern float Accel_x;              //X轴加速度值暂存
extern float Accel_y;              //Y轴加速度值暂存
extern float Accel_z;              //Z轴加速度值暂存
extern float Gyro_x;               //X轴陀螺仪数据暂存
extern float Gyro_y;               //Y轴陀螺仪数据暂存
extern float Gyro_z;               //Z轴陀螺仪数据暂存
extern float Angle_x_temp;         //由加速度计算的x倾斜角度
extern float Angle_y_temp;         //由加速度计算的y倾斜角度
extern float Angle_z_temp;         //由加速度计算的y倾斜角度
extern float Angle_X_Final;        //X最终倾斜角度
extern float Angle_Y_Final;        //Y最终倾斜角度
extern float Angle_Z_Final;        //Y最终倾斜角度
extern float bias;
extern float Gyro_Z_Drift;     //陀螺仪Z轴零飘值
 extern short	AccX;	//加速度  /32768*16g
extern short	AccY;
extern short	AccZ; 
extern short	WX;		//角速度
extern short	WY;
extern short	WZ;
extern short	Roll;	//角度
extern short	Pitch;
extern short	Yaw;
extern short	HX;		//磁场
extern short	HY;
extern short	HZ;
float Kalman_Filter_x(float Accel,float Gyro);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_y(float angle_m, float gyro_m);
void Angle_Calcu(void);
float Complementary_Filter_x(void);
#endif


