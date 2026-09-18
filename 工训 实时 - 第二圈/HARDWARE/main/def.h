#ifndef _DEF_H_
#define _DEF_H_

#include "sys.h"

#define  M_PI   				3.14159265358979323846264338327950
#define sin30					0.5
#define cos30					0.86602540378443864676372317075294
/*前进，后退，左转，右转*/
extern __IO uint32_t g_set_speed;          /* 最大速度 单位为0.1rad/sec */
extern __IO uint32_t g_step_accel;            /* 加速度 单位为0.1rad/sec^2 */
extern __IO uint32_t g_step_decel;            /* 减速度 单位为0.1rad/sec^2 */
extern __IO uint16_t g_step_angle;             /* 设置的步数*/

extern __IO uint32_t g_add_pulse_count;     /* 脉冲个数累计*/
extern __IO uint8_t  g_motion_sta;                    /* 是否在运动？0：停止，1：运动 */

/*左移，右移*/
extern __IO uint32_t LR_set_speed;          /* 最大速度 单位为0.1rad/sec */
extern __IO uint32_t LR_step_accel;            /* 加速度 单位为0.1rad/sec^2 */
extern __IO uint32_t LR_step_decel;            /* 减速度 单位为0.1rad/sec^2 */
extern __IO uint16_t LR_step_angle;             /* 设置的步数*/

//timer.c
extern u16 CCR1_Val;
extern u16 CCR2_Val;
extern u16 CCR3_Val;
extern u16 CCR4_Val;


//标志位
extern u8 Start_flag;
extern u16 Time_NUM2;
////TIM_GPIO.c
//extern u16 Freq_Num;
//extern u32 Stepz1_Flag;//布数计算控制
//extern u32 Stepz2_Flag;//布数计算控制
//extern u32 Stepz3_Flag;//布数计算控制
//extern u32 Step1_Num;//布数1
//extern u32 Step2_Num;//布数2
//extern u32 Step3_Num;//布数3

//Encoder.c
extern float Diff_Distance1;					//编码器1的微分路程
extern float Diff_Distance2;					//编码器2的微分路程
extern float Coord_Radian;						//合位移与X轴夹角弧度
extern float Coord_err;						//实时位移误差
extern float Coord_expect;						//期望误差
extern float Diff_Distance[][2];
extern float Special_Sec;						//特殊区间

//Sensor.c
extern float JY_Yaw;
extern float loca_JY_Yaw;

//WP_Ctrl.c
extern float Angle_er;//角度误差
extern float loca_Angle_err;//位移角度误差
extern float disp1_err;	//1号轮位移误差
extern float disp2_err;	//2号轮位移误差
extern float disp3_err;	//3号轮位移误差

//OLED.c
extern u8 OLED_NUM;

//kalanman.c
extern short	AccX;	//加速度  /32768*16g
extern short	AccY;
extern short	AccZ; 
extern short	WX;		//角速度
extern short	WY;
extern short	WZ;
//extern short	Roll;	//角度
//extern short	Pitch;
//extern short	Yaw;
//extern short	HX;		//磁场
//extern short	HY;
//extern short	HZ;
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
extern float Last_Angle_z_temp;    //上次Y最终倾斜角度
extern float bias;
extern float Gyro_Z_Drift;     //陀螺仪Z轴零飘值

typedef struct
{
	float	err1;
	float	err2;
	float	err3;
	float	P_set;
	float	I_set;
	float	D_set;
	float	P_sum;
	float	I_sum;
	float	D_sum;
	float	Out;
	float	Out_last;
	float	I_max;
	float	Out_max;
}PID_TypeDef;

typedef struct
{
	float x;
	float y;
}Vector2f;

typedef struct
{
	u8 one;
	u8 two;
	u8 three;
}Order;

//typedef struct
//{
//	u16 Step;//车轮布数
//	float Freq_Expect;//车轮频率期望数
//	u16 Freq_Control;//车轮频率控制数
//	float Disp_Err;//车轮路程误差
//	u16 Control_Flag;//车轮布数刷新标志位
//}Vehicle_Wheel;

typedef union
{
	u8	array[12];
	struct
	{
		long	X;
		long	Y;
		long	Angle;
	}Byte;
}JY901_TypeDef;


#endif



