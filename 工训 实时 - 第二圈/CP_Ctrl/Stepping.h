#ifndef _STEPPING_H_
#define _STEPPING_H_

#include "sys.h"
#include "def.h"

#define TIM_FREQ            72000000U                       /* 定时器主频 */
#define MAX_STEP_ANGLE      0.05625                         /* 最小步距(1.8/MICRO_STEP) */
#define PAI                 3.1415926                       /* 圆周率*/
#define FSPR                200                             /* 步进电机单圈步数 */
#define MICRO_STEP          32                              /* 步进电机驱动器细分数 */
#define T1_FREQ             (TIM_FREQ/36)                   /* 频率ft值 2MHH,与定时器频率一致*/
#define SPR                 (FSPR*MICRO_STEP)               /* 旋转一圈需要的脉冲数 */

/* 数学常数 */

#define ALPHA               ((float)(2*PAI/SPR))            /* α = 2*pi/spr */
#define A_T_x10             ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148         ((float)((T1_FREQ*0.69)/10))    /* 0.69为误差修正值 */
#define A_SQ                ((float)(2*100000*ALPHA))
#define A_x200              ((float)(200*ALPHA))            /* 2*10*10*a/10 */
	
#define Wheel_Circum 		(60*PAI)						/*前后每步距离(mm)*/
#define Wheel_Yaw 			49.05						/*原地转每圈角度(0.1度)*/
#define Wheel_Left_Right 	0.70710678						/*横移运动分解系数*/

typedef struct
{
    __IO uint8_t  run_state;                                /* 电机旋转状态 */
    __IO uint8_t  dir;                                      /* 电机旋转方向 */
    __IO int32_t  step_delay;                               /* 下个脉冲周期（时间间隔），启动时为加速度 */
    __IO uint32_t decel_start;                              /* 开始减速位置 */
    __IO int32_t  decel_val;                                /* 减速阶段步数 */
    __IO int32_t  min_delay;                                /* 速度最快，计数值最小的值(最大速度，即匀速段速度) */
    __IO int32_t  accel_count;                              /* 加减速阶段计数值 */
} speedRampData;

enum STA
{
    STOP = 0,                                               /* 加减速曲线状态：停止*/
    ACCEL,                                                  /* 加减速曲线状态：加速阶段*/
    DECEL,                                                  /* 加减速曲线状态：减速阶段*/
    RUN                                                     /* 加减速曲线状态：匀速阶段*/
};

enum DIR
{
 CCW = 0,                                                   /* 逆时针 */ 
 CW                                                         /* 顺时针 */
};

enum EN
{
 EN_OFF = 0,                                                 /* 失能电机引脚 */
 EN_ON                                                     	 /* 使能电机引脚  */
};

enum state
{
 Vehicle_Wheel = 0,                                                 /* 车轮 */
 Robot_Arm                                                     	 /* 机械臂  */
};

/******************************************************************************************/
/* 步进电机引脚定义*/
#define STEPPER_MOTOR_1       1                             /* 步进电机接口序号 */
#define STEPPER_MOTOR_2       2
#define STEPPER_MOTOR_3       3
#define STEPPER_MOTOR_4       4

//#define Perime_Dia 				5.8*M_PI									//车轮周长(cm)
#define Num_Comp 				32											//细分数
#define Stepz_Length			360/(Num_Comp*200)								//每步角度
//#define Per_Ste					(Perime_Dia/Stepz_Length)

//extern TIM_OCInitTypeDef  TIM3_OCInitStructure;
//extern Vehicle_Wheel Whee1,Whee2,Whee3;

void Enter_counter_value(u16 last_delay, u16 delay);
u16 Gets_counter_value(void);
void Foreward_DIR(u8 Num);
void Reversal_DIR(u8 Num);
void Stepping_Motor(int angle);
void stepper_star(uint8_t motor_num);
void stepper_stop(uint8_t motor_num);
void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed); /* 梯形加减速控制函数 */
void interrupt(void);
void dir_out(int motor_A,int motor_B,int motor_C,int motor_D);
void Enabled_Disenabled(int motor_A,int motor_B,int motor_C,int motor_D);
void back(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void go(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void right(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void left(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void turn_left(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void turn_right(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
#endif

