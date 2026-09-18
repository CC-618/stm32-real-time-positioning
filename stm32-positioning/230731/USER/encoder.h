#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"
#include "def.h"
#include "kalanman.h"

#define Diameter1 			3.8					//编码器1直径(cm)
#define Diameter2 			3.8					//编码器2直径(cm)
#define Angle_error			0.008673505			//编码器与机身的角度误差(弧度)


void Encoder_Init(void);
uint32_t encoder_GetFreq(void);

extern int Encoder_L_1_EXTI;
extern int Encoder_R_1_EXTI;
extern int Encoder_L,Encoder_R,encoder_total;
void Encoder_EXTIX_Init(void);
extern int mode_flag;
int Read_Encoder(u8 TIMX);

extern Vector2f Displacement,Displacement_expect,Displacement_err,Last_Displacement;	
//坐标
void Encoder1_Get(float Multiple,u16 Pulse);
void Encoder2_Get(float Multiple,u16 Pulse);
void Velocity_Fusion(void);

#endif

