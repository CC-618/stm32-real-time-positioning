#ifndef _DEF_H_
#define _DEF_H_

#include "sys.h"


#define  M_PI   				3.14159265358979323846264338327950
#define sin30					0.5
#define cos30					0.86602540378443864676372317075294

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
//typedef union
//{
//	u8		array[44];
//	struct
//	{
//		u8 	head1;
//		u8 	type1;
//		short	AccX;
//		short	AccY;
//		short	AccZ;
//		short	Temp1;
//		u8	crc1;
//		
//		u8 	head2;
//		u8 	type2;
//		short	WX;
//		short	WY;
//		short	WZ;
//		short	Vol;
//		u8	crc2;
//		
//		u8 	head3;
//		u8 	type3;
//		short	Roll;
//		short	Pitch;
//		short	Yaw;
//		short	V;
//		u8	crc3;
//		u8 	head4;
//		u8 	type4;
//		short	HX;
//		short	HY;
//		short	HZ;
//		short	temp2;
//		u8	crc4;
//	}byte;
//}JY931_TypeDef;



typedef union
{
	u8		array[12];
	struct
	{
		long 	X;
		long 	Y;
		long	Angle;
	}byte;
}JY931_TypeDef;




#endif



