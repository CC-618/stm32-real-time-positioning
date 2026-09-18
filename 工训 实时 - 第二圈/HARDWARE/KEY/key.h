#ifndef _KEY_H
#define _KEY_H	
#include "sys.h"
#include "def.h"

typedef union
{
	u8 Num;
	struct
	{
		u8 Bit0:1;
		u8 Bit1:1;
		u8 Bit2:1;
		u8 Bit3:1;
		u8 Bit4:1;
		u8 Bit5:1;
		u8 Bit6:1;
		u8 Bit7:1;
	}BITS;
}DipSwitchDef;


#define Key1	PAin(0)
#define Key2	PAin(1)



extern TIM_OCInitTypeDef  TIM3_OCInitStructure;
extern float Distance1;//编码器1的路程
extern float Distance2;//编码器2的路程
extern Vector2f Displacement;
extern u16 Sensor_Controler;

void Key_Init(void);
void DipSwitch_Init(void);
u8 DipSwitch_Read(void);
void Key_Scan(void);
void Key_execute(void);
void Key_Control(void);

#endif



