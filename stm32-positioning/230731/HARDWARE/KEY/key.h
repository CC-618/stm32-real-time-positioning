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


void Key_Init(void);
void DipSwitch_Init(void);
u8 DipSwitch_Read(void);

#endif



