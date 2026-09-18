#ifndef _LED_H
#define _LED_H	
#include "sys.h"


#define LED1 	PCout(13)
#define LED2  	PCout(14)
#define LED3  	PCout(15)


#define LEDOn  	0
#define LEDOff	1


void LED_Init(void);

#endif



