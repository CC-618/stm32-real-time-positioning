#ifndef _BUZZER_H
#define _BUZZER_H	
#include "sys.h"


#define Buzzer 	PCout(12)


#define BuzzerOn  	1
#define BuzzerOff	0


void BUZZER_Init(void);

#endif

