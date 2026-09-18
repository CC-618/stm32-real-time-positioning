#ifndef _SERIA_SCREEN_H_
#define _SERIA_SCREEN_H_

#include "sys.h"
#include <string.h>
#include "uart.h"
#include "delay.h"

void HMISends(char *buf1);
void HMISendb(u8 k);
void HMISendstart(void);
void HMT(int16_t number,int16_t number_1);
#endif
