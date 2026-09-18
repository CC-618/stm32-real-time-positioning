#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F103ZE核心板
//SPI驱动 代码	   
							  
//////////////////////////////////////////////////////////////////////////////////

/*
	SPI引脚初始化配置
	**PB13------SCLK-----D0
	**PB15------MOSI-----D1
	**PA12---------------RES(RST)
	**PB14------MISO-----DC
	**PB12------CS-------CS
*/

#define OLED_RES  PAout(12)
#define OLED_DC   PBout(14)
#define OLED_CS   PBout(12)


 				  	    													  
void SPI2_Init(void);			 //初始化SPI口 
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
		 
#endif

