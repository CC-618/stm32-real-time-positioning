#include "Seria_Screen.h"

//字符串发送函数
void HMISends(char *buf1)		  
{
	u8 i=0;
	while(1)
	{
		if(buf1[i] != 0)
	 	{
			USART_SendData(USART2,buf1[i]);  //发送一个字节
			while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET){};//等待发送结束
		 	i++;
		}
		else
		{
			return ;
		}
	}
}

//字节发送函数
void HMISendb(u8 k)		         
{		 
	u8 i;
	 for(i=0; i<3; i++)
	 {
			if(k != 0)
			{
				USART_SendData(USART2,k);  //发送一个字节
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET){};//等待发送结束
			}
			else
			{
				return ;
			}
	 } 
} 

void HMISendstart(void)
	{
	 	delay_ms(200);
		HMISendb(0xff);
		delay_ms(200);
	}
	
	
void HMT(int16_t number,int16_t number_1)
{
unsigned char buf[32];//char src[50]
//	switch(count)
//	{
//     case 3://1
//实时显示
//      HMISends("page0.t35.txt=\"Al\"");
//	 HMIsendb(0xff);
     sprintf((char *)buf,"page0.t2.txt=\"%d\"",number);
	 HMISends((char *)buf);
	 HMISendb(0xff);
	
	 sprintf((char *)buf,"page0.t3.txt=\"%d\"",number_1);
	 HMISends((char *)buf);
	 HMISendb(0xff);
}
