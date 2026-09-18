#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
//#include "main.h"
#include "def.h"
#include "delay.h"
#define		x_err		1.2931			     	//编码器补偿
#define		y_err		1.25			     	//编码器补偿
#define		x_centre		80			     	//open mv x
#define		y_centre		43			     	//open mv y 
#define		dimension		50			     	//open mv y 

#define		USART1_MAX_TX_LEN		50			     	//最大发送缓存字节数
#define		USART1_MAX_RX_LEN		50			     	//最大接收缓存字节数
extern u8	USART1_TX_FLAG;
extern u8	USART1_RX_FLAG;
extern char TIM2_USART1_FLAG;							//串口定时发送标志位
#define		USART2_MAX_TX_LEN		50			     	//最大发送缓存字节数
#define		USART2_MAX_RX_LEN		50			     	//最大接收缓存字节数
extern u8	USART2_TX_FLAG;
extern u8	USART2_RX_FLAG;

#define		USART3_MAX_TX_LEN		50			     	//最大发送缓存字节数
#define		USART3_MAX_RX_LEN		50			     	//最大接收缓存字节数
extern u8	USART3_TX_FLAG;
extern u8	USART3_RX_FLAG;

extern Order Sequence1,Sequence2;
extern u8 Colour1,Colour2;
extern u8 Order_Ctrl;
extern u8 Posture_Clear;
extern Order Sequence1,Sequence2,Variant1,Variant2;
extern u8 MV_fetch;//色环，色块
extern u8 MV_fetch_one;//色环，色块
extern u8 Co_Trim[100];	
void UART1_init(u32 BaudRate);
void UART2_init(u32 BaudRate);
void UART3_init(u32 BaudRate);

void DMA1_USART1_Init(void);
void DMA1_USART2_Init(void);
void DMA1_USART3_Init(void);

char DMA1_USART1_TxData(u8 *buffer, u32 size);
char DMA1_USART2_TxData(u8 *buffer, u32 size);
char DMA1_USART3_TxData(u8 *buffer, u32 size);

void switch_circle(u16 chromatic,u16 order);
void switch_release(u16 chromatic,u16 order);
void Fine_Tuning(void);
void Fine_Control(void);
void Switch_Ctrl(void);
void sent_data1(float A,float B,float C,float D,float E,float F,float G,float H);
void USART1_Counter(void);
void USART1_TX_RX(void);
void USART3_TX_RX(void);
void TransJY931(void);
void USART2_RX_Ctrl(void);
void USART3_RX_Ctrl(void);
void Pick_Place(void);
#endif


