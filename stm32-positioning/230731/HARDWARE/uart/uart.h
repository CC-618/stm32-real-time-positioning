#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
//#include "main.h"


#define		USART1_MAX_TX_LEN		50			     	//最大发送缓存字节数
#define		USART1_MAX_RX_LEN		50			     	//最大接收缓存字节数
extern u8	USART1_TX_FLAG;
extern u8	USART1_RX_FLAG;

#define		USART2_MAX_TX_LEN		50			     	//最大发送缓存字节数
#define		USART2_MAX_RX_LEN		50			     	//最大接收缓存字节数
extern u8	USART2_TX_FLAG;
extern u8	USART2_RX_FLAG;

#define		USART3_MAX_TX_LEN		50			     	//最大发送缓存字节数
#define		USART3_MAX_RX_LEN		50			     	//最大接收缓存字节数
extern u8	USART3_TX_FLAG;
extern u8	USART3_RX_FLAG;



void UART1_init(u32 BaudRate);
void UART2_init(u32 BaudRate);
void UART3_init(u32 BaudRate);

void DMA1_USART1_Init(void);
void DMA1_USART2_Init(void);
void DMA1_USART3_Init(void);

char DMA1_USART1_TxData(u8 *buffer, u32 size);
char DMA1_USART2_TxData(u8 *buffer, u32 size);
char DMA1_USART3_TxData(u8 *buffer, u32 size);

//void sent_data1(float A,float B,float C);	

#endif


