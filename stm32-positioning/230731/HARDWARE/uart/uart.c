
#include <string.h>
#include "uart.h"
#include "def.h"


//UART1定义
#define UART1_USART				USART1
#define UART1_PORT     			GPIOA
#define UART1_TX_PIN			GPIO_Pin_9
#define UART1_RX_PIN			GPIO_Pin_10
#define UART1_IRQn				USART1_IRQn

//UART2定义
#define UART2_USART				USART2
#define UART2_PORT     			GPIOA
#define UART2_TX_PIN			GPIO_Pin_2
#define UART2_RX_PIN			GPIO_Pin_3
#define UART2_IRQn				USART2_IRQn

//UART3定义
#define UART3_USART				USART3
#define UART3_PORT     			GPIOB
#define UART3_TX_PIN			GPIO_Pin_10
#define UART3_RX_PIN			GPIO_Pin_11
#define UART3_IRQn				USART3_IRQn

char USART1_TX_BUF[USART1_MAX_TX_LEN]; 	//发送缓冲区
u8 USART1_Rxbuf1[USART1_MAX_RX_LEN];	//发送数据缓冲区1
u8 USART1_Rxbuf2[USART1_MAX_RX_LEN];	//发送数据缓冲区2
static u8 Mark_USART1_Rxbuf=0;			//标记当前使用的是哪个缓冲区,0：使用u1rxbuf；1：使用u2rxbuf
u8 USART1_TX_FLAG=0;					//USART1发送标志，启动发送时置1
u8 USART1_RX_FLAG;						//USART1接收标志，启动接收时置1

char USART2_TX_BUF[USART2_MAX_TX_LEN]; 	//发送缓冲区
u8 USART2_Rxbuf1[USART2_MAX_RX_LEN];	//发送数据缓冲区1
u8 USART2_Rxbuf2[USART2_MAX_RX_LEN];	//发送数据缓冲区2
static u8 Mark_USART2_Rxbuf=0;			//标记当前使用的是哪个缓冲区,0：使用u1rxbuf；1：使用u2rxbuf
u8 USART2_TX_FLAG=0;					//USART2发送标志，启动发送时置1
u8 USART2_RX_FLAG;						//USART2接收标志，启动接收时置1

char USART3_TX_BUF[USART3_MAX_TX_LEN]; 	//发送缓冲区
u8 USART3_Rxbuf1[USART3_MAX_RX_LEN];	//发送数据缓冲区1
u8 USART3_Rxbuf2[USART3_MAX_RX_LEN];	//发送数据缓冲区2
static u8 Mark_USART3_Rxbuf=0;			//标记当前使用的是哪个缓冲区,0：使用u1rxbuf；1：使用u2rxbuf
u8 USART3_TX_FLAG=0;					//USART3发送标志，启动发送时置1
u8 USART3_RX_FLAG;						//USART3接收标志，启动接收时置1


extern u8	RX1_BUF[USART1_MAX_RX_LEN];
extern u8	RX2_BUF[USART2_MAX_RX_LEN];
extern u8	RX3_BUF[USART3_MAX_RX_LEN];
//extern JY931_TypeDef Rx_buf;


//-----------------------------------------------------------------
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#pragma import(__use_no_semihosting)
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
//标准库需要的支持函数
struct __FILE
{
	int handle;
};
FILE __stdout;
//------------------------------------------------------------------

/**
 * @description:重定向c库函数printf到串口，重定向后可使用printf函数
 * @param 无
 * @return 无
 */
int fputc(int ch, FILE *f)
{
	USART1->SR;
	USART_SendData(USART2, (uint8_t)ch);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{
	}
	return (ch);
}

/**
 * @description: UART1初始化，
 * @param  None
 * @return None
 */
void UART1_init(u32 BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = UART1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART1_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//串口初始化
	USART_Init(UART1_USART, &USART_InitStructure);
	
	//中断开启设置
	USART_ITConfig(UART1_USART, USART_IT_IDLE, ENABLE);			//开启检测串口空闲状态中断
	USART_ClearFlag(UART1_USART,USART_FLAG_TC);					//清除USART1标志位
	
	//使能USART
	USART_Cmd(UART1_USART, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART1_USART, USART_IT_RXNE, ENABLE);
	
	DMA1_USART1_Init();
}

/**
 * @description: UART2初始化，
 * @param  None
 * @return None
 */
void UART2_init(u32 BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = UART2_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = UART2_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART2_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//串口初始化
	USART_Init(UART2_USART, &USART_InitStructure);
	
	//中断开启设置
	USART_ITConfig(UART2_USART, USART_IT_IDLE, ENABLE);			//开启检测串口空闲状态中断
	USART_ClearFlag(UART2_USART,USART_FLAG_TC);					//清除USART1标志位
	
	//使能USART
	USART_Cmd(UART2_USART, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART2_USART, USART_IT_RXNE, ENABLE);
	
	DMA1_USART2_Init();
}

/**
 * @description: UART3初始化，
 * @param  None
 * @return None
 */
void UART3_init(u32 BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = UART3_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = UART3_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART3_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//串口初始化
	USART_Init(UART3_USART, &USART_InitStructure);
	
	//中断开启设置
	USART_ITConfig(UART3_USART, USART_IT_IDLE, ENABLE);			//开启检测串口空闲状态中断
	USART_ClearFlag(UART3_USART,USART_FLAG_TC);					//清除USART1标志位
	
	//使能USART
	USART_Cmd(UART3_USART, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART3_USART, USART_IT_RXNE, ENABLE);
	
	DMA1_USART3_Init();
}

void DMA1_USART1_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						//使能DMA1时钟

	//DMA1_USART1_RX  USART1->RAM的数据传输
	DMA_DeInit(DMA1_Channel5);												//将DMA的通道5寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&UART1_USART->DR);				//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART1_Rxbuf1;            			//设置接收缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;								//数据传输方向，从外设读取到内存
	DMA1_Init.DMA_BufferSize = USART1_MAX_RX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输
	 
	DMA_Init(DMA1_Channel5,&DMA1_Init); 									//对DMA通道5进行初始化
	
	//DMA1_USART1_TX  RAM->USART1的数据传输
	DMA_DeInit(DMA1_Channel4);												//将DMA的通道4寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&UART1_USART->DR);				//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART1_TX_BUF;              		//设置发送缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								//数据传输方向，从内存发送到外设
	DMA1_Init.DMA_BufferSize = USART1_MAX_TX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输

	DMA_Init(DMA1_Channel4,&DMA1_Init); 									//对DMA通道4进行初始化
	
	//DMA1通道5 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器
 
	//DMA1通道4 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器

	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);							//开USART1 Rx DMA中断
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);							//开USART1 Tx DMA中断

	DMA_Cmd(DMA1_Channel5,ENABLE);           								//使DMA通道5停止工作
	DMA_Cmd(DMA1_Channel4,DISABLE);           								//使DMA通道4停止工作
	 
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);        					//开启串口DMA发送
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);        					//开启串口DMA接收
}

void DMA1_USART2_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						//使能DMA1时钟

	//DMA1_USART2_RX  USART2->RAM的数据传输
	DMA_DeInit(DMA1_Channel6);												//将DMA的通道5寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&UART2_USART->DR);				//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART2_Rxbuf1;            			//设置接收缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;								//数据传输方向，从外设读取到内存
	DMA1_Init.DMA_BufferSize = USART2_MAX_RX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输
	 
	DMA_Init(DMA1_Channel6,&DMA1_Init); 									//对DMA通道5进行初始化
	
	//DMA1_USART2_TX  RAM->USART2的数据传输
	DMA_DeInit(DMA1_Channel7);												//将DMA的通道4寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&UART2_USART->DR);				//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART2_TX_BUF;              		//设置发送缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								//数据传输方向，从内存发送到外设
	DMA1_Init.DMA_BufferSize = USART2_MAX_TX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输

	DMA_Init(DMA1_Channel7,&DMA1_Init); 									//对DMA通道4进行初始化
	
	//DMA1通道6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器
 
	//DMA1通道7 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器

	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);							//开USART1 Rx DMA中断
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);							//开USART1 Tx DMA中断

	DMA_Cmd(DMA1_Channel6,ENABLE);           								//使DMA通道5停止工作
	DMA_Cmd(DMA1_Channel7,DISABLE);           								//使DMA通道4停止工作
	 
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);        					//开启串口DMA发送
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);        					//开启串口DMA接收
}

void DMA1_USART3_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						//使能DMA1时钟

	//DMA1_USART3_RX  USART3->RAM的数据传输
	DMA_DeInit(DMA1_Channel3);												//将DMA的通道5寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&UART3_USART->DR);				//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART3_Rxbuf1;            			//设置接收缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;								//数据传输方向，从外设读取到内存
	DMA1_Init.DMA_BufferSize = USART3_MAX_RX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输
	 
	DMA_Init(DMA1_Channel3,&DMA1_Init); 									//对DMA通道5进行初始化
	
	//DMA_USART1_TX  RAM->USART1的数据传输
	DMA_DeInit(DMA1_Channel2);												//将DMA的通道4寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&UART3_USART->DR);				//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART3_TX_BUF;              		//设置发送缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								//数据传输方向，从内存发送到外设
	DMA1_Init.DMA_BufferSize = USART3_MAX_TX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输

	DMA_Init(DMA1_Channel2,&DMA1_Init); 									//对DMA通道4进行初始化
	
	//DMA1通道3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器
 
	//DMA1通道2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器

	DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);							//开USART1 Rx DMA中断
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);							//开USART1 Tx DMA中断

	DMA_Cmd(DMA1_Channel3,ENABLE);           								//使DMA通道5停止工作
	DMA_Cmd(DMA1_Channel2,DISABLE);           								//使DMA通道4停止工作
	 
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);        					//开启串口DMA发送
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);        					//开启串口DMA接收
}


//DMA1_4通道 USART1发送应用源码
char DMA1_USART1_TxData(u8 *buffer, u32 size)
{
	if(USART1_TX_FLAG)return 0;	//如果上一次发送未完成返回 1
	else
	{
		USART1_TX_FLAG=1;							//USART2发送标志（启动发送）
		DMA1_Channel4->CMAR  = (uint32_t)buffer;	//设置要发送的数据地址
		DMA1_Channel4->CNDTR = size; 				//设置要发送的字节数目
		DMA_Cmd(DMA1_Channel4, ENABLE);				//开始DMA发送
		return 1;
	}
}

//DMA1_4通道 USART2发送应用源码
char DMA1_USART2_TxData(u8 *buffer, u32 size)
{
	if(USART2_TX_FLAG)return 0;	//如果上一次发送未完成返回 1
	else
	{
	
    // 将数据拆分为高八位和低八位


    // 启动DMA传输


		USART2_TX_FLAG=1;							//USART2发送标志（启动发送）
		DMA1_Channel7->CMAR  = (uint32_t)buffer;	//设置要发送的数据地址
		DMA1_Channel7->CNDTR = size; 				//设置要发送的字节数目
		DMA_Cmd(DMA1_Channel7, ENABLE);				//开始DMA发送
		return 1;
	}
}

//DMA1_2通道 USART3发送应用源码
char DMA1_USART3_TxData(u8 *buffer, u32 size)
{
	if(USART3_TX_FLAG)return 0;	//如果上一次发送未完成返回 1
	else
	{
		USART3_TX_FLAG=1;							//USART2发送标志（启动发送）
		DMA1_Channel2->CMAR  = (uint32_t)buffer;	//设置要发送的数据地址
		DMA1_Channel2->CNDTR = size; 				//设置要发送的字节数目
		DMA_Cmd(DMA1_Channel2, ENABLE);				//开始DMA发送
		return 1;
	}
}

//发送中断  DMA1_4通道 USART1
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC4)!= RESET)	//DMA发送完成标志
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4); 	//清除中断标志 
		USART_ClearFlag(USART1,USART_FLAG_TC);	//清除串口2的标志位
		DMA_Cmd(DMA1_Channel4, DISABLE );   	//关闭USART2 TX DMA1 所指示的通道
		USART1_TX_FLAG=0;						//USART2发送标志(关闭)
	}
}
//发送中断  DMA1_7通道 USART2
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC7)!= RESET)	//DMA发送完成标志
	{
		DMA_ClearITPendingBit(DMA1_IT_TC7); 	//清除中断标志 
		USART_ClearFlag(USART2,USART_FLAG_TC);	//清除串口2的标志位
		DMA_Cmd(DMA1_Channel7, DISABLE );   	//关闭USART2 TX DMA1 所指示的通道
		USART2_TX_FLAG=0;						//USART2发送标志(关闭)
	}
}
//发送中断  DMA1_2通道 USART3
void DMA1_Channel2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC2)!= RESET)	//DMA发送完成标志
	{
		DMA_ClearITPendingBit(DMA1_IT_TC2); 	//清除中断标志 
		USART_ClearFlag(USART3,USART_FLAG_TC);	//清除串口2的标志位
		DMA_Cmd(DMA1_Channel2, DISABLE );   	//关闭USART2 TX DMA1 所指示的通道
		USART3_TX_FLAG=0;						//USART2发送标志(关闭)
	}
}

//串口1空闲中断函数
void USART1_IRQHandler(void)                	
{
	u8 i;
	u8 *p;
	u8 USART1_RX_LEN = 0;											//接收数据长度
	if(USART_GetITStatus(UART1_USART, USART_IT_IDLE) != RESET)			//串口2空闲中断
	{
		USART_ReceiveData(UART1_USART); 									//清除串口2空闲中断IDLE标志位
		USART_ClearFlag(UART1_USART,USART_FLAG_TC);						//清除USART1标志位
		DMA_Cmd(DMA1_Channel5, DISABLE );   						//关闭USART1 TX DMA1 所指示的通道
		USART1_RX_LEN = USART1_MAX_RX_LEN - DMA1_Channel5->CNDTR;	//获得接收到的字节数
		if(Mark_USART1_Rxbuf)                        						//之前用的u2rxbuf，切换为u1rxbuf
		{
			p = USART1_Rxbuf2;												//先保存前一次数据地址再切换缓冲区
			memset( USART1_Rxbuf1, 0, USART1_MAX_RX_LEN );
			DMA1_Channel5->CMAR = (u32)USART1_Rxbuf1;						//切换为u1rxbuf缓冲区地址
			Mark_USART1_Rxbuf = 0;                     						//下一次切换为u2rxbuf
		}else                               						//之前用的u1rxbuf，切换为u2rxbuf
		{
			p=USART1_Rxbuf1;												//先保存前一次数据地址再切换缓冲区
			memset( USART1_Rxbuf2, 0, USART1_MAX_RX_LEN );
			DMA1_Channel5->CMAR = (u32)USART1_Rxbuf2;						//切换为u2rxbuf缓冲区地址
			Mark_USART1_Rxbuf = 1;                     						//下一次切换为u1rxbuf
		}
		DMA1_Channel5->CNDTR = USART1_MAX_RX_LEN;					//DMA通道的DMA缓存的大小
		DMA_Cmd(DMA1_Channel5, ENABLE);     						//使能USART1 TX DMA1 所指示的通道
		
		if(p != NULL)
		{
			RX1_BUF[0] = USART1_RX_LEN;
			for(i = 0; i < USART1_RX_LEN; i++)
			{
				RX1_BUF[ i + 1 ] = p[ i ];
			}
			USART1_RX_FLAG = 1;
		}
	}
}

//串口2空闲中断函数
void USART2_IRQHandler(void)                	
{
	u8 i;
	u8 *p;
	u8 USART2_RX_LEN = 0;											//接收数据长度
	if(USART_GetITStatus(UART2_USART, USART_IT_IDLE) != RESET)			//串口2空闲中断
	{
		USART_ReceiveData(UART2_USART); 									//清除串口2空闲中断IDLE标志位
		USART_ClearFlag(UART2_USART,USART_FLAG_TC);						//清除USART1标志位
		DMA_Cmd(DMA1_Channel6, DISABLE );   						//关闭USART1 TX DMA1 所指示的通道
		USART2_RX_LEN = USART2_MAX_RX_LEN - DMA1_Channel6->CNDTR;	//获得接收到的字节数
		if(Mark_USART2_Rxbuf)                        						//之前用的u2rxbuf，切换为u1rxbuf
		{
			p = USART2_Rxbuf2;												//先保存前一次数据地址再切换缓冲区
			memset( USART2_Rxbuf1, 0, USART2_MAX_RX_LEN );
			DMA1_Channel6->CMAR = (u32)USART2_Rxbuf1;						//切换为u1rxbuf缓冲区地址
			Mark_USART2_Rxbuf = 0;                     						//下一次切换为u2rxbuf
		}else                               						//之前用的u1rxbuf，切换为u2rxbuf
		{
			p=USART2_Rxbuf1;												//先保存前一次数据地址再切换缓冲区
			memset( USART2_Rxbuf2, 0, USART2_MAX_RX_LEN );
			DMA1_Channel6->CMAR = (u32)USART2_Rxbuf2;						//切换为u2rxbuf缓冲区地址
			Mark_USART2_Rxbuf = 1;                     						//下一次切换为u1rxbuf
		}
		DMA1_Channel6->CNDTR = USART2_MAX_RX_LEN;					//DMA通道的DMA缓存的大小
		DMA_Cmd(DMA1_Channel6, ENABLE);     						//使能USART1 TX DMA1 所指示的通道
		
		if(p != NULL)
		{
			RX2_BUF[0] = USART2_RX_LEN;
			for(i = 0; i < USART2_RX_LEN; i++)
			{
				RX2_BUF[ i + 1 ] = p[ i ];
			}
			USART2_RX_FLAG = 1;
		}
	}
}

//串口1空闲中断函数
void USART3_IRQHandler(void)                	
{
	u8 i;
	u8 *p;
	u8 USART3_RX_LEN = 0;											//接收数据长度
	if(USART_GetITStatus(UART3_USART, USART_IT_IDLE) != RESET)			//串口2空闲中断
	{
		USART_ReceiveData(UART3_USART); 									//清除串口2空闲中断IDLE标志位
		USART_ClearFlag(UART3_USART,USART_FLAG_TC);						//清除USART1标志位
		DMA_Cmd(DMA1_Channel3, DISABLE );   						//关闭USART1 TX DMA1 所指示的通道
		USART3_RX_LEN = USART3_MAX_RX_LEN - DMA1_Channel3->CNDTR;	//获得接收到的字节数
		if(Mark_USART3_Rxbuf)                        						//之前用的u2rxbuf，切换为u1rxbuf
		{
			p = USART3_Rxbuf2;												//先保存前一次数据地址再切换缓冲区
			memset( USART3_Rxbuf1, 0, USART3_MAX_RX_LEN );
			DMA1_Channel3->CMAR = (u32)USART3_Rxbuf1;						//切换为u1rxbuf缓冲区地址
			Mark_USART3_Rxbuf = 0;                     						//下一次切换为u2rxbuf
		}else                               						//之前用的u1rxbuf，切换为u2rxbuf
		{
			p=USART3_Rxbuf1;												//先保存前一次数据地址再切换缓冲区
			memset( USART3_Rxbuf2, 0, USART3_MAX_RX_LEN );
			DMA1_Channel3->CMAR = (u32)USART3_Rxbuf2;						//切换为u2rxbuf缓冲区地址
			Mark_USART3_Rxbuf = 1;                     						//下一次切换为u1rxbuf
		}
		DMA1_Channel3->CNDTR = USART3_MAX_RX_LEN;					//DMA通道的DMA缓存的大小
		DMA_Cmd(DMA1_Channel3, ENABLE);     						//使能USART1 TX DMA1 所指示的通道
		
		if(p != NULL)
		{
			RX3_BUF[0] = USART3_RX_LEN;
			for(i = 0; i < USART3_RX_LEN; i++)
			{
				RX3_BUF[ i + 1 ] = p[ i ];
			}
			USART3_RX_FLAG = 1;
		}
	}
}


//#define BYTE0(dwTemp) (*(char *)(&dwTemp))
//#define BYTE1(dwTemp) (*((char *)(&dwTemp)+1))
//#define BYTE2(dwTemp) (*((char *)(&dwTemp)+2))
//#define BYTE3(dwTemp) (*((char *)(&dwTemp)+3))
//	
//uint8_t BUFF[100];
//uint16_t flen;
//void sent_data1(float A,float B,float C)
//{	
//		uint8_t _cnt = 0;

//		BUFF[_cnt++]=32;//数据长度（2字节）
//		BUFF[_cnt++]=0;//数据长度
//		BUFF[_cnt++]=BYTE0(A);//数据内容A（2字节）
//		BUFF[_cnt++]=BYTE1(A);

//	
//		BUFF[_cnt++]=BYTE0(B);//数据内容B（4字节）
//		BUFF[_cnt++]=BYTE1(B);

//	
//		BUFF[_cnt++]=BYTE0(C);//数据内容C（4字节）
//		BUFF[_cnt++]=BYTE1(C);


//	
//		uint8_t sumcheck = 0;
//		uint8_t addcheck = 0;
//		uint16_t flen = BUFF[4] + BUFF[5] * 255;
//		for(uint16_t i=0; i < (flen+6); i++)//校验计算
//		{
//			sumcheck += BUFF[i]; 
//			addcheck += sumcheck; 
//		}
//		
//		BUFF[_cnt++] = sumcheck;//和校验
//		BUFF[_cnt++] = addcheck;//附加校验
//		DMA1_USART2_TxData(BUFF,_cnt);//串口发送数组uart_tx1[0]为数据第一位
//}















