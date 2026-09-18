#include "adc.h"



static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1 and GPIOC clock */   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	/* Configure PA.4、5、6  as analog input */   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;    
	GPIO_Init(GPIOA, &GPIO_InitStructure);   // PA
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;    
	GPIO_Init(GPIOC, &GPIO_InitStructure);   // PC
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;    
	GPIO_Init(GPIOB, &GPIO_InitStructure);   // PB

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ADC1_Mode_Config(u16 *Value_array)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                        //独立工作模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			                    //多通道
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			            //连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;       //由软件触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;					//Right
	ADC_InitStructure.ADC_NbrOfChannel = 9;				                    //9个通道转换
	ADC_Init(ADC1, &ADC_InitStructure);

  	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 

	/* ADC1 regular channel16 configuration */ 
	//设置采样通道IN16, 设置采样时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);//28us	ADC总转换时间=（采样时间+12.5）个ADC时钟周期 （239.5+12.5）*（1/9000000）
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 7, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 8, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 9, ADC_SampleTime_239Cycles5);	
	
  
	/* Convert the ADC1 Channel 17 with 239.5 Cycles as sampling time */ 
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint ,16,  ADC_SampleTime_239Cycles5);
	//使能温度传感器和内部参考电压   
	ADC_TempSensorVrefintCmd(ENABLE);

	/* Enable ADC1 DMA */	  
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);			                              
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));                         

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);				                        
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));	  
	 
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DMA_DeInit(DMA1_Channel1);//DMA
	/* DMA channel1 configuration */		   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;  //外设基地址 #define ADC1_DR_Address ((u32)0x4001244C)
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Value_array;	        // AD转换值所存放的内存基地址	（就是给个地址）
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                        // 外设作为数据传输的来源	
	DMA_InitStructure.DMA_BufferSize = 9;     // 定义指定DMA通道 DMA缓存的大小  u16数据宽度2，总共32字节的数据，则32/2=16
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          // 外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			        // 内存地址寄存器变
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // HalfWord
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		                    //工作在循环模式下
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                    //高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		                        //没有设置为内存到内存的传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	//DMA1通道1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);

	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);											//ENABLE DMA


}
////////////////////////////////////////////////////////////////////////////////

/*
 * 函数名：Temp_ADC1_Init
 * 描述  ：无
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void ADC_Config(u16 *Value_array)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config(Value_array);
}
