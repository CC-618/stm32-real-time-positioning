#include "main.h"


	
int main(void)
{		

	HardWare_init();
	Encoder_EXTIX_Init();

	
	Can_Send_Msg(ID_Self, CAN1, CAN_tx_buff, 8);
	
	while(1)
	{
		Key_Scan();
		Key_execute();
		Switch_num = DipSwitch_Read();
		
		if(AD_Read_flag == 1)
		{
			AD_Read_flag = 0;
			Read_ad();
		}
		if(USART1_RX_FLAG == 1)
		{
			USART1_RX_FLAG = 0;
			TransJY931();
//			OLED_ShowSignedNum(2, 1, RX1_BUF[1], 5);
		}
		if(USART2_RX_FLAG == 1)
		{
			USART2_RX_FLAG = 0;
			if(RX2_BUF[1] == 22)
			{
				RX2_BUF[1] = 0;
				Angle_z_temp = 0;
				Displacement.x = 0;
				Displacement.y = 0;
			}
		}
	
		if(uart2_Tx_flag)
		{
			uart2_Tx_flag = 0;		
			car_data_uart(Displacement.x,Displacement.y,Angle_z_temp);
		}
	}
}
//定时器2中断服务程序
void TIM2_IRQHandler(void)   //TIM2中断 2ms
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
		Angle_Calcu();
		Velocity_Fusion();//更新编码器速度，位置	
		uart2_Tx_flag=1;				
	}
}

//CAN中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	u8 i,flag = 0;
	u8 ID_temp;
	CanRxMsg rx_message;
	if(CAN_GetITStatus( CAN1,CAN_IT_FMP0 ) !=  RESET) 
	{
		CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
		CAN_Receive( CAN1, CAN_FIFO0, &rx_message );
//		if(rx_message.Data[0] == 0x05)
//		{
//			OLED_ShowString(4, 12, "05");
//		}
//		else if(rx_message.Data[0] == 0x06)
//		{
//			OLED_ShowString(4, 12, "06");
//		}

	}
}


//AD_DMA1通道1中断
void DMA1_Channel1_IRQHandler(void)
{
	u8 i;
	if(DMA_GetITStatus(DMA1_IT_TC1)!= RESET)	//DMA接收完成标志
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1); 	//清除中断标志 
		
	}
}














