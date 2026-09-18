#include "main.h"
CanRxMsg rx_message;
u32 TIM_Control;
u32 Can_Controlg;
u8 can_Flag;
int main(void)
{
	/*模块初始化*/
	HardWare_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	while (1)
	{			
		Key_Scan();//按键识别
		Key_execute();//按键执行程序
		OLED_Show();//OLED
		USART1_TX_RX();//机械臂摄像头通信		PA9 TX PA10 RX	
//		USART2_RX_Ctrl();//串口屏通信		PA2 TX PA3 RX
		USART3_RX_Ctrl();//陀螺仪，编码器通信	PB10 TX PB11 RX
		if(Can_Controlg>=50)
		{		
			Can_Controlg=0;
			Can_Send_Msg(ID_Self, CAN1, CAN_tx_buff, 8);
		}
		Pick_Place();//取放	
	}
}
	
//定时器2中断服务程序
void TIM2_IRQHandler(void)   //TIM2中断 
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
		TIM_Control++;
		Can_Controlg++;
		if(TIM_Control%2==0)
		{
			Key_Control();//按键控制
//			IWDG_Counter(200);//喂狗大约400ms内			
		}
		if(TIM_Control%10==0)
		{
//			Motor_Control();//总控制器
			TIM_Control=0;
		}	
	}
}

//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断 
{
	interrupt();
}

//CAN中断服务函数，与机械臂通信    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	if(CAN_GetITStatus( CAN1,CAN_IT_FMP0 ) !=  RESET) 
	{
		CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
		CAN_Receive( CAN1, CAN_FIFO0, &rx_message );			
		if(Incident_flag == 3)
		{		
			/*二维码识别*/
			Switch_Ctrl();
		}		
	}
}


//AD_DMA1通道1中断
void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC1)!= RESET)	//DMA接收完成标志
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1); 	//清除中断标志 
		
	}
}
