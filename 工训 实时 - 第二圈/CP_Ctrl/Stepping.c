#include "Stepping.h"
#include "output.h"

/********************************电机轮子,梯形加减速*****************************************************************************/
speedRampData g_srd               = {STOP,CW,0,0,0,0,0};  /* 加减速变量 */
//__IO int32_t  g_step_position     = 0;                    /* 当前位置 */
__IO uint8_t  g_motion_sta        = 0;                    /* 是否在运动？0：停止，1：运动 */
//__IO uint32_t g_add_pulse_count   = 0;                    /* 脉冲个数累计 */
u8 ctrl_motio;

/*
 * @brief       生成梯形运动控制参数
 * @param       step：移动的步数 (正数为顺时针，负数为逆时针).
 * @param       accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param       decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param       speed  最大速度,实际值为speed*0.1*rad/sec
 * @param       state  状态，车轮或机械臂模式
 * @param       EN     使能或失能电机1号
 * @retval      无
 */
void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
    __IO uint16_t tim_count;        /* 达到最大速度时的步数*/
    __IO uint32_t max_s_lim;        /* 必须要开始减速的步数（如果加速没有达到最大速度）*/
    __IO uint32_t accel_lim;
    if(g_motion_sta != STOP)        /* 只允许步进电机在停止的时候才继续*/
        return;
    if(step < 0)                    /* 步数为负数 */
    {   
//        g_srd.dir = CCW;            /* 逆时针方向旋转 */
//        Reversal_DIR(Num);
        step = -step;               /* 获取步数绝对值 */
    }
//    else
//    {
//        g_srd.dir = CW;             /* 顺时针方向旋转 */
//        Foreward_DIR(Num);
//    }

    if(step == 1)                   /* 步数为1 */
    {
        g_srd.accel_count = -1;     /* 只移动一步 */
        g_srd.run_state = DECEL;    /* 减速状态. */
        g_srd.step_delay = 1000;    /* 默认速度 */
    }
    else if(step != 0)              /* 如果目标运动步数不为0*/
    {
        /*设置最大速度极限, 计算得到min_delay用于定时器的计数器的值 min_delay = (alpha / t)/ w*/
        g_srd.min_delay = (int32_t)(A_T_x10 /speed); //匀速运行时的计数值

        /* 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
         step_delay = 1/tt * sqrt(2*alpha/accel)
         step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 */
        
        g_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10); /* c0 */

        max_s_lim = (uint32_t)(speed*speed / (A_x200*accel/10));/* 计算多少步之后达到最大速度的限制 max_s_lim = speed^2 / (2*alpha*accel) */

        if(max_s_lim == 0)                                      /* 如果达到最大速度小于0.5步，我们将四舍五入为0,但实际我们必须移动至少一步才能达到想要的速度 */
        {
            max_s_lim = 1;
        }
        accel_lim = (uint32_t)(step*decel/(accel+decel));       /* 这里不限制最大速度 计算多少步之后我们必须开始减速 n1 = (n1+n2)decel / (accel + decel) */

        if(accel_lim == 0)                                      /* 不足一步 按一步处理*/
        {
            accel_lim = 1;
        }
        if(accel_lim <= max_s_lim)                              /* 加速阶段到不了最大速度就得减速。。。使用限制条件我们可以计算出减速阶段步数 */
        {
            g_srd.decel_val = accel_lim - step;                 /* 减速段的步数 */
        }
        else
        {
            g_srd.decel_val = -(max_s_lim*accel/decel);         /* 减速段的步数 */
        }
        if(g_srd.decel_val == 0)                                /* 不足一步 按一步处理 */
        {
            g_srd.decel_val = -1;
        }
        g_srd.decel_start = step + g_srd.decel_val;             /* 计算开始减速时的步数 */
        
        
        if(g_srd.step_delay <= g_srd.min_delay)                 /* 如果一开始c0的速度比匀速段速度还大，就不需要进行加速运动，直接进入匀速 */
        {
            g_srd.step_delay = g_srd.min_delay;
            g_srd.run_state = RUN;
        }
        else  
        {
            g_srd.run_state = ACCEL;
        }
        g_srd.accel_count = 0;                                  /* 复位加减速计数值 */
    }
    g_motion_sta = 1;                                           /* 电机为运动状态 */
//    ST3_EN(EN_ON);	
	tim_count = TIM_GetCounter(TIM3);														//获取计数值
	
/*	设置定时器比较值,使能定时器通道,中断通道*/

		TIM_SetCompare1(TIM3,tim_count+g_srd.step_delay/2);												
		TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);																		 
		TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Enable);

		TIM_SetCompare2(TIM3,tim_count+g_srd.step_delay/2);												 
		TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);																		 
		TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);

		TIM_SetCompare3(TIM3,tim_count+g_srd.step_delay/2);												 
		TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);																		
		TIM_CCxCmd(TIM3,TIM_Channel_3,TIM_CCx_Enable);	

		TIM_SetCompare4(TIM3,tim_count+g_srd.step_delay/2);												
		TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);																		
		TIM_CCxCmd(TIM3,TIM_Channel_4,TIM_CCx_Enable);

	
//	TIM_SetCompare4(TIM3,tim_count+g_srd.step_delay/2);												
//	TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);																		
//	TIM_CCxCmd(TIM3,TIM_Channel_4,TIM_CCx_Enable);

	TIM_Cmd(TIM3, ENABLE);																//开启定时器

//    Enter_counter_value(Num, tim_count, g_srd.step_delay/2);  /* 设置定时器比较值 */
//    stepper_star(Num);                                 /* 使能定时器通道 */
}

/*  加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
    所以加速度和减速度值一般是在实际应用中多尝试出来的结果 */
/*前进，后退*/
__IO uint32_t g_set_speed  = 700;          /* 最大速度 单位为0.1rad/sec */
__IO uint32_t g_step_accel = 400;            /* 加速度 单位为0.1rad/sec^2 */
__IO uint32_t g_step_decel = 400;            /* 减速度 单位为0.1rad/sec^2 */
__IO uint16_t g_step_angle = 20;             /* 设置的步数*/

/*左移，右移*/
__IO uint32_t LR_set_speed  = 100;          /* 最大速度 单位为0.1rad/sec */
__IO uint32_t LR_step_accel = 50;            /* 加速度 单位为0.1rad/sec^2 */
__IO uint32_t LR_step_decel = 50;            /* 减速度 单位为0.1rad/sec^2 */
__IO uint16_t LR_step_angle = 20;             /* 设置的步数*/

void interrupt(void)
{
    __IO uint32_t tim_count = 0;
    __IO uint32_t tmp = 0;
    uint16_t new_step_delay = 0;                            /* 保存新（下）一个延时周期 */
    __IO static uint16_t last_accel_delay = 0;              /* 加速过程中最后一次延时（脉冲周期） */
    __IO static uint32_t step_count = 0;                    /* 总移动步数计数器*/
    __IO static int32_t rest = 0;                           /* 记录new_step_delay中的余数，提高下一步计算的精度 */
    __IO static uint8_t i = 0;                              /* 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲 */
	 if ((TIM_GetITStatus(TIM3, TIM_IT_CC1)== SET) ||(TIM_GetITStatus(TIM3, TIM_IT_CC2)== SET)
		   ||TIM_GetITStatus(TIM3, TIM_IT_CC3)== SET ||(TIM_GetITStatus(TIM3, TIM_IT_CC4)== SET))
	 {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);															// 清楚定时器中断		
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);		  
		tim_count = TIM_GetCounter(TIM3);						//获取计数值
		tmp = tim_count + g_srd.step_delay/2;               /* 整个C值里边是需要翻转两次的所以需要除以2 */
		TIM_SetCompare1(TIM3,tmp);												//设置定时器比较值 
		TIM_SetCompare2(TIM3,tmp);												//设置定时器比较值
		TIM_SetCompare3(TIM3,tmp);												//设置定时器比较值
		TIM_SetCompare4(TIM3,tmp);												//设置定时器比较值
	//	Enter_counter_value(Num, tim_count, g_srd.step_delay/2);  /* 设置定时器比较值 */

		i++;                                                /* 定时器中断次数计数值 */
		if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
		{
			i = 0;                                          /* 清零定时器中断次数计数值 */
			switch(g_srd.run_state)                         /* 加减速曲线阶段 */
			{
			case STOP:
				step_count = 0;                             /* 清零步数计数器 */
				rest = 0;                                   /* 清零余值 */
				/* 关闭通道*/
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
				TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Disable);
				TIM_ITConfig(TIM3,TIM_IT_CC1,DISABLE);
			
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
				TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Disable);
				TIM_ITConfig(TIM3,TIM_IT_CC2,DISABLE);
			
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
				TIM_CCxCmd(TIM3,TIM_Channel_3,TIM_CCx_Disable);
				TIM_ITConfig(TIM3,TIM_IT_CC3,DISABLE);
			
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
				TIM_CCxCmd(TIM3,TIM_Channel_4,TIM_CCx_Disable);	
				TIM_ITConfig(TIM3,TIM_IT_CC4,DISABLE);
			
				TIM_Cmd(TIM3, DISABLE);					// 单个电机可以关闭定时器，多个电机只关闭通道即可			
	//			stepper_stop(Num);
	//			ST3_EN(EN_OFF);
				g_motion_sta = 0;                           /* 电机为停止状态  */
				break;

			case ACCEL:
	//			g_add_pulse_count++;                        /* 只用于记录相对位置转动了多少度 */
				step_count++;                               /* 步数加1*/
	//			if(g_srd.dir == CW)
	//			{
	//				g_step_position++;                      /* 绝对位置加1  记录绝对位置转动多少度*/
	//			}
	//			else
	//			{
	//				g_step_position--;                      /* 绝对位置减1*/
	//			}
				g_srd.accel_count++;                        /* 加速计数值加1*/
				new_step_delay = g_srd.step_delay - (((2 *g_srd.step_delay) + rest)/(4 * g_srd.accel_count + 1));/* 计算新(下)一步脉冲周期(时间间隔) */
				rest = ((2 * g_srd.step_delay)+rest)%(4 * g_srd.accel_count + 1);                                /* 计算余数，下次计算补上余数，减少误差 */
				if(step_count >= g_srd.decel_start)         /* 检查是否到了需要减速的步数 */
				{
					g_srd.accel_count = g_srd.decel_val;    /* 加速计数值为减速阶段计数值的初始值 */
					g_srd.run_state = DECEL;                /* 下个脉冲进入减速阶段 */
				}
				else if(new_step_delay <= g_srd.min_delay)  /* 检查是否到达期望的最大速度 计数值越小速度越快，当你的速度和最大速度相等或更快就进入匀速*/
				{
					last_accel_delay = new_step_delay;      /* 保存加速过程中最后一次延时（脉冲周期）*/
					new_step_delay = g_srd.min_delay;       /* 使用min_delay（对应最大速度speed）*/
					rest = 0;                               /* 清零余值 */
					g_srd.run_state = RUN;                  /* 设置为匀速运行状态 */
				}
				break;

			case RUN:
	//			g_add_pulse_count++;
				step_count++;                               /* 步数加1 */
	//			if(g_srd.dir == CW)
	//			{
	//				g_step_position++;                      /* 绝对位置加1 */
	//			}
	//			else
	//			{
	//				g_step_position--;                      /* 绝对位置减1*/
	//			}
				new_step_delay = g_srd.min_delay;           /* 使用min_delay（对应最大速度speed）*/
				if(step_count >= g_srd.decel_start)         /* 需要开始减速 */
				{
					g_srd.accel_count = g_srd.decel_val;    /* 减速步数做为加速计数值 */
					new_step_delay = last_accel_delay;      /* 加阶段最后的延时做为减速阶段的起始延时(脉冲周期) */
					g_srd.run_state = DECEL;                /* 状态改变为减速 */
				}
				break;

			case DECEL:
				step_count++;                               /* 步数加1 */
	//			g_add_pulse_count++;
	//			if(g_srd.dir == CW)
	//			{
	//				g_step_position++;                      /* 绝对位置加1 */
	//			}
	//			else
	//			{
	//				g_step_position--;                      /* 绝对位置减1 */
	//			}
				g_srd.accel_count++;
				new_step_delay = g_srd.step_delay - (((2 * g_srd.step_delay) + rest)/(4 * g_srd.accel_count + 1));  /* 计算新(下)一步脉冲周期(时间间隔) */
				rest = ((2 * g_srd.step_delay)+rest)%(4 * g_srd.accel_count + 1);                                   /* 计算余数，下次计算补上余数，减少误差 */

				/* 检查是否为最后一步 */
				if(g_srd.accel_count >= 0)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
				{
					g_srd.run_state = STOP;
				}
				break;
			}
			g_srd.step_delay = new_step_delay;              /* 为下个(新的)延时(脉冲周期)赋值 */
		}
	}
}

//电机轮子,运动模式
void dir_out(int motor_A,int motor_B,int motor_C,int motor_D)
{
	if(motor_A==1)
	{
		PBout(3)=OUTPUTOff;
	}
		if(motor_A==2)
	{
		PBout(3)=OUTPUTOn;
	}
	
	if(motor_B==1)
	{
		PBout(4)=OUTPUTOn;
	}
	if(motor_B==2)
	{
		PBout(4)=OUTPUTOff;
	}
	
	if(motor_C==1)
	{
		PBout(5)=OUTPUTOff;
	}
	if(motor_C==2)
	{
		PBout(5)=OUTPUTOn;
	}
	
	if(motor_D==1)
	{
		 PBout(6)=OUTPUTOn;
	}
	if(motor_D==2)
	{
		PBout(6)=OUTPUTOff;
	}	
}
//电机轮子,使能失能
void Enabled_Disenabled(int motor_A,int motor_B,int motor_C,int motor_D)
{
	if(motor_A==0)
	{
		PCout(4)=OUTPUTOn;
	}
		if(motor_A==1)
	{
		PCout(4)=OUTPUTOff;
	}
	
		if(motor_B==0)
	{
		PCout(5)=OUTPUTOn;
	}
		if(motor_B==1)
	{
		PCout(5)=OUTPUTOff;
	}
	
		if(motor_C==0)
	{
		PBout(0)=OUTPUTOn;
	}
		if(motor_C==1)
	{
		PBout(0)=OUTPUTOff;
	}
	
		if(motor_D==0)
	{
		PBout(1)=OUTPUTOn;
	}
		if(motor_D==1)
	{
		PBout(1)=OUTPUTOff;
	}	
}
/**
 * @brief 车子前进
 * 
 * @param step   移动步数（正数为正转，负数为逆时针）
 * @param accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param speed  最大速度,实际值为speed*0.1*rad/sec
 * @return null
 */
void go(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	dir_out(1,2,2,1);
	create_t_ctrl_param(step, accel, decel, speed);
}

/**
 * @brief 车子后退
 * 
 * @param step   移动步数（正数为正转，负数为逆时针）
 * @param accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param speed  最大速度,实际值为speed*0.1*rad/sec
 * @return null
 */
void back(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	dir_out(2,2,2,2);
	create_t_ctrl_param(step, accel, decel, speed);
}

/**
 * @brief 车子右横移
 * 
 * @param step   移动步数（正数为正转，负数为逆时针）
 * @param accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param speed  最大速度,实际值为speed*0.1*rad/sec
 * @return null
 */
void right(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{	
	dir_out(2,2,1,1);	
	create_t_ctrl_param(step, accel, decel, speed);
}

/**
 * @brief 车子左横移
 * 
 * @param step   移动步数（正数为正转，负数为逆时针）
 * @param accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param speed  最大速度,实际值为speed*0.1*rad/sec
 * @return null
 */
void left(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{

    
	dir_out(1,1,2,2);
	create_t_ctrl_param(step, accel, decel, speed);
}

/**
 * @brief 车子左转弯
 * 
 * @param step   移动步数（正数为正转，负数为逆时针）
 * @param accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param speed  最大速度,实际值为speed*0.1*rad/sec
 * @return null
 */
void turn_left(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{	
	dir_out(2,1,2,1);
	create_t_ctrl_param(step, accel, decel, speed);
}

/**
 * @brief 车子右转弯
 * 
 * @param step   移动步数（正数为正转，负数为逆时针）
 * @param accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param speed  最大速度,实际值为speed*0.1*rad/sec
 * @return null
 */
void turn_right(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	dir_out(1,2,1,2);
	create_t_ctrl_param(step, accel, decel, speed);
}
