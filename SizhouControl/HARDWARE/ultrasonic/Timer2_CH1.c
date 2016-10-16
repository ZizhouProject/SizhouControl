#include "Timer2CH1.h"

//定时器2通道1输入捕获配置 arr：自动重装值； psc：时钟预分频系数
void TIM2_Catch_Init(u16 arr,u16 psc)
{		 
	RCC->APB1ENR|=1<<0;   						//TIM2 时钟使能
	RCC->APB2ENR|=1<<2;    						//使能PORTA
	 
	GPIOA->CRL&=0XFFFFFFF0;						//PA0配置
	GPIOA->CRL|=0X00000008;				
	GPIOA->ODR|=0<<0;		
	  
 	TIM2->ARR=arr;  								//设定计数器自动重装值
	TIM2->PSC=psc;  								//预分频器

	TIM2->CCMR1|=1<<0;						//CC1S=01 	选择输入端IC1映射到TI1
 	TIM2->CCMR1|=1<<4; 						//IC1F=0001 配置输入滤波器 以FCK_INT采样，两个事件后有效
 	TIM2->CCMR1|=0<<10;					 	//IC2PS=00	 设置输入分频，不分频

	TIM2->CCER|=0<<1; 							//CC1P=0	上升沿捕获
	TIM2->CCER|=1<<0; 							//CC1E=1 	允许捕获计数器的值到捕获寄存器

	TIM2->DIER|=1<<1;   							//允许捕获中断
	TIM2->DIER|=1<<0;   							//允许更新中断
	TIM2->CR1|=0x01;    							//使能定时器2
	MY_NVIC_Init(2,0,TIM2_IRQn,2);			//抢占2，子优先级0，组2
}

//Timer2中断服务函数
void TIM2_IRQHandler(void)
{ 		    
	u16 tsr;
	tsr=TIM2->SR;
 	if((flag_catch&0X80)==0)		//还未成功捕获
	{
		if(tsr&0X01)						//溢出
		{	    
			if(flag_catch&0X40)		//已经捕获到高电平
			{
				if((flag_catch&0X3F)==0X3F)		//高电平太长
				{
					flag_catch|=0X80;				//标记成功捕获了一次
					flag_val=0XFFFF;
				}else flag_catch++;
			}	 
		}
		if(tsr&0x02)								//捕获1发生捕获事件
		{	
			if(flag_catch&0X40)			//捕获到一个下降沿
			{	  			
				flag_catch|=0X80;			//标记成功捕获到一次高电平脉宽
			    flag_val=TIM2->CCR1;			//获取当前的捕获值
	 			TIM2->CCER&=~(1<<1);			//CC1P=0设置上升沿捕获
			}else  											//还未开始，第一次捕获上升沿
			{ 
				flag_val=0;
				flag_catch=0X40;						//标记捕获到了上升沿
				TIM2->CNT=0;						//计数器清空
				TIM2->CCER|=1<<1; 				//CC1P=1 设置下降沿为捕获
			}		    
		}			     	    					   
 	}
	TIM2->SR=0;										//清除中断标志
}


















