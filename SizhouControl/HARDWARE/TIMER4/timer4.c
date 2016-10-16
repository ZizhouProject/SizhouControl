#include "timer4.h"

void TIM1_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<11;	//TIM4时钟使能    
 	TIM1->ARR=arr;  	//设定计数器自动重装值 
	TIM1->PSC=psc;  	//预分频器设置
	TIM1->DIER|=1<<0;   //允许更新中断				
	TIM1->CR1|=0x01;    //使能定时器3
  MY_NVIC_Init(1,3,TIM1_UP_IRQn,2);//抢占1，子优先级3，组2									 
}

void TIM1_UP_IRQHandler(void)
{ 		    		  			    
	if(TIM1->SR&0X0001)//溢出中断;
	{
		   flag_reporter++;
		   flag_control++;
		   flag_ult_count++;
	
	}			
	TIM1->SR&=~(1<<0);//清除中断标志位 	    
}