#include "timer4.h"

void TIM1_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<11;	//TIM4ʱ��ʹ��    
 	TIM1->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM1->PSC=psc;  	//Ԥ��Ƶ������
	TIM1->DIER|=1<<0;   //��������ж�				
	TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  MY_NVIC_Init(1,3,TIM1_UP_IRQn,2);//��ռ1�������ȼ�3����2									 
}

void TIM1_UP_IRQHandler(void)
{ 		    		  			    
	if(TIM1->SR&0X0001)//����ж�;
	{
		   flag_reporter++;
		   flag_control++;
		   flag_ult_count++;
	
	}			
	TIM1->SR&=~(1<<0);//����жϱ�־λ 	    
}