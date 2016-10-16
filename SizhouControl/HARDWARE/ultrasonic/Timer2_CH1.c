#include "Timer2CH1.h"

//��ʱ��2ͨ��1���벶������ arr���Զ���װֵ�� psc��ʱ��Ԥ��Ƶϵ��
void TIM2_Catch_Init(u16 arr,u16 psc)
{		 
	RCC->APB1ENR|=1<<0;   						//TIM2 ʱ��ʹ��
	RCC->APB2ENR|=1<<2;    						//ʹ��PORTA
	 
	GPIOA->CRL&=0XFFFFFFF0;						//PA0����
	GPIOA->CRL|=0X00000008;				
	GPIOA->ODR|=0<<0;		
	  
 	TIM2->ARR=arr;  								//�趨�������Զ���װֵ
	TIM2->PSC=psc;  								//Ԥ��Ƶ��

	TIM2->CCMR1|=1<<0;						//CC1S=01 	ѡ�������IC1ӳ�䵽TI1
 	TIM2->CCMR1|=1<<4; 						//IC1F=0001 ���������˲��� ��FCK_INT�����������¼�����Ч
 	TIM2->CCMR1|=0<<10;					 	//IC2PS=00	 ���������Ƶ������Ƶ

	TIM2->CCER|=0<<1; 							//CC1P=0	�����ز���
	TIM2->CCER|=1<<0; 							//CC1E=1 	�������������ֵ������Ĵ���

	TIM2->DIER|=1<<1;   							//�������ж�
	TIM2->DIER|=1<<0;   							//��������ж�
	TIM2->CR1|=0x01;    							//ʹ�ܶ�ʱ��2
	MY_NVIC_Init(2,0,TIM2_IRQn,2);			//��ռ2�������ȼ�0����2
}

//Timer2�жϷ�����
void TIM2_IRQHandler(void)
{ 		    
	u16 tsr;
	tsr=TIM2->SR;
 	if((flag_catch&0X80)==0)		//��δ�ɹ�����
	{
		if(tsr&0X01)						//���
		{	    
			if(flag_catch&0X40)		//�Ѿ����񵽸ߵ�ƽ
			{
				if((flag_catch&0X3F)==0X3F)		//�ߵ�ƽ̫��
				{
					flag_catch|=0X80;				//��ǳɹ�������һ��
					flag_val=0XFFFF;
				}else flag_catch++;
			}	 
		}
		if(tsr&0x02)								//����1���������¼�
		{	
			if(flag_catch&0X40)			//����һ���½���
			{	  			
				flag_catch|=0X80;			//��ǳɹ�����һ�θߵ�ƽ����
			    flag_val=TIM2->CCR1;			//��ȡ��ǰ�Ĳ���ֵ
	 			TIM2->CCER&=~(1<<1);			//CC1P=0���������ز���
			}else  											//��δ��ʼ����һ�β���������
			{ 
				flag_val=0;
				flag_catch=0X40;						//��ǲ�����������
				TIM2->CNT=0;						//���������
				TIM2->CCER|=1<<1; 				//CC1P=1 �����½���Ϊ����
			}		    
		}			     	    					   
 	}
	TIM2->SR=0;										//����жϱ�־
}


















