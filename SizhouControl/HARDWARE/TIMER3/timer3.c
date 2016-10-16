#include "timer3.h"

void TIM3_PWM_Init(u16 arr,u16 psc)
{		 					 
	
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能 
	
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	 
	//RCC->APB2ENR|=1<<3;    //使能PORTB时钟	 
	GPIOA->CRL&=0X00FFFFFF;	//PA6,7清除之前的设置
	GPIOA->CRL|=0XBB000000;	//复用功能输出 
//	GPIOB->CRL&=0XFFFFFF00;	//PB0,1清除之前的设置
//	GPIOB->CRL|=0X000000BB;	//复用功能输出 
	
	TIM3->ARR=arr;			//设定计数器自动重装值 
	TIM3->PSC=psc;			//预分频器设置
  
	TIM3->CCMR1|=7<<4;  	//CH1 PWM2模式		
	TIM3->CCMR1|=7<<12;	

	
	TIM3->CCMR1|=1<<3; 		//CH1预装载使能	 
	TIM3->CCMR1|=1<<11;

	
 	TIM3->CCER|=3<<0;   	//OC1 输出使能	   
	TIM3->CCER|=3<<4;

	
	
	TIM3->CR1 = 0x0080;   	//ARPE使能 
	TIM3->CR1|= 0x01;    	//使能定时器3										  
}  

//通道34
void TIM4_PWM_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 1<<2;	//TIM4时钟使能 
	RCC->APB2ENR |= 1<<3;    //使能PORTB时钟	 
	GPIOB->CRH &= 0XFFFFFF00;	//PB8,9清除之前的设置
	GPIOB->CRH |= 0X000000BB;	//复用功能输出 
	
	TIM4->ARR=arr;			//设定计数器自动重装值 
	TIM4->PSC=psc;			//预分频器设置
	
	TIM4->CCMR2|=7<<4;  	
	TIM4->CCMR2|=7<<12;	
	
	TIM4->CCMR2|=1<<3; 		
	TIM4->CCMR2|=1<<11;
	
	TIM4->CCER|=3<<8;
	TIM4->CCER|=3<<12;
	
	
	TIM4->CR1=0x0080;   	//ARPE使能 
	TIM4->CR1|=0x01;    	//使能定时器3	
}