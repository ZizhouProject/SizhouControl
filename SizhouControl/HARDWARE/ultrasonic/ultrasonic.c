#include "ultrasonic.h"

void ultrasonic_init(void)
{
	RCC->APB2ENR|=1<<2;    				//ʹ��PORTAʱ��
	GPIOA->CRL&=0XFFFFFF0F; 
	GPIOA->CRL|=0X00000030;			//PA1�������
	GPIOA->ODR|=1<<1;      				//PA1 �����
}

void ultrasonic_work(void)
{
	ULT=1;
	delay_us(15);
	ULT=0;
}

void distance_count(u32 ult_temp)
{
	if( flag_catch&0X80)								//�ɹ���׽��һ�θߵ�ƽ
		{
			ult_temp= flag_catch&0X3F;
			ult_temp*=65536;						//���ʱ���ܺ�
			ult_temp+= flag_val;					//�õ��ܵĸߵ�ƽʱ�� ����λus
			ult_distance=ult_temp*0.017+1;			//�����Ĺ�ʽ 	ʵ�ʹ�ʽ����ult_temp/1000��/1000*340/2*100
 			flag_catch=0;								//������һ�β���
 		}
	
}
