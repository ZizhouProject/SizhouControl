#include "ultrasonic.h"

void ultrasonic_init(void)
{
	RCC->APB2ENR|=1<<2;    				//使能PORTA时钟
	GPIOA->CRL&=0XFFFFFF0F; 
	GPIOA->CRL|=0X00000030;			//PA1推挽输出
	GPIOA->ODR|=1<<1;      				//PA1 输出低
}

void ultrasonic_work(void)
{
	ULT=1;
	delay_us(15);
	ULT=0;
}

void distance_count(u32 ult_temp)
{
	if( flag_catch&0X80)								//成功捕捉到一次高电平
		{
			ult_temp= flag_catch&0X3F;
			ult_temp*=65536;						//溢出时间总和
			ult_temp+= flag_val;					//得到总的高电平时间 ，单位us
			ult_distance=ult_temp*0.017+1;			//化简后的公式 	实际公式：（ult_temp/1000）/1000*340/2*100
 			flag_catch=0;								//开启下一次捕获
 		}
	
}
