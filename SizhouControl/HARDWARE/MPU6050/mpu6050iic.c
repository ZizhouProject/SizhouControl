#include "mpu6050iic.h"


//MPU IIC 延时函数
void MPU_IIC_Delay(void)
{
	delay_us(2);
}

//初始化IIC接口
void MPU_IIC_Init(void)
{					     
 	RCC->APB2ENR|=1<<2;					//使能 PA	  2scl, 3data		 
	GPIOA->CRL&=0XFFFF00FF;			//PA2/3 
	GPIOA->CRL|=0X00003300;	  
	GPIOA->ODR|=3<<2;     				//PA2/3输出高   
}

//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();  						   //SDA线输出
	MPU_IIC_SDA=1;	  	  
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
 	MPU_IIC_SDA=0;							//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;							//钳住IIC总线，准备发送或接受数据
}	  

//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();							//SDA线输出
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;							//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL=1;  
	MPU_IIC_SDA=1;							//发送IIC总线结束信号
	MPU_IIC_Delay();							   	
}

//等待应答信号的到来			返回值：接收应答成功（0）；接收应答失败（1）
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      							//SDA设置为输入
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();	   
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;								//始终输出为0
	return 0;  
} 

//产生ACK应答
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}

//不产生ACK应答
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}		

//IIC发送一个字节 返回（从机有无应答）：有应答（1）；无应答（0）
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;										//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		MPU_IIC_SCL=1;
		MPU_IIC_Delay(); 
		MPU_IIC_SCL=0;	
		MPU_IIC_Delay();
    }	 
} 	    

//读取一个字节 ack=1时，发送ACK；ack=0时，发送NACK
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();								//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL=0; 
        MPU_IIC_Delay();
		MPU_IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();						//发送NACK
    else
        MPU_IIC_Ack(); 						//发送ACK
    return receive;
}
