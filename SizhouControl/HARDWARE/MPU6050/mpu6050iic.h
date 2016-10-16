#ifndef __MPU6050IIC_H
#define __MPU6050IIC_H

#include "sys.h"
#include "delay.h"

//IO方向设置
#define MPU_SDA_IN()  {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=8<<12;}
#define MPU_SDA_OUT() {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=3<<12;}

//IO操作函数
#define MPU_IIC_SCL    PAout(2) 		//SCL
#define MPU_IIC_SDA    PAout(3) 		//SDA	 
#define MPU_READ_SDA   PAin(3) 			//输入SDA

void MPU_IIC_Delay(void);					//MPU IIC延时函数
void MPU_IIC_Init(void);  						//MPU IIC初始化函数
void MPU_IIC_Start(void);					//MPU IIC 发送IIC开始信号
void MPU_IIC_Stop(void);						//MPU IIC 发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//MPU 	IIC  发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);		//MPU IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 				//MPU IIC等待ACK应答
void MPU_IIC_Ack(void);						//MPU IIC 发送ACK信号
void MPU_IIC_NAck(void);					//MPU IIC 不发送ACK信号

//void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);			//这两个函数干嘛的？找不到实体
//u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif