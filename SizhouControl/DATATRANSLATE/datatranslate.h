#ifndef __DATATRANSLATE_H
#define __DATATRANSLATE_H

#include "extern.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "usart.h"

//BYTE3、BYTE2、BYTE1、BYTE0：表示取某变量的高低字节，0最低，3最高。
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))			//用于拆分高字节与低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

extern u8 Data_Check,Send_Status,Send_Senser,Send_Offset,Send_PID1,Send_PID2,Send_PID3;
extern u8 data_to_send[50];

extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2,PID_PID_3,PID_PID_4;
extern POSE Att_Angle;
extern SENSOR Acc,Gyr;
extern int start;

void Data_Receive_Anl(u8 *data_buf,u8 num);  //下位机解析上位机发送数据的函数
void Data_Send_Status();							//发送基本信息（姿态、锁定状态）
void Data_Send_PID1();								//发送PID1
void Data_Send_PID2();								//发送PID2
void Data_Send_PID3();								//发送PID3
void Data_Send_Check(u16 check);					//发送校验数据
void Data_Send_Senser();							//发送传感器数据

#endif
