#ifndef __DATATRANSLATE_H
#define __DATATRANSLATE_H

#include "extern.h"
#include "mpu6050.h"
#include "mpu6050iic.h"
#include "nimingusart.h"
#include "timer3.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))			
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

extern u8 Data_Check,Send_Status,Send_Senser,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_PID4,Send_PID5,Send_PID6;
extern u8 data_to_send[50];

extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2,PID_PID_3,PID_PID_4,PID_PID_5,PID_PID_6,PID_PID_7,PID_PID_8,PID_PID_9,PID_PID_10,PID_PID_11,PID_PID_12;
extern POSE Att_Angle;							//最后发往上位机的数据
extern SENSOR_USE acc,gyr;						//中间使用的数据
extern SENSOR_FINAL Acc,Gyr;					//最后发往上位机的数据
extern SENSOR_USE acc,gyr;						//中间使用的数据
extern SENSOR_WAVEFLIT acc_wf,gyr_wf;			//滤波后的数据
extern OUT out;

extern int start;
extern int throttle,Moto_PWM_5,Moto_PWM_6,Moto_PWM_7,Moto_PWM_8;

void Data_Receive_Anl(u8 *data_buf,u8 num);  			//下位机解析上位机发送的数据函数
void Data_Send_Status(void);							//发送姿态
void Data_Send_PID1(void);								//发送PID1
void Data_Send_PID2(void);								//发送PID2
void Data_Send_PID3(void);								//发送PID3
void Data_Send_Senser(void);							//发送传感器数值
void Data_Send_MotoPWM(void);							//发送PWM波				
void Data_Send_Check(u16 check);						//数据校验

#endif
