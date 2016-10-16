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
extern POSE Att_Angle;							//�������λ��������
extern SENSOR_USE acc,gyr;						//�м�ʹ�õ�����
extern SENSOR_FINAL Acc,Gyr;					//�������λ��������
extern SENSOR_USE acc,gyr;						//�м�ʹ�õ�����
extern SENSOR_WAVEFLIT acc_wf,gyr_wf;			//�˲��������
extern OUT out;

extern int start;
extern int throttle,Moto_PWM_5,Moto_PWM_6,Moto_PWM_7,Moto_PWM_8;

void Data_Receive_Anl(u8 *data_buf,u8 num);  			//��λ��������λ�����͵����ݺ���
void Data_Send_Status(void);							//������̬
void Data_Send_PID1(void);								//����PID1
void Data_Send_PID2(void);								//����PID2
void Data_Send_PID3(void);								//����PID3
void Data_Send_Senser(void);							//���ʹ�������ֵ
void Data_Send_MotoPWM(void);							//����PWM��				
void Data_Send_Check(u16 check);						//����У��

#endif
