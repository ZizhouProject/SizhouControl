#ifndef __WAVEFILTERING_H
#define __WAVEFILTERING_H

#include "math.h"
#include "stdint.h"
#include "sys.h"
#include "extern.h"

//PID_ROL,PID_PIT,PID_YAW			外环ROL PIT YAW
//PID_PID_1,PID_PID_2,PID_PID_3    内环 ROL PIT YAW
extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2,PID_PID_3,PID_PID_4,PID_PID_5,PID_PID_6,PID_PID_7,PID_PID_8,PID_PID_9,PID_PID_10,PID_PID_11,PID_PID_12;
extern POSE Att_Angle;
extern SENSOR_FINAL Acc,Gyr;							//最后发往上位机的数据
extern SENSOR_USE acc,gyr;								//中间使用的数据
extern SENSOR_WAVEFLIT acc_wf,gyr_wf;			//滤波后的数据

extern float q0, q1,q2,q3;
extern float integralFBx, integralFBy, integralFBz;
extern int16_t Buf_AX[Buf_Num],Buf_AY[Buf_Num],Buf_AZ[Buf_Num], Buf_GX[Buf_Num],Buf_GY[Buf_Num],Buf_GZ[Buf_Num];

float invSqrt(float x);
void sensfusion6_update(float gx, float gy, float gz, float ax, float ay, float az);				//姿态解算
void IMU_Prepare(void);													//滤波参数准备
void wavefilter_data_init(void);											//三轴滤波数据初始化

#endif