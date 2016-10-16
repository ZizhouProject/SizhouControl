#ifndef __CONTROL_H
#define __CONTROL_H

#include "timer3.h"
#include "PID.h"
#include "extern.h"

extern int flag_ult_start;
extern int ult_distance;	
extern float DT,DT_dis;
extern float set_pit,set_rol,set_yaw,set_dis;
extern float error_pitch,error_roll,error_yaw,error_X,error_Y,error_Z,last_error_X,last_error_Y,last_error_Z,error_dis,last_error_dis;//last_error_pitch,last_error_roll,last_error_yaw
extern int throttle;
extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2,PID_PID_3,PID_PID_4,PID_PID_5,PID_PID_6,PID_PID_7,PID_PID_8,PID_PID_9,PID_PID_10,PID_PID_11,PID_PID_12;
extern POSE Att_Angle;							//最后发往上位机的数据
extern SENSOR_FINAL Acc,Gyr;					//最后发往上位机的数据
extern SENSOR_USE acc,gyr;						//中间使用的数据
extern SENSOR_WAVEFLIT acc_wf,gyr_wf;		
extern OUT out;


void control(int arm);

#endif



