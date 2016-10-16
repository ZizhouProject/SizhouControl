#ifndef __PID_H
#define __PID_H

#include "extern.h"

extern float DT,DT_dis;
extern float set_pit,set_rol,set_yaw,set_distance;
extern float error_pitch,error_roll,error_yaw,error_X,error_Y,error_Z,last_error_X,last_error_Y,last_error_Z,error_dis,last_error_dis;//last_error_pitch,last_error_roll,last_error_yaw
extern int throttle;
extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2,PID_PID_3,PID_PID_4,PID_PID_5,PID_PID_6,PID_PID_7,PID_PID_8,PID_PID_9,PID_PID_10,PID_PID_11,PID_PID_12;
extern POSE Att_Angle;							//最后发往上位机的数据
extern SENSOR_FINAL Acc,Gyr;					//最后发往上位机的数据
extern SENSOR_USE acc,gyr;						//中间使用的数据
extern SENSOR_WAVEFLIT acc_wf,gyr_wf;			//滤波后的数据
extern OUT out;


void Get_Shell_Pitch_P(void);																
void Get_Shell_Roll_P(void);											
void Get_Shell_Yaw_P(void);												
void Get_Shell_Pitch_I(void);												
void Get_Shell_Roll_I(void);												
void Get_Shell_Yaw_I(void);												
void Get_Shell_Pitch_D(void);											
void Get_Shell_Roll_D(void);											
void Get_Shell_Yaw_D(void);	

void Get_Core_X_P(void);										
void Get_Core_Y_P(void);	
void Get_Core_Z_P(void);	
void Get_Core_X_I(void);	
void Get_Core_Y_I(void);	
void Get_Core_Z_I(void);	
void Get_Core_X_D(void);	
void Get_Core_Y_D(void);	
void Get_Core_Z_D(void);					

void Get_Pitch_Shell(void);								
void Get_Roll_Shell(void);
void Get_Yaw_Shell(void);	

void Get_Pitch_Core(void);									
void Get_Roll_Core(void);
void Get_Yaw_Core(void);	

void Get_Ult_P(void);
void Get_Ult_I(void);
void Get_Ult_d(void);
void Get_Ult(void);
	
void pid_init(void);


#endif
