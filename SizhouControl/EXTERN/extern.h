#ifndef __EXTERN_H
#define __EXTERN_H

#include "stdint.h"

#define Buf_Num  			19								//数组长度
#define PI_D       			57.2957795f						//弧度转度，180/PI
#define D_PI       			0.01745329f						//度转弧度
//#define Acc_G 			0.0005981f 						//加速度计量程+-2g 1（65536/（4*g））=0.00059814453
//#define Gyro_G 			0.0610351f						//陀螺仪量程+-2000dps 1（65536/4000）=0.06103515625f
#define DS_PIS       		0.00106422f						//度每秒转弧度每秒
#define twoKp				0.8f   							 //0.1f 		//0.8f		//2.0f	// 2 * proportional gain
#define twoKi 				0.002f 							 //0.001f 	//0.002f	//0.008			// 2 * integral gain
#define dt 	      			0.004f								


typedef struct{											//PID参数
							float P;
							float I;
							float D;
					   }PID;
					   
typedef struct{											//姿态角
							float rol;
							float pit;
							float yaw;
					   }POSE;
					   
typedef struct{											//传感器三轴读数中间使用
							int16_t X;
							int16_t Y;
							int16_t Z;		
							int16_t err_x;
							int16_t err_y;
							int16_t err_z;
					   }SENSOR_USE;

typedef struct{											//滤波后的三轴读数
							float X;
							float Y;
							float Z;		
					   }SENSOR_WAVEFLIT;

typedef struct{											//传感器三轴读数最后发往上位机
							int16_t X;
							int16_t Y;
							int16_t Z;		
					   }SENSOR_FINAL;

typedef struct{
					float shell_pitch_p;
					float shell_pitch_i;
					float shell_pitch_d;
					float shell_roll_p;
					float shell_roll_i;
					float shell_roll_d;
					float shell_yaw_p;
					float shell_yaw_i;
					float shell_yaw_d;
					float shell_pitch;
					float shell_roll;
					float shell_yaw;
	
					float core_pitch_p;
					float core_pitch_i;
					float core_pitch_d;	
					float core_roll_p;
					float core_roll_i;
					float core_roll_d;
					float core_yaw_p;
					float core_yaw_i;
					float core_yaw_d;
					float core_pitch;
					float core_roll;
					float core_yaw;
					
					float ult_p;
					float ult_i;
					float ult_d;
					float ult;
					
				}OUT;
					   


#endif
				