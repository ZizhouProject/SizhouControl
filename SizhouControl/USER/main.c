#include "sys.h"
#include "delay.h"
#include "extern.h"
#include "mpu6050.h"
#include "mpu6050iic.h"
#include "Timer2CH1.h"
#include "ultrasonic.h"
#include "led.h"
#include "datatranslate.h"
#include "timer3.h"
#include "timer4.h"
#include "control.h"
#include "WaveFiltering.h"
#include "PID.h"
#include "nimingusart.h"
#include "debug.h"
#include "oled.h"

//PID_ROL,PID_PIT,PID_YAW 外环ROL PIT YAW
//PID_PID_1,PID_PID_2,PID_PID_3 内环 ROL(陀螺仪的X) PIT(陀螺仪的Y)YAW(陀螺仪的Z)
//PID_PID_4	超声波
PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2,PID_PID_3,PID_PID_4,PID_PID_5,PID_PID_6,PID_PID_7,PID_PID_8,PID_PID_9,PID_PID_10,PID_PID_11,PID_PID_12;
POSE Att_Angle;
SENSOR_FINAL Acc,Gyr;					//最后发往上位机的数据
SENSOR_USE acc,gyr;						//中间使用的数据
SENSOR_WAVEFLIT acc_wf,gyr_wf;			//滤波后的数据
OUT out;

u8 Data_Check,Send_Status,Send_Senser,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_PID4,Send_PID5,Send_PID6;
u8 data_to_send[50];

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;
int16_t Buf_AX[Buf_Num],Buf_AY[Buf_Num],Buf_AZ[Buf_Num], Buf_GX[Buf_Num],Buf_GY[Buf_Num],Buf_GZ[Buf_Num];

int start=0,flag_reporter=0,flag_control=0,flag_ult_count=0,flag_ult_start=0,flag_Acc=0,flag_Gyr=0;		
int16_t SA_X=0,SA_Y=0,SA_Z=0,SG_X=0,SG_Y=0,SG_Z=0,error_a_x=0,error_a_y=0,error_a_z=0,error_g_x=0,error_g_y=0,error_g_z=0;
float DT=0.004,DT_dis=1;
float set_pit=0,set_rol=0,set_yaw=0,set_distance=50;				//期望参数 pit rol yaw distance
float error_pitch=0,error_roll=0,error_yaw=0,error_X,error_Y,error_Z,last_error_X=0,last_error_Y=0,last_error_Z=0,error_dis=0,last_error_dis=0;				//last_error_pitch=0,last_error_roll=0,last_error_yaw=0,
int throttle=5850,Moto_PWM_5,Moto_PWM_6,Moto_PWM_7,Moto_PWM_8;	//初始油门
int16_t core_i,core_p,core_d;

unsigned int Acc_X_Mid = 0, Acc_Y_Mid = 0, Acc_Z_Mid = 0;
unsigned int Acc_X_Init = 0, Acc_Y_Init = 0, Acc_Z_Init = 0;
unsigned int Gro_X_Mid = 0, Gro_Y_Mid = 0, Gro_Z_Mid = 0;
unsigned int Gro_X_Init = 0, Gro_Y_Init = 0, Gro_Z_Init = 0;




int ult_distance;					//关于超声波的参数			距离
u8 flag_catch=0;				
u16 flag_val;

void throttle_stroke(void);			//函数声明
void data_change(void);
void count_ACC_GYR_bias(void);
void count_bias(void);
void data_prepare(void);

int main(void)
{
	
	
	/*-----------------测试程序所需要的变量-----------------*/
	#if DEBUG 															//						 *	
		int16_t my_Gyr[3];												//采集角度值		 *
		int16_t my_Acc[3];												//采集角速度值  *
	#endif																	//						 *
	/*-----------------测试程序所需要的变量-----------------*/
	
	
	/*-------------目前暂时还没有使用到的模块------------------*/
//	TIM2_Catch_Init(0XFFFF,71);					//以1Mhz的频率计数 	*
//	ultrasonic_init();									//超声波初始化				*
	/*-------------目前暂时还没有使用到的模块------------------*/
	
	uint16_t pwm1_mid = 0, pwm2_mid = 0, pwm3_mid = 0, pwm4_mid = 0;
	u32 ult_temp=0;
	Stm32_Clock_Init(9);						//初始化系统时钟
	delay_init(72);	   	 						//延时函数初始化
	Uart1_Init(115200);							//串口初始化波特率：115200
	LED_Init();		  								//LED初始化
	MPU_Init();											//MPU6050初始化
	OLED_Init();
	
	TIM3_PWM_Init(9999,17);					//四路PWM初始化 频率400hz
	TIM4_PWM_Init(9999,17);
	wavefilter_data_init();					//初始化陀螺仪的六个值为0
	pid_init();											//初始化PID参数为0	
	delay_ms(2000);
	/*-----------归一化陀螺仪的值--------*/
	while(!((flag_Acc>199)&&(flag_Gyr>199)))
	{
		count_ACC_GYR_bias();
	}
	count_bias();										//对归一化的值进行均值滤波
	throttle_stroke();							//电调的启动信号，必须使用！！！
	
	Moto_PWM_1 = 3300;				//1   //安全启动转速
	Moto_PWM_2 = 3300;				//4
	Moto_PWM_3 = 3300;				//2
	Moto_PWM_4 = 3300;				//3
	
	TIM1_Int_Init(39,7199);					//初始化一个定时器中断，4MS
	
	
	while(1)
	{
		#if  !DEBUG
		if(flag_ult_count>=250)					//1s测一次高
		{
			 flag_ult_count=0;
			 flag_ult_start=1;
			 ultrasonic_work();
			 distance_count(ult_temp);
		}
		
		
		if(flag_control>=1)					//每4ms一次
		{
			flag_control=0;
			data_prepare();
			control(start);
			data_change();
		}	                   
		
		if(flag_reporter>=3)					//12ms发送一次
		{
			flag_reporter=0;
			Data_Send_Status();
			Data_Send_Senser();
			Data_Send_MotoPWM();
			
			if(Send_PID1)						//PID_ROL.P/I/D		PID_PIT.P/I/D		PID_YAW.P/I/D
			{
				Send_PID1 = 0;
				Data_Send_PID1();
			}
			if(Send_PID2)						//PID_ALT.P/I/D    PID_POS.P/I/D	PID_PID_1.P/I/D
			{
				Send_PID2 = 0;					
				Data_Send_PID2();
			}
			if(Send_PID3)						//PID_PID_2.P/I/D		PID_PID_3.P/I/D	   PID_PID_4.P/I/D
			{
				Send_PID3 = 0;
				Data_Send_PID3();	
			}		
		}
		#endif
		
		#if DEBUG
			MPU_Get_Accelerometer(&my_Acc[0],&my_Acc[1],&my_Acc[2]);
			MPU_Get_Gyroscope(&my_Gyr[0],&my_Gyr[1],&my_Gyr[2]);
							
			if(flag_control>=1)					//每4ms一次
			{
				flag_control=0;
				data_prepare();
				pwm1_mid = Moto_PWM_1 + acc.err_x; if (pwm1_mid > 4500) pwm1_mid = 4500; else if (pwm1_mid < 2000) pwm1_mid = 2000;
				pwm2_mid = Moto_PWM_2 + acc.err_x; if (pwm2_mid > 4500) pwm2_mid = 4500; else if (pwm2_mid < 2000) pwm2_mid = 2000;
				pwm3_mid = Moto_PWM_3 - acc.err_x; if (pwm3_mid > 4500) pwm3_mid = 4500; else if (pwm3_mid < 2000) pwm3_mid = 2000;
				pwm4_mid = Moto_PWM_4 - acc.err_x; if (pwm4_mid > 4500) pwm4_mid = 4500; else if (pwm4_mid < 2000) pwm4_mid = 2000;
				
				Moto_PWM_1 = pwm1_mid;
				Moto_PWM_2 = pwm2_mid;
				Moto_PWM_3 = pwm3_mid;
				Moto_PWM_4 = pwm4_mid;
				
				
			}	  
			OLED_ShowString(40, 0, "Gyr:");			OLED_ShowString(80, 0, "Acc:");
		  OLED_ShowString(0, 1, "x:"); 	 		OLED_ShowNum(40,1, gyr.X, 5); 			OLED_ShowNum(80,1, acc.X, 5);							//x
			OLED_ShowString(0, 2, "y:"); 			OLED_ShowNum(40,2, gyr.Y, 5); 			OLED_ShowNum(80,2, acc.Y, 5);							//y
			OLED_ShowString(0, 3, "z:"); 			OLED_ShowNum(40,3, gyr.Z, 5); 			OLED_ShowNum(80,3, acc.Z, 5);							//z
			OLED_ShowString(0, 4, "x_int:"); 	OLED_ShowNum(40,4, Gro_X_Init, 5); 	OLED_ShowNum(80,4, Acc_X_Init, 5);
			OLED_ShowString(0, 5, "y_int:");	OLED_ShowNum(40,5, Gro_Y_Init, 5); 	OLED_ShowNum(80,5, Acc_Y_Init, 5);
			OLED_ShowString(0, 6, "z_int:");	OLED_ShowNum(40,6, Gro_Z_Init, 5); 	OLED_ShowNum(80,6, Acc_Z_Init, 5);
			//if (Acc[0] > 80000)
	
			//delay_ms(500);
		       
			printf("my_acc.x = %d\n", my_Acc[0]);
		
			
			
		#endif
	}
}

void throttle_stroke(void)
{
	Moto_PWM_1=8000;							//油门最高点   80%
	Moto_PWM_2=8000;
	Moto_PWM_3=8000;
	Moto_PWM_4=8000;
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	Moto_PWM_1=3000;							//油门最低点  30%
	Moto_PWM_2=3000;
	Moto_PWM_3=3000;
	Moto_PWM_4=3000;
	delay_ms(1000);
	delay_ms(1000);
	Moto_PWM_1=3000;	 						//准备起飞    30%
	Moto_PWM_2=3000;
	Moto_PWM_3=3000;
	Moto_PWM_4=3000;
	delay_ms(1000);
}

void data_change(void)							//数据格式转换
{
	Acc.X=(int)acc_wf.X;
	Acc.Y=(int)acc_wf.Y;
	Acc.Z=(int)acc_wf.Z;
	Gyr.X=(int)gyr.X;
	Gyr.Y=(int)gyr.Y;
	Gyr.Z=(int)gyr.Z;
}


void count_ACC_GYR_bias(void)
{
	MPU_Get_Accelerometer(&acc.X,&acc.Y,&acc.Z);
	MPU_Get_Gyroscope(&gyr.X,&gyr.Y,&gyr.Z);
	
	Acc_X_Mid += acc.X;		//SA_X+=acc.X;
	Acc_Y_Mid += acc.Y; 	//SA_Y+=acc.Y;
	Acc_Z_Mid += acc.Z;		//SA_Z+=acc.Z;
	
	Gro_X_Mid += gyr.X;		//SG_X+=gyr.X;
	Gro_Y_Mid += gyr.Y;   //SG_Y+=gyr.Y;
	Gro_Z_Mid += gyr.Z; 	//SG_Z+=gyr.Z;
	
	flag_Acc++;
	flag_Gyr++;
}

void count_bias(void)					//计算加计，陀螺仪三轴零偏
{
	Acc_X_Init = Acc_X_Mid / 200; //error_a_x=SA_X/200;
	Acc_Y_Init = Acc_Y_Mid / 200; //error_a_y=SA_Y/200;
	Acc_Z_Init = Acc_Z_Mid / 200; //error_a_z=SA_Z/200;
	
	Gro_X_Init = Gro_X_Mid / 200;	//error_g_x=SG_X/200;
	Gro_Y_Init = Gro_Y_Mid / 200;	//error_g_y=SG_Y/200;
	Gro_Z_Init = Gro_Z_Mid / 200;	//error_g_z=SG_Z/200;	
}

void data_prepare(void)
{
	unsigned int i = 0;
	u32 gyr_x_mid = 0, gyr_y_mid = 0, gyr_z_mid = 0;
	int16_t gyr_x_filter, gyr_y_filter, gyr_z_filter;
	u32 acc_x_mid = 0, acc_y_mid = 0, acc_z_mid = 0;
	int16_t acc_x_filter, acc_y_filter, acc_z_filter;
	for (i = 0; i < 10; i++)
	{
		MPU_Get_Gyroscope(&gyr.X,&gyr.Y,&gyr.Z);
		MPU_Get_Accelerometer(&acc.X,&acc.Y,&acc.Z);
		gyr_x_mid += gyr.X;
		gyr_y_mid += gyr.Y;
		gyr_z_mid += gyr.Z;
		acc_x_mid += acc.X;
		acc_y_mid += acc.Y;
		acc_z_mid += acc.Z;
	}
	
	gyr.X = gyr_x_mid / 10;
	gyr.Y = gyr_y_mid / 10;
	gyr.Z = gyr_z_mid / 10;
	
	acc.X = acc_x_mid / 10;
	acc.Y = acc_y_mid / 10;
	acc.Z = acc_z_mid / 10;
	
	acc.err_x = Gro_X_Init - acc.X ;
	
//	gyr.X=gyr.X-error_g_x+0.5;
//	gyr.Y=gyr.Y-error_g_y+0.5;
//	gyr.Z=gyr.Z-error_g_z+0.5;
//	acc.X-=error_a_x;
//	acc.Y-=error_a_y;
//	acc.Z-=error_a_z;
	 
	IMU_Prepare();
	sensfusion6_update(gyr_wf.X,gyr_wf.Y,gyr_wf.Z,acc_wf.X,acc_wf.Y,acc_wf.Z);
}

