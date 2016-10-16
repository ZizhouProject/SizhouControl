#include "PID.h"
extern int16_t core_i,core_p,core_d;

/************外环PID*********/
//P
void Get_Shell_Pitch_P(void)												//PITCH  P		
{
	out.shell_pitch_p=error_pitch*PID_PIT.P;
}

void Get_Shell_Roll_P(void)												//ROLL P
{
	out.shell_roll_p=error_roll*PID_ROL.P;
}

void Get_Shell_Yaw_P(void)												//YAW  P
{
	out.shell_yaw_p=error_yaw*PID_YAW.P;
}

//I
void Get_Shell_Pitch_I(void)											//PITCH I
{
	if(PID_PIT.I != 0)
	{	
		out.shell_pitch_i+=error_pitch*PID_PIT.I*DT;		
		if(out.shell_pitch_i > 350)									//Limit integration
		{
			out.shell_pitch_i=350;
		}
		if(out.shell_pitch_i <-350)
		{
			out.shell_pitch_i=-350;
		}
	}
	else
		out.shell_pitch_i=0;
}

void Get_Shell_Roll_I(void)												//ROLL I
{
	if(PID_ROL.I != 0)
	{
		out.shell_roll_i+=error_roll*PID_ROL.I*DT;	
		if(out.shell_roll_i > 350)							
		{
			out.shell_roll_i=350;
		}
		if(out.shell_roll_i <-350)
		{
			out.shell_roll_i=-350;
		}		
	}
	else
		out.shell_roll_i=0;
}

void Get_Shell_Yaw_I(void)														//YAW I
{
	if(PID_YAW.I != 0)
	{
		out.shell_yaw_i+=error_yaw*PID_YAW.I*DT;		
		if(out.shell_yaw_i > 350)								
		{
			out.shell_yaw_i=350;
		}
		if(out.shell_yaw_i <-350)
		{
			out.shell_yaw_i=-350;
		}
	}
	else
		out.shell_yaw_i=0;
}

//D
void Get_Shell_Pitch_D(void)
{
	//out.shell_pitch_d=(error_pitch-last_error_pitch)/DT*PID_PIT.D;
	//last_error_pitch=error_pitch;
	out.shell_pitch_d=-gyr.Y*PID_PIT.D;	
}

void Get_Shell_Roll_D(void)
{
	//out.shell_roll_d=(error_roll-last_error_roll)/DT*PID_ROL.D;
	//last_error_roll=error_roll;
	out.shell_roll_d=gyr.X*PID_ROL.D;
}

void Get_Shell_Yaw_D(void)
{
	//out.shell_yaw_d=(error_yaw-last_error_yaw)/DT*PID_YAW.D;
	//last_error_yaw=error_yaw;
	out.shell_yaw_d=gyr.Z*PID_YAW.D;
}
/*********************************/

/*********外环总输出量*******/
void Get_Pitch_Shell(void)									//Pitch的外环输出，作为内环Y的输入
{
	out.shell_pitch=out.shell_pitch_p+out.shell_pitch_i+out.shell_pitch_d;
}

void Get_Roll_Shell(void)									//Roll的外环输出，作为内环Y的输入
{
	//out. shell_roll=out.shell_roll_p+out.shell_pitch_i-out.shell_roll_d;
	out.shell_roll=out.shell_roll_p+out.shell_roll_i+out.shell_roll_d;
}

void Get_Yaw_Shell(void)									//Yaw的外环输出，作为内环Z的输入
{
	out.shell_yaw=out.shell_yaw_p+out.shell_yaw_i+out.shell_yaw_d;
}
/*************************/


/************内环PID*********/
//P
void Get_Core_X_P(void)										//陀螺仪X轴(ROLL)的P				PID_PID_1
{
	error_X=out.shell_roll-gyr.X;
	out.core_roll_p=error_X*PID_PID_1.P;
	core_p=(int)out.core_roll_p;
}

void Get_Core_Y_P(void)										//陀螺仪Y轴(PITCH)的P				PID_PID_2
{
//	error_Y=out.shell_pitch-gyr.Y;
		error_Y=out.shell_pitch+gyr.Y;
		out.core_pitch_p=error_Y*PID_PID_2.P;
//		core_p=(int)out.core_pitch_p;
}

void Get_Core_Z_P(void)										//陀螺仪Z轴(YAW)的P					PID_PID_3
{
	error_Z=out.shell_yaw-gyr.Z;
	out.core_yaw_p=error_Z*PID_PID_3.P;
}

//I
void Get_Core_X_I(void)										//陀螺仪X轴(ROLL)的I
{
	if(PID_PID_1.I !=0)
	{
		out.core_roll_i+=error_X*PID_PID_1.I*DT;
		if(out.core_roll_i>350)												//Limit integration
		{
			out.core_roll_i=350;
		}
		if(out.core_roll_i<-350)
		{
			out.core_roll_i=-350;
		}
	}
	else
		out.core_roll_i=0;
	
	core_i=(int)out.core_roll_i;
}

void Get_Core_Y_I(void)										//陀螺仪Y轴(PITCH)的I
{
	if(PID_PID_2.I != 0)
	{
		out.core_pitch_i+=error_Y*PID_PID_2.I*DT;
		if(out.core_pitch_i>350)												//Limit integration
		{
			out.core_pitch_i=350;
		}
		if(out.core_pitch_i<-350)
		{
			out.core_pitch_i=-350;
		}
	}
	else
		out.core_pitch_i=0;
	
//	core_i=(int)out.core_pitch_i;
}

void Get_Core_Z_I(void)											//陀螺仪Z轴(YAW)的I
{
	if(PID_PID_3.I !=0)
	{
		out.core_yaw_i+=error_Z*PID_PID_3.I*DT;
		if(out.core_yaw_i>350)												//Limit integration
		{
			out.core_yaw_i=350;
		}
		if(out.core_yaw_i<-350)
		{
			out.core_yaw_i=-350;
		}
	}
	else
		out.core_yaw_i=0;
}

//D
void Get_Core_X_D(void)
{
	out.core_roll_d=(error_X-last_error_X)*PID_PID_1.D/DT;
	last_error_X=error_X;
	core_d=(int)out.core_roll_d;
}

void Get_Core_Y_D(void)
{
	out.core_pitch_d=(error_Y-last_error_Y)*PID_PID_2.D/DT;
	last_error_Y=error_Y;
//	core_d=(int)out.core_pitch_d;
}

void Get_Core_Z_D(void)
{
	out.core_yaw_d=(error_Z-last_error_Z)*PID_PID_3.D/DT;
	last_error_Z=error_Z;
}
/****************************/

/*********内环总输出量*******/								//内环总输出量控制电机
void Get_Pitch_Core(void)									//内环跟Y轴有关	
{
	out.core_pitch=out.core_pitch_p+out.core_pitch_i+out.core_pitch_d;
}

void Get_Roll_Core(void)									//内环跟X轴有关
{
	out.core_roll=out.core_roll_p+out.core_roll_i+out.core_roll_d;
}

void Get_Yaw_Core(void)										//内环跟Z轴有关	
{
	out.core_yaw=out.core_yaw_p+out.core_yaw_i+out.core_yaw_d;
}

/*********超声波PID输出******/
void Get_Ult_P(void)										//超声波P
{
	out.ult_p=error_dis*PID_PID_4.P;
}

void Get_Ult_I(void)										//超声波I
{
	if(PID_PID_4.I != 0)
	{
		out.ult_i+=error_dis*PID_PID_4.I*DT_dis;
		if(out.ult_i>350)												//Limit integration		7%
		{
			out.ult_i=350;
		}
		if(out.ult_i<-350)
		{
			out.ult_i=-350;
		}
	}
	else
		out.ult_i=0;
}

void Get_Ult_d(void)
{
	out.ult_d=(error_dis-last_error_dis)*PID_PID_4.D/DT_dis;
	last_error_dis=error_dis;
}

void Get_Ult(void)
{
	out.ult=out.ult_p+out.ult_i+out.ult_d;
}

//PID初始化
void pid_init(void)								
{
	PID_PIT.P=0;
	PID_PIT.I=0;
	PID_PIT.D=0;
	PID_ROL.P=0;
	PID_ROL.I=0;
	PID_ROL.D=0;
	PID_YAW.P=0;
	PID_YAW.I=0;
	PID_YAW.D=0;
	PID_PID_1.P=0;						//外化ROLL，陀螺仪X
	PID_PID_1.I=0;
	PID_PID_1.D=0;
	PID_PID_2.P=0;						//外环PITCH,陀螺仪Y
	PID_PID_2.I=0;
	PID_PID_2.D=0;
	PID_PID_3.P=0;						//外环YAW,陀螺仪Z
	PID_PID_3.I=0;
	PID_PID_3.D=0;
}
