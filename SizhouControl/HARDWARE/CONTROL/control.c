#include "control.h"

void control(int arm)
{
	if(arm&&(((Att_Angle.pit<60)&&(-60<Att_Angle.pit)) && ((Att_Angle.rol<60)&&(-60<Att_Angle.rol))))
	{
		error_pitch=set_pit-Att_Angle.pit;
		error_roll=set_rol-Att_Angle.rol;
		error_yaw=set_yaw-Att_Angle.yaw;
//		error_pitch=Att_Angle.pit-set_pit;
//		error_roll=Att_Angle.rol-set_rol;
//		error_yaw=Att_Angle.yaw-set_yaw;
//		if(flag_ult_start>=1)
//		{
//			flag_ult_start=0;
//			error_dis=ult_distance-set_dis;
//			Get_Ult_P();
//			Get_Ult_I();
//			Get_Ult_d();
//			Get_Ult();
//		}
		
		Get_Shell_Pitch_P();					//PITCH 外环
		Get_Shell_Pitch_I();	
		Get_Shell_Pitch_D();	
		Get_Pitch_Shell();
		
		Get_Shell_Roll_P();						//ROLL 外环
		Get_Shell_Roll_I();
		Get_Shell_Roll_P();
		Get_Roll_Shell();
		
		Get_Shell_Yaw_P();						//YAW 外环
		Get_Shell_Yaw_I();
		Get_Shell_Yaw_D();
		Get_Yaw_Shell();
		
		Get_Core_Y_P();							//内环
		Get_Core_Y_I();
		Get_Core_Y_D();
		Get_Pitch_Core();
		
		Get_Core_X_P();
		Get_Core_X_I();
		Get_Core_X_D();
		Get_Roll_Core();
		
		Get_Core_Z_P();
		Get_Core_Z_I();
		Get_Core_Z_D();
		Get_Yaw_Core();
			
//		Moto_PWM_1=throttle-out.core_pitch;//-out.core_yaw+out.ult;十字
//		Moto_PWM_2=throttle-out.core_roll;//+out.core_yaw+out.ult;
//		Moto_PWM_3=throttle+out.core_pitch;//-out.core_yawl+out.ult;
//		Moto_PWM_4=throttle+out.core_roll;//+out.core_yaw+out.ult;

		Moto_PWM_1=(int)(throttle+out.core_pitch-out.core_roll);//-out.core_yaw+out.ult;X字
		Moto_PWM_2=(int)(throttle-out.core_pitch-out.core_roll);//+out.core_yaw+out.ult;
		Moto_PWM_3=(int)(throttle-out.core_pitch+out.core_roll);//-out.core_yaw+out.ult;
		Moto_PWM_4=(int)(throttle+out.core_pitch+out.core_roll);//+out.core_yaw+out.ult;
	}
	else
	{
		out.core_roll_i=0;
		out.core_pitch_i=0;
		out.core_yaw_i=0;
		out.shell_pitch_i=0;
		out.shell_roll_i=0;
		out.shell_yaw_i=0;
		out.ult_i=0;
		Moto_PWM_1=7000;					
		Moto_PWM_2=7000;
		Moto_PWM_3=7000;
		Moto_PWM_4=7000;
	}
}