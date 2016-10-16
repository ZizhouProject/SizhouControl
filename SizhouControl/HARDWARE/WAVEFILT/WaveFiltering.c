#include "WaveFiltering.h"

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMU_Prepare(void)													//滤波参数准备			FIR滤波
{
    u8 i = 0;
	float temp_AX = 0, temp_AY = 0, temp_AZ = 0,temp_GX = 0, temp_GY = 0, temp_GZ = 0;					//临时变量
	
	float Hn[Buf_Num] =											//滤波器抽头系数
	{
		-0.0048,  0.0000,  0.0155,  0.0186, -0.0152,
 		-0.0593, -0.0345,  0.1045,  0.2881,  0.3739,
 		 0.2881,  0.1045, -0.0345, -0.0593, -0.0152,
		 0.0186,  0.0155,  0.0000, -0.0048
	};

//加速度计滤波	
	Buf_AX[0] = acc.X;						//存放加速度计值
	Buf_AY[0] = acc.Y;
	Buf_AZ[0] = acc.Z;
	
	for(i = 0; i < Buf_Num; i++)
	{
		temp_AX += Buf_AX[i] * Hn[i];
		temp_AY += Buf_AY[i] * Hn[i];
		temp_AZ += Buf_AZ[i] * Hn[i];
	}
	
	acc_wf.X = temp_AX;
	acc_wf.Y = temp_AY;
	acc_wf.Z = temp_AZ;
	
	for(i = 0; i < Buf_Num -1; i++)
	{
		Buf_AX[Buf_Num-1-i] = Buf_AX[Buf_Num-2-i];
		Buf_AY[Buf_Num-1-i] = Buf_AY[Buf_Num-2-i];
		Buf_AZ[Buf_Num-1-i] = Buf_AZ[Buf_Num-2-i];
	}
	 
	gyr_wf.X=gyr.X * DS_PIS;					//度每秒转换为弧度每秒	
	gyr_wf.Y=gyr.Y * DS_PIS;
	gyr_wf.Z=gyr.Z * DS_PIS;
	
}

void sensfusion6_update(float gx, float gy, float gz, float ax, float ay, float az)					//姿态解算（使用滤波后的参数）
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa,qb,qc;
	float s1,s2,s3;
	
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))	
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az);
		ax = ax * recipNorm;
		ay = ay * recipNorm;
		az = az * recipNorm;
		
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
	
		if(twoKi > 0.0f)
		{		
			integralFBx += twoKi * halfex * dt;  		// integral error scaled by Ki				
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			
			gx += integralFBx; 							 // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
		  integralFBx = 0.0f; 								// prevent integral windup
		  integralFBy = 0.0f;
		  integralFBz = 0.0f;
		}

		gx += twoKp * halfex;						// Apply proportional feedback
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
		
	// Integrate rate of change of quaternion	
	gx *= (0.5f * dt);   												// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	
	qa = q0;
	qb = q1;
	qc = q2;
	
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	s1=2*(q1*q3 - q0*q2);
	s2= 2*(q0*q1 + q2*q3);
	s3=gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	if(s1>1)
		s1=1;
	if(s1<-1)
		s1=-1;
	
	Att_Angle.pit=asinf(s1)*PI_D;
	Att_Angle.rol=atan2f(s2,s3)*PI_D;
	Att_Angle.yaw= atan2(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3)  *PI_D;
	
	
//	Att_Angle.pit=PI_D*asinf(2 * q1 * q3 - 2 * q0 * q2);
//	Att_Angle.rol=PI_D*atan2f(2 * q0 * q1 + 2 * q2 * q3, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3); //2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1
//	Att_Angle.yaw=PI_D*atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1);//MPU_Data.GZ * 0.015267 * 0.004f;     
}

void wavefilter_data_init(void)								//三轴滤波数据初始化
{
	 acc_wf.X=0;
	 acc_wf.Y=0;
	 acc_wf.Z=0;
	 gyr_wf.X=0;
	 gyr_wf.Y=0;
	 gyr_wf.Z=0;
}

//void IMU_Prepare(void)													//滤波参数准备			FIR滤波
//{
//    u8 i = 0;
//	float temp_AX = 0, temp_AY = 0, temp_AZ = 0,temp_GX = 0, temp_GY = 0, temp_GZ = 0;					//临时变量
//	
//	float Hn[Buf_Num] =											//滤波器抽头系数
//	{
//		-0.0048,  0.0000,  0.0155,  0.0186, -0.0152,
// 		-0.0593, -0.0345,  0.1045,  0.2881,  0.3739,
// 		 0.2881,  0.1045, -0.0345, -0.0593, -0.0152,
//		 0.0186,  0.0155,  0.0000, -0.0048
//	};
//	
//	
//	
//	Buf_AX[0] = acc.X;						//存放加速度计值
//	Buf_AY[0] = acc.Y;
//	Buf_AZ[0] = acc.Z;
//	
//	for(i = 0; i < Buf_Num; i++)
//	{
//		temp_AX += Buf_AX[i] * Hn[i];
//		temp_AY += Buf_AY[i] * Hn[i];
//		temp_AZ += Buf_AZ[i] * Hn[i];
//	}
//	
//	acc_wf.X = temp_AX;
//	acc_wf.Y = temp_AY;
//	acc_wf.Z = temp_AZ;
//	
//	for(i = 0; i < Buf_Num -1; i++)
//	{
//		Buf_AX[Buf_Num-1-i] = Buf_AX[Buf_Num-2-i];
//		Buf_AY[Buf_Num-1-i] = Buf_AY[Buf_Num-2-i];
//		Buf_AZ[Buf_Num-1-i] = Buf_AZ[Buf_Num-2-i];
//	}
//	
//	for(i = 0; i < Buf_Num; i++)
//	{
//		temp_GX += Buf_GX[i] * Hn[i];
//		temp_GY += Buf_GY[i] * Hn[i];
//		temp_GZ += Buf_GZ[i] * Hn[i];
//	}
//	
//	acc_wf.X = temp_AX;
//	acc_wf.Y = temp_AY;
//	acc_wf.Z = temp_AZ;
//	
//	for(i = 0; i < Buf_Num -1; i++)
//	{
//		Buf_AX[Buf_Num-1-i] = Buf_AX[Buf_Num-2-i];
//		Buf_AY[Buf_Num-1-i] = Buf_AY[Buf_Num-2-i];
//		Buf_AZ[Buf_Num-1-i] = Buf_AZ[Buf_Num-2-i];
//	}
//	
//	gyr_wf.X=gyr.X * D_PI;					//度每秒转弧度每秒
//	gyr_wf.Y=gyr.Y * D_PI;
//	gyr_wf.Z=gyr.Z * D_PI;
//}





