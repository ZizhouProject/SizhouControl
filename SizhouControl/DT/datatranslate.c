#include "datatranslate.h"
#include "nimingusart.h"

extern int16_t core_i,core_p,core_d;

//以下为下位机解析上位机发送数据的函数
void Data_Receive_Anl(u8 *data_buf,u8 num)
{															
	u8 sum = 0;
	u8 i=0;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头

	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
//			MPU6050_CalOff_Acc();				//加速度计矫正
			start =1;
		if(*(data_buf+4)==0X02)
//			MPU6050_CalOff_Gyr();				//陀螺仪矫正
			start =0;
//		if(*(data_buf+4)==0X03)
//		{MPU6050_CalOff_Acc();MPU6050_CalOff_Gyr();}
		if(*(data_buf+4)==0X04)
//			Cal_Compass();						//磁力计矫正
			throttle+=20;								//油门减20
		if(*(data_buf+4)==0X05)
//			MS5611_CalOffset();					//气压计矫正
			throttle-=5;								//油门加5
	}
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)					//读取飞控
		{
			Send_PID1 = 1;
			Send_PID2 = 1;
			Send_PID3 = 1;
			Send_PID4 = 1;
			Send_PID5 = 1;
			Send_PID6 = 1;
		}
		if(*(data_buf+4)==0X02)
			Send_Offset = 1;
		}																//PID三个参数范围：P:/10;I:/1000;D:/1000
	if(*(data_buf+2)==0X10)												//	PID1(1~3)
	{
			PID_ROL.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;		//外环ROLL
			PID_ROL.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_ROL.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100000;
			PID_PIT.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;		//外环PITCH
			PID_PIT.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_PIT.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100000;
			PID_YAW.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/1000;		//外环YAW
			PID_YAW.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			PID_YAW.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100000;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X11)								//PID2(4~6)
	{
			PID_ALT.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;		//ALT
			PID_ALT.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_ALT.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100000;
			PID_POS.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;		//POS
			PID_POS.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_POS.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100000;
			PID_PID_1.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/1000;	//内环PITCH
			PID_PID_1.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			PID_PID_1.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100000;
			Data_Send_Check(sum);
	}
		if(*(data_buf+2)==0X12)								//PID3(7~9)
	{
			PID_PID_2.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;		//内环ROLL
			PID_PID_2.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_PID_2.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100000;
			PID_PID_3.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;	//内环YAW
			PID_PID_3.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_PID_3.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100000;
			PID_PID_4.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/1000;	//没用到
			PID_PID_4.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			PID_PID_4.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100000;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X13)								//PID4(10-12)				//没用到
	{
			PID_PID_5.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			PID_PID_5.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_PID_5.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100000;
			PID_PID_6.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;
			PID_PID_6.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_PID_6.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100000;
			PID_PID_7.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/1000;
			PID_PID_7.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			PID_PID_7.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100000;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X14)								//PID5(13-15)	没用到
	{
			PID_PID_8.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			PID_PID_8.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_PID_8.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100000;
			PID_PID_9.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;
			PID_PID_9.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_PID_9.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100000;
			PID_PID_10.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/1000;
			PID_PID_10.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			PID_PID_10.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100000;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X15)								//PID6(16-17)	没用到
	{
			PID_PID_11.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			PID_PID_11.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10000;
			PID_PID_11.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100000;
			PID_PID_12.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;
			PID_PID_12.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/10000;
			PID_PID_12.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100000;
			Data_Send_Check(sum);
	}
//	if(*(data_buf+2)==0X16)								//OFFSET
//	{
//		AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
//		AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;	
//	}
}

//以下为下位机发往上位机的函数
void Data_Send_Senser(void)								//发送传感器数据
{																   
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(core_p);  				 //高8位
	data_to_send[_cnt++]=BYTE0(core_p);  				 //低8位
	data_to_send[_cnt++]=BYTE1(core_i);  				 //高8位
	data_to_send[_cnt++]=BYTE0(core_i);  				 //低8位
	data_to_send[_cnt++]=BYTE1(core_d);  				 //高8位
	data_to_send[_cnt++]=BYTE0(core_d);  				 //低8位
//	data_to_send[_cnt++]=BYTE1(Acc.X);  				 //高8位
//	data_to_send[_cnt++]=BYTE0(Acc.X);  				 //低8位
//	data_to_send[_cnt++]=BYTE1(Acc.Y);
//	data_to_send[_cnt++]=BYTE0(Acc.Y);
//	data_to_send[_cnt++]=BYTE1(Acc.Z);
//	data_to_send[_cnt++]=BYTE0(Acc.Z);
	data_to_send[_cnt++]=BYTE1(Gyr.X);
	data_to_send[_cnt++]=BYTE0(Gyr.X);
	data_to_send[_cnt++]=BYTE1(Gyr.Y);
	data_to_send[_cnt++]=BYTE0(Gyr.Y);
	data_to_send[_cnt++]=BYTE1(Gyr.Z);
	data_to_send[_cnt++]=BYTE0(Gyr.Z);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	Uart1_Put_Buf(data_to_send,_cnt);
}

void Data_Send_Status(void)							//发送基本信息：姿态		
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(Att_Angle.rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Att_Angle.pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Att_Angle.yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
		
	data_to_send[_cnt++]=0xA1;
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	Uart1_Put_Buf(data_to_send,_cnt);
}

////发送PWM波
void Data_Send_MotoPWM(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Moto_PWM_1);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_1);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_2);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_2);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_3);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_3);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_4);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_4);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_5);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_5);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_6);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_6);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_7);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_7);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_8);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_8);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Put_Buf(data_to_send,_cnt);
}

void Data_Send_PID1(void)				//发送PID1
{																		
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	_temp = PID_ROL.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ROL.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ROL.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PIT.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PIT.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PIT.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_YAW.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_YAW.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_YAW.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Put_Buf(data_to_send,_cnt);
}

void Data_Send_PID2(void)										//发送PID2
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;

	_temp = PID_ALT.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ALT.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_ALT.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_POS.P * 100;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	_temp = PID_POS.I * 100;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	_temp = PID_POS.D * 100;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	_temp = PID_PID_1.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_1.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_1.D * 100;

	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Put_Buf(data_to_send,_cnt);
}

void Data_Send_PID3(void)								//发送PID3
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x12;
	data_to_send[_cnt++]=0;
	
	_temp = PID_PID_2.P * 100;									
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_2.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_2.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_3.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_3.I * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_3.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_4.P * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_4.I * 100;
 	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID_PID_4.D * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Put_Buf(data_to_send,_cnt);
}

void Data_Send_Check(u16 check)						//数据校验
{																			
	u8 sum = 0;
	u8 i=0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);

	for(i=0;i<7;i++)
		sum += data_to_send[i];
	
	data_to_send[7]=sum;

	Uart1_Put_Buf(data_to_send,8);
}





