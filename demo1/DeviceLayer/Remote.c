#include "Remote.h"

#include "main.h"
#include "MathData.h"




uint8_t Remote_RecieveDate[18]={0};
remote_data_t RemoteData={0};


 void RemoteInit(void)
 {
	RemoteData.remote_or_board=REMOTE;

 }

/**
 * \brief 解析遥控接收数据
 */
void RemoteData_Unpack(void)
{
	remote_t Temp={0};
	Temp. ch0 = ((Remote_RecieveDate[1] << 8) | Remote_RecieveDate[0]) & 0x7ff;
	Temp. ch0 -=1024;
	Temp. ch1 = ((Remote_RecieveDate[2] << 5) | (Remote_RecieveDate[1] >> 3)) & 0x7ff;
	Temp. ch1 -=1024;
	Temp. ch2 = ((Remote_RecieveDate[4] << 10) | (Remote_RecieveDate[3] << 2) | (Remote_RecieveDate[2] >> 6)) & 0x7ff;
	Temp. ch2 -=1024;
	Temp. ch3 = ((Remote_RecieveDate[5] << 7) | (Remote_RecieveDate[4] >> 1)) & 0x7ff;
	Temp. ch3 -=1024;
	Temp. wheel = (Remote_RecieveDate[17] << 8) | (Remote_RecieveDate[16]);
	Temp. wheel -=1024;

	Temp. s1 = (Remote_RecieveDate[5] >> 4) & 0x0003;
	Temp. s2 =((Remote_RecieveDate[5] >> 4) & 0x000c) >> 2;

	Temp. x = (Remote_RecieveDate[7] << 8) | Remote_RecieveDate[6];
	Temp. y = (Remote_RecieveDate[9] << 8) | Remote_RecieveDate[8];
	Temp. z = (Remote_RecieveDate[11] << 8) | Remote_RecieveDate[10];
	Temp. l = Remote_RecieveDate[12];
	Temp. r = Remote_RecieveDate[13];

	uint16_t button = (Remote_RecieveDate[15] << 8) | Remote_RecieveDate[14];
	Temp. w = button & 0x80;
	Temp. s = button & 0x40;
	Temp. a = button & 0x20;
	Temp. d = button & 0x10;
	Temp. q = button & 0x08;
	Temp. e = button & 0x84;
	Temp. shirt = button & 0x02;
	Temp. ctrl = button & 0x01;


	if(RemoteData.remote_or_board== REMOTE)
	{
		RemoteData.target_spee_x =Temp.ch3 * ChassisSpeed_RemoteChange;
		RemoteData.target_speed_y =Temp.ch2 * ChassisSpeed_RemoteChange;
		RemoteData.target_speed_yaw=Temp.ch0 * YawSpeed_RemoteChange;
		RemoteData.target_speed_pitch=Temp.ch1 *PitchSpeed_RemoteChange;
	}
	else if(RemoteData.remote_or_board==BOARD)
	{
		RemoteData.target_spee_x =(Temp.w -Temp.s) * ChassisSpeed_BoardChange;
		RemoteData.target_speed_y =(Temp.a -Temp.d) * ChassisSpeed_BoardChange;
		if(Temp.l)
		{
			RemoteData.target_speed_yaw=Temp.x * -1 * YawSpeed_BoardChange;
			RemoteData.target_speed_pitch=Temp.y * -1 * PitchSpeed_BoardChange;
		}
		else
		{
			RemoteData.target_speed_yaw=0;
			RemoteData.target_speed_pitch=0;
		}
	}
	RemoteData.chassis_status=Temp.s1;

	RemoteData.fire_type=Temp.s2;

	if(Temp.wheel<60)
		RemoteData.shoot_strategy=NO_SHOOT;
	else if(Temp.wheel>660)
		RemoteData.shoot_strategy=MULTI_SHOOT;
	else
		RemoteData.shoot_strategy=SINGLE_SHOOT;

}
