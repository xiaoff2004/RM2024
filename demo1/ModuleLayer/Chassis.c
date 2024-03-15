#include "Chassis.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "remote.h"
#include "math.h"
#include "MathData.h"

//底盘设置(Mecanum 为1,Omni_4 为0 )
#define chassis_type 0

/**
 * \brief	yaw位置处理
 *			计算角速度
 */
void ChassisAngle_Solve(void)
{
	//经过处理后可认为枪口位置编码器返回值为0
	motor_info[CHASSIS_POSI].angle = (motor_info[YAW].angle-YAW_InitalAngle +8192)%8192;
	float wheel_speed_sum = -((float)motor_info[CHASSIS1].speed + (float)motor_info[CHASSIS2].speed
							+ (float)motor_info[CHASSIS3].speed + (float)motor_info[CHASSIS4].speed);
	motor_info[CHASSIS_POSI].speed = wheel_speed_sum / (2*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK)*60
													/(2*MECANUM_WHEEL_RADIUS * PI)*3591/187);
//	RemoteData.SpeedX = (-Motor_Info[Chassis1].Speed + Motor_Info[Chassis2].Speed + Motor_Info[Chassis3].Speed + Motor_Info[Chassis4].Speed)/4;
//	RemoteData.SpeedY = (Motor_Info[Chassis1].Speed + Motor_Info[Chassis2].Speed - Motor_Info[Chassis3].Speed - Motor_Info[Chassis4].Speed)/4;
}

/**
 * \brief 底盘运动解算
 * TODO:云台跟随解算不确定
 */
void Chassis_UpData(void)
{
	//判断底盘的状态（不转、转、底盘跟随）
	switch((uint8_t)RemoteData.chassis_status)
	{
	case CHASSIS_NO_SPIN:
		RemoteData.target_speed_w=0;
	case CHASSIS_SPIN:
		RemoteData.target_speed_w=TARGET_SPEED_W;

	case CHASSIS_FOLLOW:
		{
			motor_info[CHASSIS_POSI].target_angle=YAW_InitalAngle;
			Posi_IncrPID(&PosiPID_Info[CHASSIS_POSI], &IncrPID_Info[CHASSIS_POSI], &motor_info[CHASSIS_POSI]);
			RemoteData.target_speed_w=PosiPID_Info[CHASSIS_POSI].Output;
		}
	}

	//将世界坐标系下的速度转换成底盘坐标系下的速度
	ChassisAngle_Solve();
	float angle =(float)motor_info[CHASSIS_POSI].angle/8192.0f * 2.0f *PI;
	RemoteData.target_spee_x=RemoteData.target_spee_x*cosf(angle) +RemoteData.target_speed_y*sinf(angle);
	RemoteData.target_speed_y=RemoteData.target_speed_y*cosf(angle) -RemoteData.target_spee_x*sinf(angle);

	//麦轮底盘解算
    #if chassis_type
	{

		RemoteData.target_speed_w=RemoteData.target_speed_yaw * 4.0f;
		motor_info[CHASSIS1].target_speed = -(RemoteData.target_spee_x - RemoteData.target_speed_y + RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS2].target_speed = (RemoteData.target_spee_x + RemoteData.target_speed_y - RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS3].target_speed = (RemoteData.target_spee_x - RemoteData.target_speed_y - RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS4].target_speed = -(RemoteData.target_spee_x + RemoteData.target_speed_y + RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS1].target_speed *=0.1;//可以再精简一些
		motor_info[CHASSIS2].target_speed *=0.1;
		motor_info[CHASSIS3].target_speed *= 0.1;
		motor_info[CHASSIS4].target_speed *=0.1;
			}

    #else

		motor_info[CHASSIS1].target_speed = -(RemoteData.target_spee_x - RemoteData.target_speed_y + RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS2].target_speed = (RemoteData.target_spee_x + RemoteData.target_speed_y - RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS3].target_speed = (RemoteData.target_spee_x - RemoteData.target_speed_y - RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;
		motor_info[CHASSIS4].target_speed = -(RemoteData.target_spee_x + RemoteData.target_speed_y + RemoteData.target_speed_w *0.5*(MECANUM_WHEEL_BASE + MECANUM_WHEEL_TARCK))*60/(2*MECANUM_WHEEL_RADIUS *PI)*3591/187;

    #endif
}

