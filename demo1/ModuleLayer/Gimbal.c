#include "Gimbal.h"
#include "Motor.h"
#include "imu.h"
#include "Pid.h"
#include "Remote.h"
#include "MathData.h"

/**
 * \brief 云台位姿初始化
 */
void Gimbal_Init(void)
{
    My_CAN_Init();
    PID_Init();
	motor_info[YAW].target_angle=YAW_InitalAngle;
	motor_info[PITCH].target_angle=Pitch_InitalAngle;
	Posi_IncrPID(&PosiPID_Info[YAW], &IncrPID_Info[YAW], &motor_info[YAW]);
	Posi_IncrPID(&PosiPID_Info[PITCH], &IncrPID_Info[PITCH], &motor_info[PITCH]);
	Send_GimbalConf(PosiPID_Info[YAW].Output,PosiPID_Info[PITCH].Output);
    HAL_Delay(10);
    BMI088_Init();
}

/**
 * \brief 云台PID计算
 */
void Gimbal_UpData(void)
{
	motor_info[YAW].target_angle=imu_data.angle[0]*8192-1;
	motor_info[YAW].target_angle+=RemoteData.target_speed_yaw;

	motor_info[PITCH].target_angle+=RemoteData.target_speed_pitch;

	Posi_IncrPID(&PosiPID_Info[YAW], &IncrPID_Info[YAW], &motor_info[YAW]);
	Posi_IncrPID(&PosiPID_Info[PITCH], &IncrPID_Info[PITCH], &motor_info[PITCH]);
}
