/*
 * @file		pid.c/h
 * @brief		pid相关程序
 * @brief		GM6020   	MaxOutput: 	30000
 * 							MaxSpeed:	320rpm(No load)
 * 										132rpm(Loading)
 *
 *				M3508(19:1)		MaxOutput: 	16384
 *							MaxSpeed:	482rpm(No load)
 * 										469rpm(Loading)
 *
 *
 * @history
 *
 *
 *
 */


#include "Pid.h"
#include "main.h"
#include "math.h"
#include "Motor.h"

sPosiPID_Info PosiPID_Info[10];
sIncrPID_Info IncrPID_Info[10];

//Chassis1=0,Chassis2,Chassis3,Chassis4,frict_L,frict_R,Ammo,Chassis_Posi,Yaw,Pitch

float PidInfo[2][10][5]={							//PosiPID{Kp,Ki,Kd,MaxSum,MaxOutput}
		{	{0},									//chasiss1
			{0},									//chasiss2
			{0},									//chasiss3
			{0},									//chasiss4

			{0},									//frict_L
			{0},									//frict_R
			{0},									//Ammo

			{0.1f ,0 ,0 ,5000 ,28000},			//Chassis_Posi

			{10.0f ,0 ,0 ,28000 ,28000},			//Yaw
			{0}},									//Pitch

													//IncrPID{Kp,Ki,Kd,MaxOutput,0}
		{	{2.0f ,0.003f ,0 ,15000 ,0},			//chasiss1
			{2.0f ,0.003f ,0 ,15000 ,0},			//chasiss2
			{2.0f ,0.003f ,0 ,15000 ,0},			//chasiss3
			{2.0f ,0.003f ,0 ,15000 ,0},			//chasiss4

			{0},									//frict_L
			{0},									//frict_R
			{10.0f ,2.0f ,0 ,1000 ,0},			//Ammo

			{0},									//Chassis_Posi

			{1.0f ,5.0f ,0 ,28000 ,0},			//Yaw
			{0}}									//Pitch
};

/**
 * \brief PID限幅
 * \param Value 当前值
 * \param MaxValue 最大值
 * \return 限幅后的输出值
 */
float PIDInfo_Limit(float Value,float MaxValue)
{
	if(fabs(Value) > MaxValue)
	{
		if(Value >= 0)
			Value = MaxValue;
		else
			Value = -MaxValue;
	}

	return Value;
}

/**
 * \brief PID信息初始化
 */
void PID_Init(void)
{
	for(uint8_t i=0;i<10;i++)
	{
		PosiPID_Info[i].Kp		=PidInfo[0][i][0];
		PosiPID_Info[i].Ki		=PidInfo[0][i][1];
		PosiPID_Info[i].Kd		=PidInfo[0][i][2];
		PosiPID_Info[i].MaxSum	=PidInfo[0][i][3];
		PosiPID_Info[i].MaxOutput=PidInfo[0][i][4];
		PosiPID_Info[i].Err		=0;
		PosiPID_Info[i].Sum  	=0;
		PosiPID_Info[i].Output	=0;

		IncrPID_Info[i].Kp		=PidInfo[1][i][0];
		IncrPID_Info[i].Ki		=PidInfo[1][i][1];
		IncrPID_Info[i].Kd		=PidInfo[1][i][2];
		IncrPID_Info[i].MaxOutput=PidInfo[1][i][3];
		IncrPID_Info[i].Err		=0;
		IncrPID_Info[i].LastErr	=0;
		IncrPID_Info[i].Output	=0;
	}
}

/**
 * \brief				位置式PID
 * \param PosiPID_Info	位置式PID结构体
 * \param MotorInfo		电机信息结构体
 */
void PosiPID(sPosiPID_Info *PosiPID_Info,motor_info_t *MotorInfo)
{
	PosiPID_Info->LastErr=PosiPID_Info->Err;
	PosiPID_Info->Err=MotorInfo->target_angle -MotorInfo->angle;
	if(PosiPID_Info->Err > 4096)
		PosiPID_Info->Err -= 8192;
	else if(PosiPID_Info->Err < -4096)
		PosiPID_Info->Err += 8192;
	PosiPID_Info->Sum +=PosiPID_Info->Err;
	PosiPID_Info->Sum =PIDInfo_Limit(PosiPID_Info->Sum, PosiPID_Info->MaxSum);
	PosiPID_Info->Output =PosiPID_Info->Kp * PosiPID_Info->Err
						+PosiPID_Info->Ki *	PosiPID_Info->Sum
						+PosiPID_Info->Kd *	(PosiPID_Info->Err -PosiPID_Info->LastErr);
	PosiPID_Info->Output =PIDInfo_Limit(PosiPID_Info->Output, PosiPID_Info->MaxOutput);
}

/**
 * \brief				增量式PID
 * \param IncrPID_Info	增量式PID结构体
 * \param MotorInfo		电机信息结构体
 */
void IncrPID(sIncrPID_Info *IncrPID_Info,motor_info_t *MotorInfo)
{
	IncrPID_Info->LastLastErr =IncrPID_Info->LastErr ;
	IncrPID_Info->LastErr =IncrPID_Info ->Err;
	IncrPID_Info->Err =MotorInfo->target_speed -MotorInfo ->speed;
	IncrPID_Info->Output +=IncrPID_Info->Kp *(IncrPID_Info->Err -IncrPID_Info ->LastErr)
					 +IncrPID_Info->Ki *IncrPID_Info->Err
					 +IncrPID_Info->Kd *(IncrPID_Info->Err -2 * IncrPID_Info->LastErr +IncrPID_Info->LastLastErr);
	IncrPID_Info->Output =PIDInfo_Limit(IncrPID_Info->Output, IncrPID_Info->MaxOutput);
}

/**
 * \brief				位置-速度环串级PID
 *						当位置式PID结构体为NULL时，为增量式PID
 * \param PosiPID_Info	位置式PID结构体
 * \param IncrPID_Info	增量式PID结构体
 * \param MotorInfo		电机信息结构体
 */
void Posi_IncrPID(sPosiPID_Info *PosiPID_Info,sIncrPID_Info *IncrPID_Info,motor_info_t *MotorInfo)
{
	IncrPID_Info->LastLastErr =IncrPID_Info->LastErr ;
	IncrPID_Info->LastErr =IncrPID_Info ->Err;
	if(PosiPID_Info==NULL)
	{
		IncrPID_Info->Err =MotorInfo->target_speed -MotorInfo ->speed;
	}
	else
	{
		PosiPID_Info->LastErr=PosiPID_Info->Err;
		PosiPID_Info->Err=MotorInfo->target_angle -MotorInfo->angle;
		if(PosiPID_Info->Err > 4096)
			PosiPID_Info->Err -= 8192;
		else if(PosiPID_Info->Err < -4096)
			PosiPID_Info->Err += 8192;
		PosiPID_Info->Sum +=PosiPID_Info->Err;
		PosiPID_Info->Sum =PIDInfo_Limit(PosiPID_Info->Sum, PosiPID_Info->MaxSum);
		PosiPID_Info->Output =PosiPID_Info->Kp * PosiPID_Info->Err
							+PosiPID_Info->Ki *	PosiPID_Info->Sum
							+PosiPID_Info->Kd *	(PosiPID_Info->Err -PosiPID_Info->LastErr);
		PosiPID_Info->Output =PIDInfo_Limit(PosiPID_Info->Output, PosiPID_Info->MaxOutput);

		IncrPID_Info->Err =PosiPID_Info->Output -MotorInfo ->speed;
	}
	IncrPID_Info->Output +=IncrPID_Info->Kp *(IncrPID_Info->Err -IncrPID_Info ->LastErr)
					 +IncrPID_Info->Ki *IncrPID_Info->Err
					 +IncrPID_Info->Kd *(IncrPID_Info->Err -2 * IncrPID_Info->LastErr +IncrPID_Info->LastLastErr);
	IncrPID_Info->Output =PIDInfo_Limit(IncrPID_Info->Output, IncrPID_Info->MaxOutput);
}

/**
 * \brief	PID信息清零
 *			防车车发疯
 */
void PID_Clear(void)
{
	for(uint8_t i =0;i<10;i++)
	{
		IncrPID_Info[i].Kp		=0;
		IncrPID_Info[i].Ki		=0;
		IncrPID_Info[i].Kd		=0;
		IncrPID_Info[i].MaxOutput=0;
		IncrPID_Info[i].Err		=0;
		IncrPID_Info[i].LastErr	=0;
		IncrPID_Info[i].Output	=0;
	}
}
