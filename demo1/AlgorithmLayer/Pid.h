#ifndef __PID_H__
#define __PID_H__

#include "Motor.h"


typedef struct
{
	float Kp,Ki,Kd;
	float Err,LastErr;
	float Sum,MaxSum;
	float Output,MaxOutput;
}sPosiPID_Info;

typedef struct
{
	float Kp,Ki,Kd;
	float Err,LastErr,LastLastErr;
	float Output,MaxOutput;
}sIncrPID_Info;


extern sPosiPID_Info PosiPID_Info[10];
extern sIncrPID_Info IncrPID_Info[10];

void PID_Init(void);
void PosiPID(sPosiPID_Info *PosiPID_Info,motor_info_t *MotorInfo);
void IncrPID(sIncrPID_Info *IncrPID_Info,motor_info_t *MotorInfo);
void Posi_IncrPID(sPosiPID_Info *PosiPID_Info,sIncrPID_Info *IncrPID_Info,motor_info_t *MotorInfo);
void PID_Clear(void);

#endif
