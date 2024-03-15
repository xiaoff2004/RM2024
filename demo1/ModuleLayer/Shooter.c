#include "Shooter.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "remote.h"
#include "MathData.h"

/**
 * \brief 摩擦轮速度初始化
 */
void Shooter_Init(void)
{
	motor_info[FRICT_L].target_speed = ShooterSpeed;
	motor_info[FRICT_R].target_speed = -ShooterSpeed;
}

/**
 * \brief 摩擦轮+拨弹轮PID计算
 */



