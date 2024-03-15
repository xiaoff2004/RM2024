#ifndef __MATHDATA_H_
#define __MATHDATA_H_

#include "main.h"

#define PI 3.14159265358979f
//麦轮底盘轴距（长）
#define MECANUM_WHEEL_BASE 0.30f
//麦轮底盘轮距（宽）
#define MECANUM_WHEEL_TARCK 0.33f
//麦轮半径
#define MECANUM_WHEEL_RADIUS 0.075f
//全向轮底盘半径
#define OMNI_RADIUS 0.30f
//全向轮半径
#define OMNI_WhEEL_RADIUS 0.075f
//小陀螺角速度
#define TARGET_SPEED_W 5.0f
//Yaw初始角度
#define YAW_InitalAngle 0
//Pitch初始角度
#define Pitch_InitalAngle 0
//摩擦轮速度
#define ShooterSpeed 16.0f
//倒弹速度
#define ReverseSpeed 0.0f
//通道值映射到底盘或Yaw轴、Pitch轴速度的比值
//遥控器映射值
#define ChassisSpeed_RemoteChange 0.006f
#define YawSpeed_RemoteChange		0.006f
#define PitchSpeed_RemoteChange		0.6f
//键盘映射值
#define ChassisSpeed_BoardChange 0.6f
#define YawSpeed_BoardChange		0.6f
#define PitchSpeed_BoardChange		0.6f




#endif
