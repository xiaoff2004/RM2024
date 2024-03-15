#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "main.h"

typedef struct
{
	int16_t ch0;		//遥控器通道
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	uint8_t s1;			//开关位（1~3）
	uint8_t s2;
	int16_t wheel;		//左上方拨轮

	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t l;
	uint8_t r;

	uint8_t w;
	uint8_t s;
	uint8_t a;
	uint8_t d;
	uint8_t q;
	uint8_t e;
	uint8_t shirt;
	uint8_t ctrl;
}remote_t;

//底盘状态（不转，底盘跟随，小陀螺）
typedef enum
{
	CHASSIS_NO_SPIN=1,
	CHASSIS_FOLLOW,
	CHASSIS_SPIN
}chassis_status_e;

//瞄准策略（无，手瞄，自瞄）
typedef enum
{
	NO_FIRE=1,
	FREE_FIRE,
	AUTO_FIRE
}fire_type_e;

//开火策略（无，倒弹，单发，多发）
typedef enum
{
	NO_SHOOT,
	REVERSE,//反转（倒弹）
	SINGLE_SHOOT,
	MULTI_SHOOT
}shoot_strategy_e;

//遥控方式（遥控器，键盘）
typedef enum
{
	REMOTE,
	BOARD
}remote_or_board_e;


//遥控解算后的数据
typedef struct
{
    uint8_t data_size;//数据大小

	int16_t target_spee_x; //x的目标速度
	int16_t target_speed_y;//y的目标速度
	int16_t target_speed_w;//w的目标速度
	int16_t target_speed_yaw;//yaw的目标速度
	int16_t target_speed_pitch;//pitch的目标速度
	int16_t target_angle;//底盘旋转的角度

	chassis_status_e chassis_status;//底盘状态
	fire_type_e fire_type;//瞄准策略
	shoot_strategy_e shoot_strategy;//开火策略
	remote_or_board_e remote_or_board;//遥控方式


}remote_data_t;


extern remote_data_t RemoteData;
extern uint8_t Remote_RecieveDate[18];

void RemoteInit(void);
void RemoteData_Unpack(void);

#endif
