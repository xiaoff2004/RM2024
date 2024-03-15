#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

typedef struct
{
	uint16_t target_angle;
	uint16_t angle;
	int16_t target_speed;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
}motor_info_t;

typedef enum
{
	CHASSIS1=0,//back:201     to:200
	CHASSIS2,//back:202     to:200
	CHASSIS3,//back:203     to:200
	CHASSIS4,//back:204     to:200

	FRICT_L,  	//摩擦轮back:205 to:1FF
	FRICT_R,	//back:206 to:1FF
	AMMO,		//拨弹轮back:207 to:1FF

	CHASSIS_POSI,//

	YAW,		//为了方便解算，将yaw电机设为8，前面空的一个编号为底盘角度环控制服务  back:209 to:2FF
	PITCH																		//back:20A to:2FF
}motor_no_e;

typedef enum
{
	CHASSIS,
	GIMBAL,
	FRICT
}MotorTXHeader_No_e;

//发送报文标识符
#define CHASSIS_TXID 	0x200			//chassis1,chassis2,chsaais3,chassis4
#define GIMBAL_TXID		0x2FF			//Yaw,Pitch
#define FRICT_TXID		0x1FF		//Frict_L,Frict_R,Ammo

//TODO:存在后面的can接收不到反馈信息的可能，实测再解决
////反馈接收报文标识符
///can1
//#define Chassis1_RXID 	0x201
//#define Chassis2_RXID 	0x202
//#define Chassis3_RXID 	0x203
//#define Chassis4_RXID 	0x204
///can2
//#define Frict_L_RXID  	0x205			//摩擦轮
//#define Frict_R_RXID  	0x206
//#define Ammo_RXID			0x207
//#define Yaw_RXID			0x209
//#define Pitch_RXID		0x20A

extern motor_info_t motor_info[10];
extern CAN_TxHeaderTypeDef motor_tx_header[3];


void My_CAN_Init(void);
void Send_ChasissInfo(int16_t Chasiss1,int16_t Chasiss,int16_t Chasiss3,int16_t Chasiss4);
void Send_GimbalConf(int16_t Yaw,int16_t Pitch);
void Send_AmmoSpeed(int16_t Frict_L,int16_t Frict_R,int16_t Ammo);
void Motor_RecieveData(motor_info_t *Motor_Info,uint8_t *Data);


#endif
