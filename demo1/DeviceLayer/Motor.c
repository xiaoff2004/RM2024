#include "Motor.h"

#include "main.h"
#include "can.h"

motor_info_t motor_info[10];
CAN_TxHeaderTypeDef motor_tx_header[3];

/**
 * \brief 设置CAN基本信息
 */
void Motor_TXInfo_Init(void)
{
	for(uint8_t i=0; i<3; i++)
	{
		motor_tx_header[i].ExtId = 0x00;
		motor_tx_header[i].IDE = CAN_ID_STD;
		motor_tx_header[i].RTR = CAN_RTR_DATA;
		motor_tx_header[i].DLC = 8;
	}
	motor_tx_header[CHASSIS].StdId=CHASSIS_TXID;
	motor_tx_header[GIMBAL].StdId=GIMBAL_TXID;
	motor_tx_header[FRICT].StdId=FRICT_TXID;
}

/**
 * \brief 自定义CAN初始化
 */
void My_CAN_Init(void)
{
	CanConfig_Init();
	Motor_TXInfo_Init();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

/**
 * \brief 发生chassis的电流参数（范围：-16384 ~ 16384）
 * \param Chasiss1 Chasiss1的电流参数（范围：-16384 ~ 16384）
 * \param Chasiss2 Chasiss2的电流参数（范围：-16384 ~ 16384）
 * \param Chasiss3 Chasiss3的电流参数（范围：-16384 ~ 16384）
 * \param Chasiss4 Chasiss4的电流参数（范围：-16384 ~ 16384）
 */
void Send_ChasissInfo(int16_t Chasiss1,int16_t Chasiss2,int16_t Chasiss3,int16_t Chasiss4)
{
	uint8_t Data[8]={0};
	Data[0]=Chasiss1>>8;
	Data[1]=Chasiss1;
	Data[2]=Chasiss2>>8;
	Data[3]=Chasiss2;
	Data[4]=Chasiss3>>8;
	Data[5]=Chasiss3;
	Data[6]=Chasiss4>>8;
	Data[7]=Chasiss4;
	HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header[CHASSIS], Data, (uint32_t *)CAN_TX_MAILBOX0);
}

/**
 * \brief 发送Gimbal的电压参数（范围：-30000 ~ 30000）
 * \param Yaw Yaw的电压参数（范围：-30000 ~ 30000）
 * \param Pitch Pitch的电压参数（范围：-30000 ~ 30000）
 */
void Send_GimbalConf(int16_t Yaw,int16_t Pitch)
{
	uint8_t Data[8]={0};
	Data[0]=Yaw>>8;
	Data[1]=Yaw;
	Data[2]=Pitch>>8;
	Data[3]=Pitch;
	HAL_CAN_AddTxMessage(&hcan2, &motor_tx_header[GIMBAL], Data, (uint32_t *)CAN_TX_MAILBOX0);
}

/**
 * \brief
 * \param Frict_L 左摩擦轮的电流参数（范围：-16384 ~ 16384）
 * \param Frict_R 右摩擦轮的电流参数（范围：-16384 ~ 16384）
 * \param Ammo 拨弹轮的电流参数（范围：-10000 ~ 10000）
 */
void Send_AmmoSpeed(int16_t Frict_L,int16_t Frict_R,int16_t Ammo)
{
	uint8_t Data[8]={0};
	Data[0]=Frict_L>>8;
	Data[1]=Frict_L;
	Data[2]=Frict_R>>8;
	Data[3]=Frict_R;
	Data[4]=Ammo>>8;
	Data[5]=Ammo;
	HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header[FRICT], Data, (uint32_t *)CAN_TX_MAILBOX0);
}

/**
 * \brief 解析CAN反馈信息
 * \param motor_info 某个电机的数据
 * \param Data CAN反馈信息
 */
void Motor_RecieveData(motor_info_t *motor_info,uint8_t *Data)
{
	motor_info->angle = (Data[0]<<8)|Data[1];
	motor_info->speed = (Data[2]<<8)|Data[3];
	motor_info->current = (Data[4]<<8)|Data[5];
	motor_info->temperature =Data[6];
}



