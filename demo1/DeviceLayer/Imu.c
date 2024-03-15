#include "imu.h"
#include "main.h"
#include "spi.h"
#include "math.h"
#include "MahonyAHRS.h"
#include "MathData.h"

imu_data_t imu_data;

/**
 * \brief Gyro写数据
 * \param addr 地址
 * \param data 数据
 */
void Gyro_WriteByte(uint8_t addr,uint8_t data)
{
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
	uint8_t TxAddr= (addr & 0x7f);
	HAL_SPI_Transmit(&hspi1, &TxAddr, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
}

/**
 * @brief Gyro读数据
 * @param addr 地址
 * @param pbuff 读出的数据
 * @param len 要读数据的长度
 */
void Gyro_ReadBytes(uint8_t addr,uint8_t *pbuff,uint8_t len)
{
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
	uint8_t temp = (addr | 0x80);
	HAL_SPI_Transmit(&hspi1, &temp, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)== HAL_SPI_STATE_BUSY_RX);
	for(uint8_t i=0;i<len;i++)
	{
		HAL_SPI_Receive(&hspi1, &temp, 1, 1000);
		while(HAL_SPI_GetState(&hspi1)== HAL_SPI_STATE_BUSY_RX);
		pbuff[i]=temp;
	}
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
}

/**
 * \brief Accel写数据
 * \param addr 地址
 * \param data 数据
 */
void Accel_WriteByte(uint8_t addr,uint8_t data)
{
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
	uint8_t TxData = (addr & 0x7f) ;
	HAL_SPI_Transmit(&hspi1, &TxData, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
}

/**
 * \brief Accel读数据（在真正读数据前要读一个无效数据）
 * \param addr 地址
 * \param pbuff 读出的数据
 * \param len 要读数据的长度
 */
void Accel_ReadBytes(uint8_t addr,uint8_t *pbuff,uint8_t len)
{
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
	uint8_t temp= (addr | 0x80);
	HAL_SPI_Transmit(&hspi1, &temp, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)== HAL_SPI_STATE_BUSY_RX);
	HAL_SPI_Receive(&hspi1, &temp, 1, 1000);
	while(HAL_SPI_GetState(&hspi1)== HAL_SPI_STATE_BUSY_RX);
	for(uint8_t i=0;i<len;i++)
	{
		HAL_SPI_Receive(&hspi1, &temp, 1, 1000);
		while(HAL_SPI_GetState(&hspi1)== HAL_SPI_STATE_BUSY_RX);
		pbuff[i]=temp;
	}
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
}

/**
 * \brief BMI088初始化
 */
void BMI088_Init(void)
{
	Accel_WriteByte(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);//软重启
	HAL_Delay(50);
	Accel_WriteByte(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON); //打开加速度计电源
	Accel_WriteByte(ACC_RANGE_ADDR, ACC_RANGE_3G);
	Accel_WriteByte(ACC_CONF_ADDR,ACC_CONF_RESERVED<<7 |ACC_CONF_BWP_OSR4<<6 |ACC_CONF_ODR_200_Hz );

	Gyro_WriteByte(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);//软重启
	HAL_Delay(50);
	Gyro_WriteByte(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);//陀螺仪变成正常模式
	Gyro_WriteByte(GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
	Gyro_WriteByte(GYRO_BANDWIDTH_ADDR, GYRO_ODR_200Hz_BANDWIDTH_64Hz);

	imu_data.angle_q[0]=1;
}

/**
 * \brief BMI088读取数据
 * \param data 读取的数据
 */
void Get_ImuData(imu_data_t *data)
{
	uint8_t buff_acc[ACC_XYZ_LEN],buff_gyro[GYRO_XYZ_LEN],buff_temp[TEMP_LEN];


	int16_t Accel[3],Gyro[3];

	Accel_ReadBytes(ACC_X_LSB_ADDR,buff_acc ,ACC_XYZ_LEN );
	Accel[0]=((int16_t)buff_acc[1] << 8) + (int16_t)buff_acc[0];
	Accel[1]=((int16_t)buff_acc[3] << 8) + (int16_t)buff_acc[2];
	Accel[2]=((int16_t)buff_acc[5] << 8) + (int16_t)buff_acc[4];

	data->accel[0]=(float)Accel[0]*BMI088_ACCEL_3G_SEN;
	data->accel[1]=(float)Accel[1]*BMI088_ACCEL_3G_SEN;
	data->accel[2]=(float)Accel[2]*BMI088_ACCEL_3G_SEN;


	Gyro_ReadBytes(GYRO_RATE_X_LSB_ADDR, buff_gyro, GYRO_XYZ_LEN);
	Gyro[0]=((int16_t)buff_gyro[1] << 8) + (int16_t)buff_gyro[0];
	Gyro[1]=((int16_t)buff_gyro[3] << 8) + (int16_t)buff_gyro[2];
	Gyro[2]=((int16_t)buff_gyro[5] << 8) + (int16_t)buff_gyro[4];
	data->gyro[0]=(float)Gyro[0] / 65.536f * DEG2SEC;
	data->gyro[1]=(float)Gyro[1] / 65.536f * DEG2SEC;
	data->gyro[2]=(float)Gyro[2] / 65.536f * DEG2SEC;


	Accel_ReadBytes(TEMP_MSB_ADDR, buff_temp, TEMP_LEN);
	uint16_t Temp=(buff_temp[0]<<3) +(buff_temp[1]>>5);
	int16_t Tempr=0;
	if(Temp >1023)
		Tempr = (int16_t)Temp -2048;
	else
		Tempr = (int16_t)Temp;
	data->temperature = Tempr * TEMP_UNIT +TEMP_BIAS;
    //TODO:去零漂
    imu_data.gyro[1] -= (11.5390333f / 65.536) * (PI / 180);
    imu_data.gyro[2] -= (22.4231017f / 65.536) * (PI / 180);

//    data->accel[1] -= (11.5390333f / 65.536)*(PI/180);
//    data->accel[2] -= (10.4231017f / 65.536)*(PI/180);
//    data->gyro[1] -= (141.763613f * 0.0008974);

	MahonyAHRSupdateIMU(data->angle_q, data->gyro[0], data->gyro[1], data->gyro[2], data->accel[0], data->accel[1], data->accel[2]);
	data->angle[0] = atan2f(2.0f*(data->angle_q[0]*data->angle_q[3]+data->angle_q[1]*data->angle_q[2]), 2.0f*(data->angle_q[0]*data->angle_q[0]+data->angle_q[1]*data->angle_q[1])-1.0f);
	data->angle[1] = asinf(-2.0f*(data->angle_q[1]*data->angle_q[3]-data->angle_q[0]*data->angle_q[2]));
	data->angle[2] = atan2f(2.0f*(data->angle_q[0]*data->angle_q[1]+data->angle_q[2]*data->angle_q[3]),2.0f*(data->angle_q[0]*data->angle_q[0]+data->angle_q[3]*data->angle_q[3])-1.0f);

    //TODO:定义坐标系
}


