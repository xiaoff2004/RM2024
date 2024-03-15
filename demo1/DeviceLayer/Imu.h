#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

#define imu_spihandle 			hspi1
#define CS1_Accel_Pin 			GPIO_PIN_4
#define CS1_Accel_GPIO_Port 	GPIOA
#define CS1_Gyro_Pin 			GPIO_PIN_0
#define CS1_Gyro_GPIO_Port 		GPIOB
#define IST8310_Reset_Pin 		GPIO_PIN_6
#define IST8310_Reset_GPIO_Port GPIOG
#define IST8310_Ready_Pin 		GPIO_PIN_3
#define IST8310_Ready_GPIO_Port GPIOG

/*-----加速度计寄存器表-----*/
#define ACC_CHIP_ID_ADDR 0x00
#define ACC_CHIP_ID_VAL 0x1E

#define ACC_ERR_REG_ADDR 0x02

#define ACC_STATUS_ADDR 0x03

#define ACC_X_LSB_ADDR 0x12
#define ACC_X_MSB_ADDR 0x13
#define ACC_Y_LSB_ADDR 0x14
#define ACC_Y_MSB_ADDR 0x15
#define ACC_Z_LSB_ADDR 0x16
#define ACC_Z_MSB_ADDR 0x17
#define ACC_XYZ_LEN 6

#define SENSORTIME_0_ADDR 0x18
#define SENSORTIME_0_UNIT (39.0625f / 1000000.0f)
#define SENSORTIME_1_ADDR 0x19
#define SENSORTIME_1_UNIT (10.0 / 1000.0f)
#define SENSORTIME_2_ADDR 0x1A
#define SENSORTIME_2_UNIT (2.56f)
#define SENSORTIME_LEN 3

#define ACC_INT_STAT_1_ADDR 0x1D

#define TEMP_MSB_ADDR 0x22
#define TEMP_LSB_ADDR 0x23
#define TEMP_LEN 2
#define TEMP_UNIT 0.125f
#define TEMP_BIAS 23.0f

#define ACC_CONF_ADDR 0x40
#define ACC_CONF_RESERVED 0x01
#define ACC_CONF_BWP_OSR4 0x00
#define ACC_CONF_BWP_OSR2 0x01
#define ACC_CONF_BWP_NORM 0x02
#define ACC_CONF_ODR_12_5_Hz 0x05
#define ACC_CONF_ODR_25_Hz 0x06
#define ACC_CONF_ODR_50_Hz 0x07
#define ACC_CONF_ODR_100_Hz 0x08
#define ACC_CONF_ODR_200_Hz 0x09
#define ACC_CONF_ODR_400_Hz 0x0A
#define ACC_CONF_ODR_800_Hz 0x0B
#define ACC_CONF_ODR_1600_Hz 0x0C

#define ACC_RANGE_ADDR 0x41
#define ACC_RANGE_3G 0x00
#define ACC_RANGE_6G 0x01
#define ACC_RANGE_12G 0x02
#define ACC_RANGE_24G 0x03

#define INT1_IO_CTRL_ADDR 0x53

#define INT2_IO_CTRL_ADDR 0x54

#define INT_MAP_DATA_ADDR 0x58

#define ACC_SELF_TEST_ADDR 0x6D
#define ACC_SELF_TEST_OFF 0x00
#define ACC_SELF_TEST_POS 0x0D
#define ACC_SELF_TEST_NEG 0x09

#define ACC_PWR_CONF_ADDR 0x7C
#define ACC_PWR_CONF_SUS 0x03
#define ACC_PWR_CONF_ACT 0x00

#define ACC_PWR_CTRL_ADDR 0x7D
#define ACC_PWR_CTRL_ON 0x04
#define ACC_PWR_CTRL_OFF 0x00

#define ACC_SOFTRESET_ADDR 0x7E
#define ACC_SOFTRESET_VAL 0xB6

/*-----陀螺仪寄存器表-----*/
#define GYRO_CHIP_ID_ADDR 0x00
#define GYRO_CHIP_ID_VAL 0x0F

#define GYRO_RATE_X_LSB_ADDR 0x02
#define GYRO_RATE_X_MSB_ADDR 0x03
#define GYRO_RATE_Y_LSB_ADDR 0x04
#define GYRO_RATE_Y_MSB_ADDR 0x05
#define GYRO_RATE_Z_LSB_ADDR 0x06
#define GYRO_RATE_Z_MSB_ADDR 0x07
#define GYRO_XYZ_LEN 6

#define GYRO_INT_STAT_1_ADDR 0x0A

#define GYRO_RANGE_ADDR 0x0F
#define GYRO_RANGE_2000_DEG_S 0x00
#define GYRO_RANGE_1000_DEG_S 0x01
#define GYRO_RANGE_500_DEG_S 0x02
#define GYRO_RANGE_250_DEG_S 0x03
#define GYRO_RANGE_125_DEG_S 0x04

#define GYRO_BANDWIDTH_ADDR 0x10
#define GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define GYRO_ODR_400Hz_BANDWIDTH_47Hz 0x03
#define GYRO_ODR_200Hz_BANDWIDTH_23Hz 0x04
#define GYRO_ODR_100Hz_BANDWIDTH_12Hz 0x05
#define GYRO_ODR_200Hz_BANDWIDTH_64Hz 0x06
#define GYRO_ODR_100Hz_BANDWIDTH_32Hz 0x07

#define GYRO_LPM1_ADDR 0x11
#define GYRO_LPM1_NOR 0x00
#define GYRO_LPM1_SUS 0x80
#define GYRO_LPM1_DEEP_SUS 0x20

#define GYRO_SOFTRESET_ADDR 0x14
#define GYRO_SOFTRESET_VAL 0xB6

#define GYRO_INT_CTRL_ADDR 0x15

#define GYRO_INT3_INT4_IO_CONF_ADDR 0x16

#define GYRO_INT3_INT4_IO_MAP_ADDR 0x18

#define GYRO_SELF_TEST_ADDR 0x3C
#define GYRO_SELF_TEST_ON 0x01

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define DEG2SEC 0.0174532925f
#define SEC2DEG 57.295779578f

typedef struct
{
	float accel[3];
	float gyro[3];
	float angle_q[4];
	float angle[3];
	float temperature;
}imu_data_t;

typedef enum
{
	ACCEL,
	GYRO
}imu_data_type_t;


extern imu_data_t imu_data;

void BMI088_Init(void);
void Get_ImuData(imu_data_t *data);


#endif



