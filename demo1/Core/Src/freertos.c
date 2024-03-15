/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "remote.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Shooter.h"
#include "WatchDog.h"
#include "MathData.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_Read */
osThreadId_t IMU_ReadHandle;
const osThreadAttr_t IMU_Read_attributes = {
  .name = "IMU_Read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_SendMessage */
osThreadId_t CAN_SendMessageHandle;
const osThreadAttr_t CAN_SendMessage_attributes = {
  .name = "CAN_SendMessage",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Remote_RecieveM */
osThreadId_t Remote_RecieveMHandle;
const osThreadAttr_t Remote_RecieveM_attributes = {
  .name = "Remote_RecieveM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AMMO_SendMessag */
osThreadId_t AMMO_SendMessagHandle;
const osThreadAttr_t AMMO_SendMessag_attributes = {
  .name = "AMMO_SendMessag",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start_IMU_Read(void *argument);
void Start_CAN_SendMessage(void *argument);
void Start_Remote_RecieveMessage(void *argument);
void Start_AMMO_SendMessage(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMU_Read */
  IMU_ReadHandle = osThreadNew(Start_IMU_Read, NULL, &IMU_Read_attributes);

  /* creation of CAN_SendMessage */
  CAN_SendMessageHandle = osThreadNew(Start_CAN_SendMessage, NULL, &CAN_SendMessage_attributes);

  /* creation of Remote_RecieveM */
  Remote_RecieveMHandle = osThreadNew(Start_Remote_RecieveMessage, NULL, &Remote_RecieveM_attributes);

  /* creation of AMMO_SendMessag */
  AMMO_SendMessagHandle = osThreadNew(Start_AMMO_SendMessage, NULL, &AMMO_SendMessag_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      if(WatchDog_Detect(&Remote_WatchDog_Info)==0)
          PID_Clear();
      HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_IMU_Read */
/**
* @brief Function implementing the IMU_Read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IMU_Read */
void Start_IMU_Read(void *argument)
{
  /* USER CODE BEGIN Start_IMU_Read */
  /* Infinite loop */
  for(;;)
  {
      Get_ImuData(&imu_data);
    osDelay(1);
  }
  /* USER CODE END Start_IMU_Read */
}

/* USER CODE BEGIN Header_Start_CAN_SendMessage */
/**
* @brief Function implementing the CAN_SendMessage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CAN_SendMessage */
void Start_CAN_SendMessage(void *argument)
{
  /* USER CODE BEGIN Start_CAN_SendMessage */
  /* Infinite loop */
  for(;;)
  {
//      Chassis_UpData();
//      Gimbal_UpData();
//      Posi_IncrPID(NULL, &IncrPID_Info[CHASSIS1], &motor_info[CHASSIS1]);
//      Posi_IncrPID(NULL, &IncrPID_Info[CHASSIS2], &motor_info[CHASSIS2]);
//      Posi_IncrPID(NULL, &IncrPID_Info[CHASSIS3], &motor_info[CHASSIS3]);
//      Posi_IncrPID(NULL, &IncrPID_Info[CHASSIS4], &motor_info[CHASSIS4]);
//      Posi_IncrPID(&PosiPID_Info[YAW], &IncrPID_Info[YAW], &motor_info[YAW]);
//      Posi_IncrPID(&PosiPID_Info[PITCH], &IncrPID_Info[PITCH], &motor_info[PITCH]);
//
//      Send_ChasissInfo(IncrPID_Info[CHASSIS1].Output,IncrPID_Info[CHASSIS2].Output, IncrPID_Info[CHASSIS3].Output, IncrPID_Info[CHASSIS4].Output);
//      Send_GimbalConf(PosiPID_Info[YAW].Output, PosiPID_Info[PITCH].Output);

    osDelay(1);
  }
  /* USER CODE END Start_CAN_SendMessage */
}

/* USER CODE BEGIN Header_Start_Remote_RecieveMessage */
/**
* @brief Function implementing the Remote_RecieveM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Remote_RecieveMessage */
void Start_Remote_RecieveMessage(void *argument)
{
  /* USER CODE BEGIN Start_Remote_RecieveMessage */

  /* Infinite loop */
  for(;;)
  {
    if( RemoteData.data_size == 18 )
    {
      RemoteData_Unpack();
    }
    osDelay(1);
  }

  /* USER CODE END Start_Remote_RecieveMessage */
}

/* USER CODE BEGIN Header_Start_AMMO_SendMessage */
/**
* @brief Function implementing the AMMO_SendMessag thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AMMO_SendMessage */
void Start_AMMO_SendMessage(void *argument)
{
  /* USER CODE BEGIN Start_AMMO_SendMessage */
  uint16_t num=0,LastAngle=0;
  /* Infinite loop */
  for(;;)
  {
////    if(RemoteData.shoot_strategy== REVERSE)
////	{
////		motor_info[FRICT_L].target_speed =	-ReverseSpeed;
////		motor_info[FRICT_R].target_speed =	 ReverseSpeed;
////	}
////	else
////	{
//		motor_info[FRICT_L].target_speed = 	ShooterSpeed;
//		motor_info[FRICT_R].target_speed = 	-ShooterSpeed;
////	}
//    for(uint8_t i=FRICT_L;i<=FRICT_R;i++)
//    {
//        IncrPID(&IncrPID_Info[i], &motor_info[i]);
//    }
//  if(RemoteData.shoot_strategy==SINGLE_SHOOT && num==0)
//    {
//        LastAngle=motor_info[AMMO].angle;
//        motor_info[AMMO].target_speed=8000;
//        num++;
//    }
//    if(RemoteData.shoot_strategy==SINGLE_SHOOT
//            && num<27
//            && (motor_info[AMMO].angle-LastAngle+8192)%8192>2048)
//    {
//        motor_info[AMMO].target_speed=8000;
//        num++;
//    }
//    else if(RemoteData.shoot_strategy==SINGLE_SHOOT && num==27)
//    {
//        motor_info[AMMO].target_speed=0;
//    }
//    else if(RemoteData.shoot_strategy==NO_SHOOT)
//    {
//        motor_info[AMMO].target_speed=0;
//        num=0;
//    }
//    else if(RemoteData.shoot_strategy==MULTI_SHOOT)
//    {
//        motor_info[AMMO].target_speed=8000;
//    }
//    Posi_IncrPID(NULL, &IncrPID_Info[AMMO], &motor_info[AMMO]);
//    Send_AmmoSpeed(IncrPID_Info[FRICT_L].Output,IncrPID_Info[FRICT_R].Output ,IncrPID_Info[AMMO].Output );
    osDelay(10);
  }
  /* USER CODE END Start_AMMO_SendMessage */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

