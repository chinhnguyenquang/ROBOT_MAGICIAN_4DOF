/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "alt_main.h"
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
/* Definitions for Sensor_Task */
osThreadId_t Sensor_TaskHandle;
const osThreadAttr_t Sensor_Task_attributes = {
  .name = "Sensor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskHmi */
osThreadId_t TaskHmiHandle;
const osThreadAttr_t TaskHmi_attributes = {
  .name = "TaskHmi",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_DMA_HMI */
osThreadId_t Task_DMA_HMIHandle;
const osThreadAttr_t Task_DMA_HMI_attributes = {
  .name = "Task_DMA_HMI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Task_controlmot */
osThreadId_t Task_controlmotHandle;
const osThreadAttr_t Task_controlmot_attributes = {
  .name = "Task_controlmot",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for Task_Dwin_get_t */
osThreadId_t Task_Dwin_get_tHandle;
const osThreadAttr_t Task_Dwin_get_t_attributes = {
  .name = "Task_Dwin_get_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Queue_DWIN */
osMessageQueueId_t Queue_DWINHandle;
const osMessageQueueAttr_t Queue_DWIN_attributes = {
  .name = "Queue_DWIN"
};
/* Definitions for Seme_Tx_dwin */
osSemaphoreId_t Seme_Tx_dwinHandle;
const osSemaphoreAttr_t Seme_Tx_dwin_attributes = {
  .name = "Seme_Tx_dwin"
};
/* Definitions for Theta_xyz */
osSemaphoreId_t Theta_xyzHandle;
const osSemaphoreAttr_t Theta_xyz_attributes = {
  .name = "Theta_xyz"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Sensor_AS5600(void *argument);
void Dwin_Task(void *argument);
void TASK_DMA_ENTRY(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of Seme_Tx_dwin */
  Seme_Tx_dwinHandle = osSemaphoreNew(1, 1, &Seme_Tx_dwin_attributes);

  /* creation of Theta_xyz */
  Theta_xyzHandle = osSemaphoreNew(1, 0, &Theta_xyz_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_DWIN */
  Queue_DWINHandle = osMessageQueueNew (37, sizeof(uint8_t), &Queue_DWIN_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Sensor_Task */
  Sensor_TaskHandle = osThreadNew(Sensor_AS5600, NULL, &Sensor_Task_attributes);

  /* creation of TaskHmi */
  TaskHmiHandle = osThreadNew(Dwin_Task, NULL, &TaskHmi_attributes);

  /* creation of Task_DMA_HMI */
  Task_DMA_HMIHandle = osThreadNew(TASK_DMA_ENTRY, NULL, &Task_DMA_HMI_attributes);

  /* creation of Task_controlmot */
  Task_controlmotHandle = osThreadNew(StartTask05, NULL, &Task_controlmot_attributes);

  /* creation of Task_Dwin_get_t */
  Task_Dwin_get_tHandle = osThreadNew(StartTask06, NULL, &Task_Dwin_get_t_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Sensor_AS5600 */
/**
  * @brief  Function implementing the Sensor_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Sensor_AS5600 */
void Sensor_AS5600(void *argument)
{
  /* USER CODE BEGIN Sensor_AS5600 */
  /* Infinite loop */
	Sensor_AS5600_RTOS();
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END Sensor_AS5600 */
}

/* USER CODE BEGIN Header_Dwin_Task */
/**
* @brief Function implementing the TaskHmi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Dwin_Task */
void Dwin_Task(void *argument)
{
  /* USER CODE BEGIN Dwin_Task */
  /* Infinite loop */
	Dwin_RTOS();
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END Dwin_Task */
}

/* USER CODE BEGIN Header_TASK_DMA_ENTRY */
/**
* @brief Function implementing the Task_DMA_HMI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK_DMA_ENTRY */
void TASK_DMA_ENTRY(void *argument)
{
  /* USER CODE BEGIN TASK_DMA_ENTRY */
  /* Infinite loop */
	DwinUsartTask_RTOS();
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END TASK_DMA_ENTRY */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Task_controlmot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
	Control_motor_RTOS();
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the Task_Dwin_get_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	Control_Dwin_get_theta_RTOS();
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

