/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
osThreadId defaultTaskHandle;
osThreadId ServoTaskHandle;
osThreadId BlueteethTaskHandle;
osThreadId ChangSlowTaskHandle;
osThreadId IMUTaskHandle;
osThreadId OledTaskHandle;
osThreadId MotorTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId JY901TaskHandle;
osThreadId DistanceTaskHandle;
osThreadId FinfLineTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ServoCallback(void const * argument);
void BlueteethCallback(void const * argument);
void ChangSlowCallback(void const * argument);
void IMUCallBack(void const * argument);
void OledCallback(void const * argument);
void MotorCallback(void const * argument);
void ChassisCallback(void const * argument);
void JY901Callback(void const * argument);
void DistanceCallback(void const * argument);
void FinfLineCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, ServoCallback, osPriorityNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of BlueteethTask */
  osThreadDef(BlueteethTask, BlueteethCallback, osPriorityNormal, 0, 128);
  BlueteethTaskHandle = osThreadCreate(osThread(BlueteethTask), NULL);

  /* definition and creation of ChangSlowTask */
  osThreadDef(ChangSlowTask, ChangSlowCallback, osPriorityNormal, 0, 128);
  ChangSlowTaskHandle = osThreadCreate(osThread(ChangSlowTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, IMUCallBack, osPriorityNormal, 0, 128);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of OledTask */
  osThreadDef(OledTask, OledCallback, osPriorityNormal, 0, 128);
  OledTaskHandle = osThreadCreate(osThread(OledTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, MotorCallback, osPriorityNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, ChassisCallback, osPriorityNormal, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of JY901Task */
  osThreadDef(JY901Task, JY901Callback, osPriorityNormal, 0, 128);
  JY901TaskHandle = osThreadCreate(osThread(JY901Task), NULL);

  /* definition and creation of DistanceTask */
  osThreadDef(DistanceTask, DistanceCallback, osPriorityNormal, 0, 128);
  DistanceTaskHandle = osThreadCreate(osThread(DistanceTask), NULL);

  /* definition and creation of FinfLineTask */
  osThreadDef(FinfLineTask, FinfLineCallback, osPriorityNormal, 0, 128);
  FinfLineTaskHandle = osThreadCreate(osThread(FinfLineTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ServoCallback */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoCallback */
__weak void ServoCallback(void const * argument)
{
  /* USER CODE BEGIN ServoCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ServoCallback */
}

/* USER CODE BEGIN Header_BlueteethCallback */
/**
* @brief Function implementing the BlueteethTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlueteethCallback */
__weak void BlueteethCallback(void const * argument)
{
  /* USER CODE BEGIN BlueteethCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BlueteethCallback */
}

/* USER CODE BEGIN Header_ChangSlowCallback */
/**
* @brief Function implementing the ChangSlowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChangSlowCallback */
__weak void ChangSlowCallback(void const * argument)
{
  /* USER CODE BEGIN ChangSlowCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChangSlowCallback */
}

/* USER CODE BEGIN Header_IMUCallBack */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUCallBack */
__weak void IMUCallBack(void const * argument)
{
  /* USER CODE BEGIN IMUCallBack */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMUCallBack */
}

/* USER CODE BEGIN Header_OledCallback */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OledCallback */
__weak void OledCallback(void const * argument)
{
  /* USER CODE BEGIN OledCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OledCallback */
}

/* USER CODE BEGIN Header_MotorCallback */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorCallback */
__weak void MotorCallback(void const * argument)
{
  /* USER CODE BEGIN MotorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorCallback */
}

/* USER CODE BEGIN Header_ChassisCallback */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisCallback */
__weak void ChassisCallback(void const * argument)
{
  /* USER CODE BEGIN ChassisCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisCallback */
}

/* USER CODE BEGIN Header_JY901Callback */
/**
* @brief Function implementing the JY901Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JY901Callback */
__weak void JY901Callback(void const * argument)
{
  /* USER CODE BEGIN JY901Callback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END JY901Callback */
}

/* USER CODE BEGIN Header_DistanceCallback */
/**
* @brief Function implementing the DistanceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DistanceCallback */
__weak void DistanceCallback(void const * argument)
{
  /* USER CODE BEGIN DistanceCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DistanceCallback */
}

/* USER CODE BEGIN Header_FinfLineCallback */
/**
* @brief Function implementing the FinfLineTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FinfLineCallback */
__weak void FinfLineCallback(void const * argument)
{
  /* USER CODE BEGIN FinfLineCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FinfLineCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
