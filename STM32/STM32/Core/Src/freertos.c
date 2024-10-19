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
#include "bsp.h"
#include "stdlib.h"
#include "board.h"
#include "utilities.h"
#include "usbd_cdc_if.h"
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
osThreadId TrajectoryTaskHandle;
osThreadId JogTaskHandle;
osThreadId MotorDataTaskHandle;
osMessageQId TrajCmdQueueHandle;
osMessageQId JogCmdQueueHandle;
osMutexId bufferMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTrajectoryTask(void const * argument);
void StartJogTask(void const * argument);
void ProcessMotorDataTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
        uint32_t *pulIdleTaskStackSize) {
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
  /* Create the mutex(es) */
  /* definition and creation of bufferMutex */
  osMutexDef(bufferMutex);
  bufferMutexHandle = osMutexCreate(osMutex(bufferMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of TrajCmdQueue */
  osMessageQDef(TrajCmdQueue, 16, uint32_t);
  TrajCmdQueueHandle = osMessageCreate(osMessageQ(TrajCmdQueue), NULL);

  /* definition and creation of JogCmdQueue */
  osMessageQDef(JogCmdQueue, 16, uint32_t);
  JogCmdQueueHandle = osMessageCreate(osMessageQ(JogCmdQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TrajectoryTask */
  osThreadDef(TrajectoryTask, StartTrajectoryTask, osPriorityNormal, 0, 256);
  TrajectoryTaskHandle = osThreadCreate(osThread(TrajectoryTask), NULL);

  /* definition and creation of JogTask */
  osThreadDef(JogTask, StartJogTask, osPriorityBelowNormal, 0, 128);
  JogTaskHandle = osThreadCreate(osThread(JogTask), NULL);

  /* definition and creation of MotorDataTask */
  osThreadDef(MotorDataTask, ProcessMotorDataTask, osPriorityLow, 0, 128);
  MotorDataTaskHandle = osThreadCreate(osThread(MotorDataTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTrajectoryTask */
/**
 * @brief Function implementing the TrajectoryTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTrajectoryTask */
void StartTrajectoryTask(void const * argument)
{
  /* USER CODE BEGIN StartTrajectoryTask */
//	osThreadSuspend(MotorDataTaskHandle);
	Board_Init();

	TrajMsg *buf;
	osEvent evt;
	/* Infinite loop */
	for (;;) {
		evt = osMessageGet(TrajCmdQueueHandle, osWaitForever);
		if (evt.status == osEventMessage) {
			buf = (TrajMsg*) evt.value.p;
			Process_Traj_Cmd(buf);
		}
		osDelay(50);
	}
  /* USER CODE END StartTrajectoryTask */
}

/* USER CODE BEGIN Header_StartJogTask */
/**
 * @brief Function implementing the JogTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartJogTask */
void StartJogTask(void const * argument)
{
  /* USER CODE BEGIN StartJogTask */
	JogMsg *buf;
	osEvent evt;
	/* Infinite loop */
	for (;;) {
		evt = osMessageGet(JogCmdQueueHandle, osWaitForever);
		if (evt.status == osEventMessage) {
			buf = (JogMsg*) evt.value.p;
			switch (buf->new_cmd) {
				case CABLE:
					Process_Cable_Cmd(&(buf->cable_cmd));
					break;
				case ARCHOR:
					Process_Archor_Cmd(&(buf->archor_cmd));
					break;
				case STEER:
					Process_GO_Cmd(&(buf->go_cmd));
					break;
				case ROLL:
					Process_A1_Cmd(&(buf->a1_cmd));
					break;
				default:
					break;
			}
		}
		osDelay(50);
	}
  /* USER CODE END StartJogTask */
}

/* USER CODE BEGIN Header_ProcessMotorDataTask */
/**
* @brief Function implementing the MotorDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProcessMotorDataTask */
void ProcessMotorDataTask(void const * argument)
{
  /* USER CODE BEGIN ProcessMotorDataTask */
	MotorFBData motor_fb_data = {0};
	uint8_t data[20] = {0XAA, 0X55};
  /* Infinite loop */
  for(;;)
  {
	if(Buffer_Get(&motor_fb_buffer, &motor_fb_data.motor_type, &motor_fb_data.id, &motor_fb_data.pos, &motor_fb_data.vel)){
		memcpy(&data[2], &motor_fb_data, sizeof(motor_fb_data));
		CDC_Transmit_FS((uint8_t*)data, 12);
	}
    osDelay(10);
  }
  /* USER CODE END ProcessMotorDataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
