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
#include "bsp_can.h"
#include "bsp_pid.h"
#include "bsp_remote.h"
#include "stdio.h"      
#include "string.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int16_t SD_out_1,SD_out_2,SD_out_3,SD_out_4;
int16_t SD_out_7,WZ_out_7;
int16_t SD_out_5,WZ_out_5;
int jieqiu,tuisong,mocalun = 20000;
extern PID_ SDpid_1,SDpid_2,SDpid_3,SDpid_4;
extern Motor Motor_1,Motor_2,Motor_3,Motor_4,Motor_5,Motor_7;
extern PID_ WZpid_5,SDpid_5;
extern PID_ WZpid_7,SDpid_7;
int16_t  get_speed[4];
char test_buf1[128] ;
float vx,vy,vw;

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
osThreadId pid_canHandle;
osThreadId uratHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void pid_can_Task(void const * argument);
void uart_Task(void const * argument);

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
  /* definition and creation of pid_can */
  osThreadDef(pid_can, pid_can_Task, osPriorityNormal, 0, 128);
  pid_canHandle = osThreadCreate(osThread(pid_can), NULL);

  /* definition and creation of urat */
  osThreadDef(urat, uart_Task, osPriorityAboveNormal, 0, 128);
  uratHandle = osThreadCreate(osThread(urat), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_pid_can_Task */
/**
  * @brief  Function implementing the pid_can thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pid_can_Task */
void pid_can_Task(void const * argument)
{
  /* USER CODE BEGIN pid_can_Task */
  /* Infinite loop */
  for(;;)
  {
		get_absolute_angle(&Motor_7);
		chassis_motor( Elrs.ch[4], Elrs.ch[3], Elrs.ch[1],  get_speed);
		SD_out_1 = PID_compute(&SDpid_1, get_speed[0], Motor_1.speed );
    SD_out_2 = PID_compute(&SDpid_2, get_speed[1], Motor_2.speed );
		SD_out_3 = PID_compute(&SDpid_3, get_speed[2], Motor_3.speed );	
		SD_out_4 = PID_compute(&SDpid_4, get_speed[3], Motor_4.speed );
    WZ_out_5 = PID_compute(&WZpid_5, tuisong, Motor_5.cumulative_angle);
		SD_out_5 = PID_compute(&SDpid_5, WZ_out_5, Motor_5.speed );
		WZ_out_7 = PID_compute(&WZpid_7, jieqiu, Motor_7.cumulative_angle);
		SD_out_7 = PID_compute(&SDpid_7, WZ_out_7, Motor_7.speed );	
		CAN_cmd_chassis(SD_out_1, SD_out_2, SD_out_3, SD_out_4);
    CAN_cmd_jieqiu(SD_out_5, 0, SD_out_7, 0);
		motor5065(0x3FF, mocalun);

    osDelay(10);
  }
  /* USER CODE END pid_can_Task */
}

/* USER CODE BEGIN Header_uart_Task */
/**
* @brief Function implementing the urat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_Task */
void uart_Task(void const * argument)
{
  /* USER CODE BEGIN uart_Task */

  /* Infinite loop */
  for(;;)
  {
		
		if (Elrs.ch[7] == 3 && (Elrs.ch_flag[8] == 3 || Elrs.ch_flag[8] == 1)) 
        {
					  jieqiu = -50000;
			

            Elrs.ch_flag[8] = 0;
        }
		
		if (Elrs.ch[7] == 3 && (Elrs.ch_flag[8] == 3 || Elrs.ch_flag[8] == 1))
    {
      if(tuisong == 0)
          tuisong = -337000;
          //mocalun = 20000;
      else
          tuisong = 0;
      Elrs.ch_flag[8] = 0;
    }

		if (Elrs.ch[7] == 1 && (Elrs.ch_flag[8] == 3 || Elrs.ch_flag[8] == 1)) 
        {
            
            jieqiu = -85000;			

            Elrs.ch_flag[8] = 0;
        }
		
		
		

	
    osDelay(10);
  }
  /* USER CODE END uart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
