/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MOTOR.h"
#include "OA.h"
#include "SERVO.h"
#include "util.h"
#include "LIDAR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*====避障参数设置=======*/
const avoidance_obj avoid_params = {
	.secure_distance = 35,
	.hotzone_limit = 50,
	.minimum_distance = 10
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HOT_ZONE 0
#define COLD_ZONE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId task_LED_blinkHandle;
osThreadId task_motorHandle;
osThreadId task_initializeHandle;
osThreadId task_servoHandle;
osThreadId task_lidarHandle;
osMessageQId byte_form_data_queueHandle;
osMessageQId min_dist_queueHandle;
osMessageQId lidar_data_transmission_queueHandle;
osSemaphoreId servo_moved_flagHandle;
osSemaphoreId initial_array_fill_flagHandle;
/* USER CODE BEGIN PV */
uint8_t ZONE_FLAG = HOT_ZONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void func_LED_blink(void const * argument);
void func_motor(void const * argument);
void func_initialize(void const * argument);
void func_servo(void const * argument);
void func_lidar(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of servo_moved_flag */
  osSemaphoreDef(servo_moved_flag);
  servo_moved_flagHandle = osSemaphoreCreate(osSemaphore(servo_moved_flag), 1);
 
  /* definition and creation of initial_array_fill_flag */
  osSemaphoreDef(initial_array_fill_flag);
  initial_array_fill_flagHandle = osSemaphoreCreate(osSemaphore(initial_array_fill_flag), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of byte_form_data_queue */
  osMessageQDef(byte_form_data_queue, 1, uint32_t);
  byte_form_data_queueHandle = osMessageCreate(osMessageQ(byte_form_data_queue), NULL);

  /* definition and creation of min_dist_queue */
  osMessageQDef(min_dist_queue, 1, uint16_t);
  min_dist_queueHandle = osMessageCreate(osMessageQ(min_dist_queue), NULL);

  /* definition and creation of lidar_data_transmission_queue */
  osMessageQDef(lidar_data_transmission_queue, 32, uint8_t);
  lidar_data_transmission_queueHandle = osMessageCreate(osMessageQ(lidar_data_transmission_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of task_LED_blink */
  osThreadDef(task_LED_blink, func_LED_blink, osPriorityIdle, 0, 128);
  task_LED_blinkHandle = osThreadCreate(osThread(task_LED_blink), NULL);

  /* definition and creation of task_motor */
  osThreadDef(task_motor, func_motor, osPriorityNormal, 0, 128);
  task_motorHandle = osThreadCreate(osThread(task_motor), NULL);

  /* definition and creation of task_initialize */
  osThreadDef(task_initialize, func_initialize, osPriorityHigh, 0, 128);
  task_initializeHandle = osThreadCreate(osThread(task_initialize), NULL);

  /* definition and creation of task_servo */
  osThreadDef(task_servo, func_servo, osPriorityAboveNormal, 0, 128);
  task_servoHandle = osThreadCreate(osThread(task_servo), NULL);

  /* definition and creation of task_lidar */
  osThreadDef(task_lidar, func_lidar, osPriorityNormal, 0, 128);
  task_lidarHandle = osThreadCreate(osThread(task_lidar), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 15;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//开启 PWM 通道
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA8 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB12
                           PB13 PB14 PB15 PB4
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_func_LED_blink */
/**
  * @brief  Function implementing the task_LED_blink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_func_LED_blink */
void func_LED_blink(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_func_motor */
/**
* @brief Function implementing the task_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_motor */
void func_motor(void const * argument)
{
  /* USER CODE BEGIN func_motor */
	/*func_motor()任务读取队列信息，*/
	uint32_t byte_form_array = 0;
	uint16_t minimum_distance = 0;
	osEvent temp;
  /* Infinite loop */
  for(;;)
  {
		temp = osMessageGet(byte_form_data_queueHandle,osWaitForever);
		if(temp.status==osEventMessage)
		{
			byte_form_array = temp.value.v;
		}
		
		temp = osMessageGet(min_dist_queueHandle,100);
		if(temp.status==osEventMessage)
		{
			minimum_distance = temp.value.v;
		}
		
		printf("minimum distance:%d, set param:%d\n",minimum_distance,avoid_params.minimum_distance);
		if(byte_form_array == 0x17||byte_form_array == 0x15||byte_form_array == 0x1d||
			byte_form_array == 0x1f||minimum_distance <= avoid_params.minimum_distance)
				{
					backward(18);
				}//当1的个数大于4或障碍数组为10101时，倒车
		else if((byte_form_array & 0x0e) == 0x00) forward(17);
		else if((byte_form_array&0x1c)>>3 >= (byte_form_array&0x07)) left_steel();
		else if((byte_form_array&0x1c)>>3 < (byte_form_array&0x07)) right_steel();
		else printf("undefined");		
				
    osDelay(50);
  }
  /* USER CODE END func_motor */
}

/* USER CODE BEGIN Header_func_initialize */
/**
* @brief Function implementing the task_initialize thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_initialize */
void func_initialize(void const * argument)
{
  /* USER CODE BEGIN func_initialize */
	for(;;)
	{
		HAL_Delay(50);
		initialize_servo(15,1,150);
		osThreadTerminate(task_initializeHandle);
	}	
  /* USER CODE END func_initialize */
}

/* USER CODE BEGIN Header_func_servo */
/**
* @brief Function implementing the task_servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_servo */
void func_servo(void const * argument)
{
  /* USER CODE BEGIN func_servo */
  /* Infinite loop */
  for(;;)
  {
		spin_servo(&servo_pos,&direction,end_angle_pos,start_angle_pos);
		osSemaphoreRelease(servo_moved_flagHandle);
    osDelay(170);
  }
  /* USER CODE END func_servo */
}

/* USER CODE BEGIN Header_func_lidar */
/**
* @brief Function implementing the task_lidar thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_lidar */
void func_lidar(void const * argument)
{
  /* USER CODE BEGIN func_lidar */
	//int32_t angle  = 0;
	int32_t temp1=0,temp2=0;//临时变量
	uint8_t minimum_dis = 0;
	const uint8_t divide_span = (end_angle_pos - start_angle_pos)/5;
	const uint8_t skip_digits = (20-(end_angle_pos - start_angle_pos))/2;
	static uint8_t initial_array_fill_flag = 0;
	uint16_t byte_form_array=0;
	//存放雷达探测到前方障碍物距离数据
	static uint16_t obstacle_array[20];
	static uint8_t obstacle_array_bitmask[20];
	static uint8_t deducted_obstacle_array[5];
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(servo_moved_flagHandle,osWaitForever);
		read_one_frame(&lidar_distance,&huart2,frame_data);
		if(lidar_distance < 2)
		{
			osDelay(20);
			read_one_frame(&lidar_distance,&huart2,frame_data);
		}		
		//angle = convert_servo_angle(servo_pos);
		/*Step 2:读取障碍物数据，等待障碍数组填满*/
		obstacle_array[servo_pos-5] = lidar_distance;		
		/*仅当已经收集完一次障碍物数据后再开始数据处理*/
		if(initial_array_fill_flag < (20-2*skip_digits))
		{
			initial_array_fill_flag += 1;
		}else 
		{
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//开启 PWM 通道
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);//开启 PWM 通道
			initial_array_fill_flag = 0xFF;
		}
		if(initial_array_fill_flag == 0xFF)
		{
			//找出障碍物的最小距离
			minimum_dis = obstacle_array[skip_digits];
			for(temp1=(skip_digits);temp1<20-(skip_digits);temp1++)
			{
				if(obstacle_array[temp1]<=minimum_dis)
					minimum_dis = obstacle_array[temp1];
			}
			
			/*Sub step 1:计算障碍物的平均距离，如果小于热区hotzone设定的分界值，则设定为热区模式*/
			temp2 = 0;
			for(temp1=(skip_digits);temp1<20-(skip_digits);temp1++)
			{
				temp2 += obstacle_array[temp1];
			}
			temp2 /= 20;
			if(temp2 <= avoid_params.hotzone_limit) ZONE_FLAG = HOT_ZONE;
			else ZONE_FLAG = COLD_ZONE;
			//printf("Current mode:%d\n",ZONE_FLAG);
			
			/*Sub step 2:处理长度20的障碍物数组*/
			if(lidar_distance >= avoid_params.secure_distance)
			{
				obstacle_array_bitmask[servo_pos-5] = 0;
			}else
			{
				obstacle_array_bitmask[servo_pos-5] = 1;//1表示检测到障碍物在避障范围内
			}
			
			/*Sub step 2:把长度为20的障碍物数组缩减为5*/
			deducted_obstacle_array[0] = 0;
			deducted_obstacle_array[4] = 0;
			for(temp2=0;temp2<divide_span;temp2++)
			{
				if(obstacle_array_bitmask[(skip_digits)+temp2]==1)deducted_obstacle_array[0] = 1;
				if(obstacle_array_bitmask[(skip_digits+4*divide_span)+temp2]==1)deducted_obstacle_array[4] = 1;
			}	

			for(temp1=1;temp1<4;temp1++)
			{
				deducted_obstacle_array[temp1] = 0;
				for(temp2=0;temp2<divide_span;temp2++)
				{
					deducted_obstacle_array[temp1] += obstacle_array_bitmask[(skip_digits+temp1*divide_span)+temp2];
				}
				deducted_obstacle_array[temp1] = (deducted_obstacle_array[temp1]/(divide_span/2))>1?1:0;
			}		
//调试用			
			printf("\n");			
			for(temp1=0;temp1<5;temp1++)
			{
				printf("%d ",deducted_obstacle_array[temp1]);
			}
			printf("\n");		
			byte_form_array = 0;
			for(temp1=0;temp1<5;temp1++)
			{
				temp2 = deducted_obstacle_array[temp1];
				byte_form_array |= temp2 << (4-temp1);
			}
			osMessagePut(byte_form_data_queueHandle,byte_form_array,50);
			osMessagePut(min_dist_queueHandle,minimum_dis,50);
		}

    osDelay(100);
  }
  /* USER CODE END func_lidar */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
