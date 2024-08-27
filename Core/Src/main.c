/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for CONTROL */
osThreadId_t CONTROLHandle;
const osThreadAttr_t CONTROL_attributes = {
  .name = "CONTROL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AxisXYDesire */
osThreadId_t AxisXYDesireHandle;
const osThreadAttr_t AxisXYDesire_attributes = {
  .name = "AxisXYDesire",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AutoRun */
osThreadId_t AutoRunHandle;
const osThreadAttr_t AutoRun_attributes = {
  .name = "AutoRun",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for WitMotion */
osThreadId_t WitMotionHandle;
const osThreadAttr_t WitMotion_attributes = {
  .name = "WitMotion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myPID */
osThreadId_t myPIDHandle;
const osThreadAttr_t myPID_attributes = {
  .name = "myPID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AxisDesireQueue */
osMessageQueueId_t AxisDesireQueueHandle;
const osMessageQueueAttr_t AxisDesireQueue_attributes = {
  .name = "AxisDesireQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartControl(void *argument);
void StartAxisXYDesire(void *argument);
void StartAutoRun(void *argument);
void StartTaskWit(void *argument);
void StartPID(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PID_Param_t pid;
double out_1, out_2, out_3=0;
double rpm_1, rpm_2, rpm_3=0;

volatile double Vd=0;
volatile double Theta=0;

uint8_t select_hand=5;
uint8_t flag_V_slow=true;
uint8_t rx_data;
uint8_t Dis;

uint8_t count=0;

void pid_config(void){
	pid.Kp=0.3;
	pid.Ki=0.2;
	pid.Kd=0.005;
	pid.target_val_1=V1;
	pid.target_val_2=V2;
	pid.target_val_3=V3;
	PID_init(&pid);
}
/**---UART6 Interrupt--*/
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//
//PUTCHAR_PROTOTYPE
//{
//  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//  return ch;
//}
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_SPI3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//  PCA9685_Init(&hi2c1);

  	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 100); //motor 1
  	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 100); //motor 2
  	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 100); //motor 3
//  	HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, GPIO_PIN_SET);
//  	HAL_GPIO_WritePin(DIRECTION_2_GPIO_Port, DIRECTION_2_Pin, GPIO_PIN_SET);
//  	HAL_GPIO_WritePin(DIRECTION_3_GPIO_Port, DIRECTION_3_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);

//    UARTStdioConfig(USART2,true);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of AxisDesireQueue */
  AxisDesireQueueHandle = osMessageQueueNew (20, sizeof(uint8_t), &AxisDesireQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CONTROL */
  CONTROLHandle = osThreadNew(StartControl, NULL, &CONTROL_attributes);

  /* creation of AxisXYDesire */
  AxisXYDesireHandle = osThreadNew(StartAxisXYDesire, NULL, &AxisXYDesire_attributes);

  /* creation of AutoRun */
  AutoRunHandle = osThreadNew(StartAutoRun, NULL, &AutoRun_attributes);

  /* creation of WitMotion */
  WitMotionHandle = osThreadNew(StartTaskWit, NULL, &WitMotion_attributes);

  /* creation of myPID */
  myPIDHandle = osThreadNew(StartPID, NULL, &myPID_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xfff-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
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
  huart3.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIRECTION_3_Pin|DIRECTION_2_Pin|DIRECTION_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC_Output_GPIO_Port, HC_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SS_Pin */
  GPIO_InitStruct.Pin = SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIRECTION_3_Pin DIRECTION_2_Pin DIRECTION_1_Pin */
  GPIO_InitStruct.Pin = DIRECTION_3_Pin|DIRECTION_2_Pin|DIRECTION_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : HC_Output_Pin */
  GPIO_InitStruct.Pin = HC_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC_Output_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
char Set_Default[] = "Enter the start value:\n";
char Desired[] = "Enter the desired value:\n";
char Repeated[] = "The process is repeated:\n";
char NotRepeated[] = "The process is not repeated:\n";
char Go[] = "Start!\n";
char Restart[] = "Restart!\n";
char Pause[] = "Pause!\n";
char errorValue[] = "Error Value!\n";
char GoTo[] = "Go to:";
void Trans(int16_t x, int16_t y){
	char data_trans[10];
	int len;

	len = sprintf(data_trans, "%d %d\n", x, y);
	HAL_UART_Transmit(&huart3, (uint8_t*)&data_trans, len,delay_trans);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartControl */
/**
  * @brief  Function implementing the CONTROL thread.
  * @param  argument: Not used
  * @retval None
  */
void Moving(uint8_t PSX_RX, double Vd, double Theta){
	if(flag_V_slow==true) Vd /= 500.0f;
	else if(flag_V_slow==false) Vd /= 300.0f;
	Robot_Move(Vd, Theta,0);
	switch(PSX_RX){
	case Rotate_Right:
		Robot_Move(0, 0, -0.3);
		break;
	case Rotate_Left:
		Robot_Move(0, 0, 0.3);
		break;
	}
}
void Hand(uint8_t PSX_RX){
	switch(PSX_RX){
	case Fast:
		flag_V_slow=false;
		break;
	case Slow:
		flag_V_slow=true;
		break;
	}

}
void Calculate_angle(double LX, double LY) {

    double angle_rad = atan(LY / LX);
    Theta = angle_rad * (180.0f / PI)-30.0f;
    if(LX < 0)Theta += 180.0f;
    if(LX >= 0 && LY < 0)Theta += 360.0f;
    if(Theta<0) Theta+=360.0f;
}

void Calculate_Vd(uint8_t PSX_RX[]){
	int16_t LX=PSX_RX[7];
	int16_t LY=PSX_RX[8];

	LX=LX-128;
	LY=127-LY;

	Vd=(fabs(LX)>=fabs(LY))?fabs(LX):fabs(LY);
	if(Vd<50) Vd=0;
	Calculate_angle((double)LX, (double)LY);
}
/* USER CODE END Header_StartControl */
void StartControl(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {

	uint8_t PSX_RX[9]= { 0x00 };
	uint8_t PSX_TX[2] = {
		0x01, 0x42
	};
	spi_enable;
	HAL_SPI_TransmitReceive(&hspi3, PSX_TX, PSX_RX, 9, 10);
	spi_disable;
	if(PSX_RX[1]==0x73){
		vTaskSuspend(AutoRunHandle);
		vTaskSuspend(WitMotionHandle);

		Calculate_Vd(PSX_RX);
		Hand(PSX_RX[4]);
		Moving(PSX_RX[3],Vd,Theta);
	} else{
		vTaskResume(AutoRunHandle);
		vTaskResume(WitMotionHandle);

	}

		osDelay(10);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAxisXYDesire */
/**
* @brief Function implementing the AxisXYDesire thread.
* @param argument: Not used
* @retval None
*/

int16_t X_desire[N]={0};
int16_t Y_desire[N]={0};
int16_t X_real[1]={0};
int16_t Y_real[1]={0};

bool repeat=false;
bool pause=false;
bool restart=false;
bool runDone=true;


uint8_t X_pause[N]={0};
int8_t buff[50];

uint8_t idx=1;
int8_t idx_real=0;

volatile uint32_t timeToRun;

static state_t State=FIRST;
static run_t Run=NO;
void timeRun(run_t state, int16_t Big, int16_t Small){
	Run=state;
	timeToRun=(int32_t)(Big-Small)*(int32_t)speed;
	/*
	 * speed la thoi gian hoan thanh 1m (ms)
	 * thoi gian chay bang quang duong * speed
	 * */
	runDone=false;
//	vTaskSuspend(AutoRunHandle);
}
int convertToNegative(int number) {
    return -number;
}
void Caculate_Desire(int16_t desire[], uint8_t idx, int8_t buff[]){
	desire[idx]=0;
	int i = 0;
	bool negative=false;
	if(buff[0]=='-'){
		negative=true;
		i=1;
	}
	else{
		negative=false;
		i=0;
	}

	for(;i < count; i++) {
		desire[idx] = desire[idx] * 10 + buff[i];
	}

	if(negative) desire[idx] = convertToNegative(desire[idx]);
	count=0;
}
void Begin(void){
	if(idx_real<=(idx-1)){
		if(X_pause[idx_real]){
			HAL_UART_Transmit(&huart3, (uint8_t*)&Pause, sizeof(Pause)-1,delay_trans);
			State=PAUSE;
			pause=true;
			idx_real++;
		}
		else{
			if(Y_desire[idx_real]==Y_real[0]){
				if(X_desire[idx_real]>X_real[0]){
					timeRun(FORWARD, X_desire[idx_real], X_real[0]);
				}else if(X_desire[idx_real]<X_real[0]){
					timeRun(BACKWARD, X_real[0], X_desire[idx_real]);
				}
			}
			else if(X_desire[idx_real]==X_real[0]){
				if(Y_desire[idx_real]>Y_real[0]){
					timeRun(RIGHT, Y_desire[idx_real], Y_real[0]);
				}else if(Y_desire[idx_real]<Y_real[0]){
					timeRun(LEFT, Y_real[0], Y_desire[idx_real]);
				}
			}
			HAL_UART_Transmit(&huart3, (uint8_t*)&GoTo, sizeof(GoTo)-1,delay_trans);
			Trans(X_desire[idx_real], Y_desire[idx_real]);

			X_real[0]=X_desire[idx_real];
			Y_real[0]=Y_desire[idx_real];
			idx_real++;
		}
	}
	else if(idx >1){
		if(!repeat){
			HAL_UART_Transmit(&huart3, (uint8_t*)&Pause, sizeof(Pause)-1,delay_trans);
			State=PAUSE;
		}
		if(repeat){
			HAL_UART_Transmit(&huart3, (uint8_t*)&Repeated, sizeof(Repeated)-1,delay_trans);
			State=REPEAT;
		}
	}
}
void SetDefault(msgQueueObj_t msg, int16_t *X_real, int16_t *Y_real){
	if((msg.buffer)!='\n'&&(msg.buffer)!=' '){
		if((msg.buffer)=='-'){
			buff[count]=msg.buffer;

			count++;
		}
		else if(isdigit((int)msg.buffer)){
			buff[count]=(msg.buffer-'0');
			count++;
		}
	}
	else if((msg.buffer)==' '&& count!=0){
		Caculate_Desire(X_real, 0, buff);
		X_desire[0]=X_real[0];
	}
	else if((msg.buffer)=='\n' && count!=0){
		Caculate_Desire(Y_real, 0, buff);
		Y_desire[0]=Y_real[0];
		Trans(X_real[0], Y_real[0]);
		memset(buff,0,sizeof(buff));
	}
}
void Desire(msgQueueObj_t msg){
	if(pause){
		X_pause[idx]=1;
		idx==N-1?idx=1:idx++;
		pause=!pause;
		return;
	}
	if((msg.buffer)!='\n'&&(msg.buffer)!=' '){
		if((msg.buffer)=='-'){
			buff[count]=msg.buffer;

			count++;
		}
		else if(isdigit((int)msg.buffer)){
			buff[count]=(msg.buffer-'0');
			count++;
		}
	}
	else if((msg.buffer)==' '&& count!=0){
		Caculate_Desire(X_desire, idx, buff);
	}
	else if((msg.buffer)=='\n' && count!=0){
		Caculate_Desire(Y_desire, idx, buff);

		if(X_desire[idx]!=X_desire[idx-1] && Y_desire[idx]!=Y_desire[idx-1]){
			HAL_UART_Transmit(&huart3, (uint8_t*)&errorValue, sizeof(errorValue)-1,delay_trans);
			return;
		}
		Trans(X_desire[idx], Y_desire[idx]);
		memset(buff,0,sizeof(buff));
		if(idx==N-1) idx=1;
		else idx++;
	}
}

/* USER CODE END Header_StartAxisXYDesire */
void StartAxisXYDesire(void *argument)
{
  /* USER CODE BEGIN StartAxisXYDesire */
	uint32_t count;
	msgQueueObj_t msg;
	osStatus_t status=osError;
  /* Infinite loop */
  for(;;)
  {
	count = osMessageQueueGetCount(AxisDesireQueueHandle);
	if(count >0 ) status = osMessageQueueGet(AxisDesireQueueHandle, &msg.buffer, NULL, delay_trans);   // wait for message
	else 	msg.buffer='\0';
	if (status == osOK) {
		if(isalpha((int)msg.buffer)){
			if(msg.buffer=='f'){
				HAL_UART_Transmit(&huart3, (uint8_t*)&Set_Default, sizeof(Set_Default)-1,delay_trans);
				State=FIRST;
			}
			if(msg.buffer=='d'){
				HAL_UART_Transmit(&huart3, (uint8_t*)&Desired, sizeof(Desired)-1,delay_trans);
				State=DESIRE;
			}
			else if(msg.buffer=='r'){
				if(!repeat)HAL_UART_Transmit(&huart3, (uint8_t*)&Repeated, sizeof(Repeated)-1,delay_trans);
				if(repeat) HAL_UART_Transmit(&huart3, (uint8_t*)&NotRepeated, sizeof(NotRepeated)-1,delay_trans);
				repeat=!repeat;
			}
			else if(msg.buffer=='g'){
				HAL_UART_Transmit(&huart3, (uint8_t*)&Go, sizeof(Go)-1,delay_trans);
				State=GO;
			}
			else if(msg.buffer=='s'){
				HAL_UART_Transmit(&huart3, (uint8_t*)&Restart, sizeof(Restart)-1,delay_trans);
				restart=true;
				State=RESTART;
			}
			else if(msg.buffer=='p'){
				HAL_UART_Transmit(&huart3, (uint8_t*)&Pause, sizeof(Pause)-1,delay_trans);
				if(State==GO) State=PAUSE;
				pause=true;
			}
		}
	}
	switch(State){
	case FIRST:
		SetDefault(msg, X_real, Y_real);
		break;
	case DESIRE:
		Desire(msg);
		break;
	case REPEAT:
		idx_real=0;
		State=GO;
		break;
	case GO:
		pause = false;
		if(runDone){
			Begin();
		}
		break;
	case PAUSE:
		break;
	case RESTART:
		memset(X_desire,0,sizeof(X_desire));
		memset(Y_desire,0,sizeof(Y_desire));
		memset(X_pause,0,sizeof(X_pause));

		X_real[0]=0;
		Y_real[0]=0;

		idx=1; idx_real=0;
		repeat=false;
		break;
	}
    osDelay(1);
  }
  /* USER CODE END StartAxisXYDesire */
}

/* USER CODE BEGIN Header_StartAutoRun */
/**
* @brief Function implementing the AutoRun thread.
* @param argument: Not used
* @retval None
*/
static uint32_t updateTimeToRun=0;
static uint8_t flag_dr=donot;

void Caculate_Run(uint32_t timeToRun, uint16_t angle){
	if(restart){
		Robot_Move(0, 0, 0);
		Run=NO;
		return;
	}
	if(pause){
		Robot_Move(0, 0, 0);
		return;
	}
	if(flag_dr==right || flag_dr==left){
		return;
	}
	if(updateTimeToRun<=timeToRun ){
		Robot_Move(0.2, 60, 0);
		updateTimeToRun+=ticktak;
	}
	else{
		Run=NO;
	}

}
/* USER CODE END Header_StartAutoRun */
void StartAutoRun(void *argument)
{
  /* USER CODE BEGIN StartAutoRun */
  /* Infinite loop */
  for(;;)
  {
	switch(Run){
	case FORWARD:
		Caculate_Run(timeToRun, Angle_FORWARD);
		break;
	case BACKWARD:
		Caculate_Run(timeToRun, Angle_FORWARD);
		break;
	case RIGHT:
		Caculate_Run(timeToRun, Angle_FORWARD);
		break;
	case LEFT:
		Caculate_Run(timeToRun, Angle_FORWARD);
		break;
	case NO:
		runDone=true;
		updateTimeToRun=0;
		restart=false;
		break;
	}
	osDelay(ticktak);
  }
  /* USER CODE END StartAutoRun */
}

/* USER CODE BEGIN Header_StartTaskWit */
/**
* @brief Function implementing the WitMotion thread.
* @param argument: Not used
* @retval None
*/
int16_t angle_desire=0;
/* USER CODE END Header_StartTaskWit */
void StartTaskWit(void *argument)
{
  /* USER CODE BEGIN StartTaskWit */
  /* Infinite loop */
  for(;;)
  {
	switch(Run){
	case FORWARD:
		angle_desire=0;
		break;
	case BACKWARD:
		angle_desire=-179;
		break;
	case RIGHT:
		angle_desire=-90;
		break;
	case LEFT:
		angle_desire=90;
		break;
	case NO:
		break;
	}
//	UARTprintf("angle z: %d \r\n",angle_real.z);
	int16_t diff=fabs(angle_desire - angle_real.z);
	if(pause==true || runDone == true){
		flag_dr=no;
	}
	else if(angle_real.z >= angle_desire-tolerance && angle_real.z <= angle_desire+tolerance){
		flag_dr=donot;
	}
	else if(angle_real.z > angle_desire+tolerance ){
		if(diff <= 180) flag_dr=right;
		else flag_dr=left;
	}
	else if(angle_real.z < angle_desire-tolerance){
		if(diff <= 180) flag_dr=left;
		else flag_dr=right;
	}

	if (flag_dr==no){
		Robot_Move(0, 0, 0);
	}
	else if (flag_dr==donot){
		vTaskResume(AutoRunHandle);
	}
	else if(flag_dr==right){
		vTaskSuspend(AutoRunHandle);
		if(diff>=40) Robot_Move(0, 0, -0.2);
		else Robot_Move(0, 0, -0.025);
	}
	else if(flag_dr==left){
		vTaskSuspend(AutoRunHandle);
		if(diff>=40) Robot_Move(0, 0, 0.2);
		else Robot_Move(0, 0, 0.025);
	}
	osDelay(50);
  }
  /* USER CODE END StartTaskWit */
}

/* USER CODE BEGIN Header_StartPID */
/**
* @brief Function implementing the myPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPID */
void StartPID(void *argument)
{
  /* USER CODE BEGIN StartPID */
  /* Infinite loop */
  for(;;)
  {
//	  PID();
    osDelay(100);
  }
  /* USER CODE END StartPID */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
