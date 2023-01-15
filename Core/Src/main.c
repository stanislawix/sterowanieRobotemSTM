/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "jsmn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_BUFFER_LENGTH 32
#define UART1_TX_BUFFER_LENGTH 60
#define UART_TX_TIMEOUT_MS 100
#define DMA_RECEIVE_LENGTH 30
#define HCSR04_TIME_BETWEEN_NEXT_SENSOR_USAGE_MS 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for blink01 */
osThreadId_t blink01Handle;
const osThreadAttr_t blink01_attributes = {
  .name = "blink01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for blink02 */
osThreadId_t blink02Handle;
const osThreadAttr_t blink02_attributes = {
  .name = "blink02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for pwmTask */
osThreadId_t pwmTaskHandle;
const osThreadAttr_t pwmTask_attributes = {
  .name = "pwmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
int i;
int r;
int commandBufferIndex;
jsmn_parser p;
jsmntok_t t[128]; /* We expect no more than 128 tokens */
char commandBuffer[COMMAND_BUFFER_LENGTH] = {0};
int nestingLevel;
int speed = 0;
int direction = 0;
int drive_direction = 0;
int motor_speed = 0;

/// ODBIOR KOMEND Z BLUETOOTH
uint8_t ReceiveBuffer[32];
uint8_t UART1_txBuffer[UART1_TX_BUFFER_LENGTH] = {0};

/// CZUJNIKI
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t distance1 = 0;
uint8_t distance2 = 0;
uint8_t distance3 = 0;
uint8_t distance4 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
void StartBlink01(void *argument);
void StartBlink02(void *argument);
void StartPwmTask(void *argument);
void StartSensorTask(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s);
void zeroingBuffersAndJsonParser();
void delay_us (uint16_t us);
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
    commandBufferIndex = 0;
    nestingLevel = 0;
//  speed = 0;
//  direction = 0;

    jsmn_init(&p);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3); //start timera 3 dla generowania PWM do sterowania silnikami i serwem
  HAL_TIM_Base_Start(&htim4); //start timera 4 do odmierzania mikrosekund dla obslugi czujnikow ultradzwiekowych HC-04
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //start timera 3 do generowania PWM do sterowania silnikami i serwem robota
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, DMA_RECEIVE_LENGTH);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink01 */
  blink01Handle = osThreadNew(StartBlink01, NULL, &blink01_attributes);

  /* creation of blink02 */
  blink02Handle = osThreadNew(StartBlink02, NULL, &blink02_attributes);

  /* creation of pwmTask */
  pwmTaskHandle = osThreadNew(StartPwmTask, NULL, &pwmTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

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
    while (1) {

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1680-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CZUJNIK_1_TRIG_Pin|CZUJNIK_2_TRIG_Pin|CZUJNIK_3_TRIG_Pin|CZUJNIK_4_TRIG_Pin
                          |REAR_PHASE_1_Pin|REAR_PHASE_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|FRONT_LEFT_PHASE_1_Pin|FRONT_LEFT_PHASE_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FRONT_RIGHT_PHASE_1_Pin|FRONT_RIGHT_PHASE_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CZUJNIK_1_TRIG_Pin */
  GPIO_InitStruct.Pin = CZUJNIK_1_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CZUJNIK_1_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CZUJNIK_2_TRIG_Pin CZUJNIK_3_TRIG_Pin CZUJNIK_4_TRIG_Pin REAR_PHASE_1_Pin
                           REAR_PHASE_2_Pin */
  GPIO_InitStruct.Pin = CZUJNIK_2_TRIG_Pin|CZUJNIK_3_TRIG_Pin|CZUJNIK_4_TRIG_Pin|REAR_PHASE_1_Pin
                          |REAR_PHASE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin FRONT_LEFT_PHASE_1_Pin FRONT_LEFT_PHASE_2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|FRONT_LEFT_PHASE_1_Pin|FRONT_LEFT_PHASE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FRONT_RIGHT_PHASE_1_Pin FRONT_RIGHT_PHASE_2_Pin */
  GPIO_InitStruct.Pin = FRONT_RIGHT_PHASE_1_Pin|FRONT_RIGHT_PHASE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

void zeroingBuffersAndJsonParser() {
    commandBufferIndex = 0;
    nestingLevel = 0;
    for (int z = 0; z < COMMAND_BUFFER_LENGTH; z++)
        commandBuffer[z] = 0;
    for (int z = 0; z < UART1_TX_BUFFER_LENGTH; z++)
        UART1_txBuffer[z] = 0;
    jsmn_init(&p);
}

void zeroingUartTxBuffer() {
    for (int z = 0; z < UART1_TX_BUFFER_LENGTH; z++)
        UART1_txBuffer[z] = 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    // Check if UART1 trigger the Callback
    if (huart->Instance == USART1) {
        for (int k = 0; k < Size; k++) {
            if (ReceiveBuffer[k] == '{') {
                commandBuffer[commandBufferIndex] = ReceiveBuffer[k];
                commandBufferIndex++;
                nestingLevel++;
            } else if (ReceiveBuffer[k] == '}') {
                commandBuffer[commandBufferIndex] = ReceiveBuffer[k];
                commandBufferIndex++;
                nestingLevel--;
            } else if (ReceiveBuffer[k] == '!') {
                if (nestingLevel != 0) {
                    zeroingBuffersAndJsonParser();
//                    const char *error = "Invalid JSON command - wrong JSON nesting level";
//                    HAL_UART_Transmit(&huart1, error, strlen(error), UART_TX_TIMEOUT_MS);
                    speed = direction = 0;
                    break;
                } else {
                    r = jsmn_parse(&p, commandBuffer, strlen(commandBuffer), t, sizeof(t) / sizeof(t[0]));
                    if (r < 0) {
//                    HAL_UART_Transmit(&huart1, "Failed to parse JSON\n", 22, UART_TX_TIMEOUT_MS);
                    } else if (r < 1 || t[0].type != JSMN_OBJECT) {
//                    HAL_UART_Transmit(&huart1, "Object expected\n", 17, UART_TX_TIMEOUT_MS);
                    } else {
                        for (i = 1; i < r; i++) {
                            if (jsoneq((char *) commandBuffer, &t[i], "spd") == 0) {
                                /* We may use strndup() to fetch string value */
                                speed = strtol((char *) &commandBuffer + (t[i + 1].start), NULL, 10);
                                if (speed > 0) {
                                    motor_speed = speed;
                                    drive_direction = 1;
                                } else if (speed < 0) {
                                    motor_speed = -speed;
                                    drive_direction = -1;
                                } else {
                                    motor_speed = 0;
                                    drive_direction = 0;
                                }
//                                sprintf((char *) &UART1_txBuffer, "Speed: %.*s, konw_spd: %d\n",
//                                        t[i + 1].end - t[i + 1].start,
//                                        commandBuffer + t[i + 1].start, speed);
                                i++;
//                            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
                            } else if (jsoneq(commandBuffer, &t[i], "dir") == 0) {
                                direction = strtol((char *) &commandBuffer + (t[i + 1].start), NULL, 10);
//                                sprintf((char *) &UART1_txBuffer, "Direction: %.*s, konw_dir: %d\n",
//                                        t[i + 1].end - t[i + 1].start,
//                                        commandBuffer + t[i + 1].start, direction);
                                i++;
//                            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
                            } else {
//                                sprintf((char *) &UART1_txBuffer, "Unexpected key: %.*s\n", t[i].end - t[i].start,
//                                        commandBuffer + t[i].start);
//                            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
                            }
                        }
                    }
                    zeroingBuffersAndJsonParser();
                    break;
                }
            } else {
                commandBuffer[commandBufferIndex] = ReceiveBuffer[k];
                commandBufferIndex++;
            }

            //TODO: mozna zoptymalizowac i wywalic poza ifa
            // Start to listening again - IMPORTANT!
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, DMA_RECEIVE_LENGTH);
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        }
    } else {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, DMA_RECEIVE_LENGTH);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    ReceiveBuffer[0] = 0;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // if the interrupt source is channel1
        if (Is_First_Captured == 0) { // if the first value is not captured
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
            Is_First_Captured = 1;  // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else if (Is_First_Captured == 1) {  // if the first is already captured
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            } else if (IC_Val1 > IC_Val2) {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            distance1 = Difference * .034 / 2;
            Is_First_Captured = 0; // set it back to false

//            sprintf((char *) &UART1_txBuffer, "Odleglosc1: %d\n", distance1);
//            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
//            zeroingUartTxBuffer();

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
        }
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { // if the interrupt source is channel2
        if (Is_First_Captured == 0) { // if the first value is not captured
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
            Is_First_Captured = 1;  // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else if (Is_First_Captured == 1) {  // if the first is already captured
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            } else if (IC_Val1 > IC_Val2) {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            distance2 = Difference * .034 / 2;
            Is_First_Captured = 0; // set it back to false

//            sprintf((char *) &UART1_txBuffer, "Odleglosc2: %d\n", distance2);
//            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
//            zeroingUartTxBuffer();

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
        }
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { // if the interrupt source is channel3
        if (Is_First_Captured == 0) { // if the first value is not captured
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
            Is_First_Captured = 1;  // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else if (Is_First_Captured == 1) {  // if the first is already captured
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            } else if (IC_Val1 > IC_Val2) {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            distance3 = Difference * .034 / 2;
            Is_First_Captured = 0; // set it back to false

//            sprintf((char *) &UART1_txBuffer, "Odleglosc3: %d\n", distance3);
//            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
//            zeroingUartTxBuffer();

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
        }
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { // if the interrupt source is channel4
        if (Is_First_Captured == 0) { // if the first value is not captured
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
            Is_First_Captured = 1;  // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else if (Is_First_Captured == 1) {  // if the first is already captured
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            } else if (IC_Val1 > IC_Val2) {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            distance4 = Difference * .034 / 2;
            Is_First_Captured = 0; // set it back to false

//            sprintf((char *) &UART1_txBuffer, "Odleglosc4: %d\n", distance4);
//            HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
//            zeroingUartTxBuffer();

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
        }
    }
}

void delay_us (uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim4,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);  // wait for the counter to reach the us input in the parameter
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlink01 */
/**
* @brief Function implementing the blink01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void *argument)
{
  /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osDelay(500);
    }

    osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlink02 */
/**
* @brief Function implementing the blink02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink02 */
void StartBlink02(void *argument)
{
  /* USER CODE BEGIN StartBlink02 */
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osDelay(600);
    }

    osThreadTerminate(NULL);
  /* USER CODE END StartBlink02 */
}

/* USER CODE BEGIN Header_StartPwmTask */
/**
* @brief Function implementing the pwmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPwmTask */
void StartPwmTask(void *argument)
{
  /* USER CODE BEGIN StartPwmTask */
    /* Infinite loop */
    for (;;) {
        htim3.Instance->CCR1 = (direction / 4) + 75;  // duty cycle: 25 is 0.5ms ; 75 is 1.5 ms ; 125 is 2.5ms
        if (drive_direction == 0) {
            htim3.Instance->CCR2 = 0; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy
            htim3.Instance->CCR3 = 0; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy
            htim3.Instance->CCR4 = 0; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy

            HAL_GPIO_WritePin(REAR_PHASE_1_GPIO_Port, REAR_PHASE_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(REAR_PHASE_2_GPIO_Port, REAR_PHASE_2_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(FRONT_LEFT_PHASE_1_GPIO_Port, FRONT_LEFT_PHASE_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FRONT_LEFT_PHASE_2_GPIO_Port, FRONT_LEFT_PHASE_2_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(FRONT_RIGHT_PHASE_1_GPIO_Port, FRONT_RIGHT_PHASE_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FRONT_RIGHT_PHASE_2_GPIO_Port, FRONT_RIGHT_PHASE_2_Pin, GPIO_PIN_RESET);
        } else if (drive_direction == 1) {
            // 9.5 -:- 20 ms
            // 9.5ms = 9.5 * 50 = 475
            // 20ms = 20 * 50 = 1000
            htim3.Instance->CCR2 = (motor_speed * 525 / 100) + 300; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy
            htim3.Instance->CCR3 = (motor_speed * 525 / 100) + 300; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy
            htim3.Instance->CCR4 = (motor_speed * 525 / 100) + 300; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy

            HAL_GPIO_WritePin(REAR_PHASE_1_GPIO_Port, REAR_PHASE_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(REAR_PHASE_2_GPIO_Port, REAR_PHASE_2_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(FRONT_LEFT_PHASE_1_GPIO_Port, FRONT_LEFT_PHASE_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FRONT_LEFT_PHASE_2_GPIO_Port, FRONT_LEFT_PHASE_2_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(FRONT_RIGHT_PHASE_1_GPIO_Port, FRONT_RIGHT_PHASE_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(FRONT_RIGHT_PHASE_2_GPIO_Port, FRONT_RIGHT_PHASE_2_Pin, GPIO_PIN_RESET);
        } else if (drive_direction == -1) {
            htim3.Instance->CCR2 = (motor_speed * 525 / 100) + 300; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy
            htim3.Instance->CCR3 = (motor_speed * 525 / 100) + 300; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy
            htim3.Instance->CCR4 = (motor_speed * 525 / 100) + 300; //inaczej to, chyba innym pinem zmienia sie kierunek jazdy

            HAL_GPIO_WritePin(REAR_PHASE_1_GPIO_Port, REAR_PHASE_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(REAR_PHASE_2_GPIO_Port, REAR_PHASE_2_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(FRONT_LEFT_PHASE_1_GPIO_Port, FRONT_LEFT_PHASE_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(FRONT_LEFT_PHASE_2_GPIO_Port, FRONT_LEFT_PHASE_2_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(FRONT_RIGHT_PHASE_1_GPIO_Port, FRONT_RIGHT_PHASE_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FRONT_RIGHT_PHASE_2_GPIO_Port, FRONT_RIGHT_PHASE_2_Pin, GPIO_PIN_SET);
        }
        osDelay(50);
//        osThreadYield();
    }

    osThreadTerminate(NULL);
  /* USER CODE END StartPwmTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
    /* Infinite loop */
    for (;;) {
//        HAL_UART_Transmit(&huart1, "Sprawdzam odleglosc...\n", 23, UART_TX_TIMEOUT_MS);

        HAL_GPIO_WritePin(CZUJNIK_1_TRIG_GPIO_Port, CZUJNIK_1_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
        delay_us(11);  // wait for 10 us
        HAL_GPIO_WritePin(CZUJNIK_1_TRIG_GPIO_Port, CZUJNIK_1_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
        osDelay(HCSR04_TIME_BETWEEN_NEXT_SENSOR_USAGE_MS);
        
        HAL_GPIO_WritePin(CZUJNIK_2_TRIG_GPIO_Port, CZUJNIK_2_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
        delay_us(11);  // wait for 10 us
        HAL_GPIO_WritePin(CZUJNIK_2_TRIG_GPIO_Port, CZUJNIK_2_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
        osDelay(HCSR04_TIME_BETWEEN_NEXT_SENSOR_USAGE_MS);

        HAL_GPIO_WritePin(CZUJNIK_3_TRIG_GPIO_Port, CZUJNIK_3_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
        delay_us(11);  // wait for 10 us
        HAL_GPIO_WritePin(CZUJNIK_3_TRIG_GPIO_Port, CZUJNIK_3_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
        osDelay(HCSR04_TIME_BETWEEN_NEXT_SENSOR_USAGE_MS);

        HAL_GPIO_WritePin(CZUJNIK_4_TRIG_GPIO_Port, CZUJNIK_4_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
        delay_us(11);  // wait for 10 us
        HAL_GPIO_WritePin(CZUJNIK_4_TRIG_GPIO_Port, CZUJNIK_4_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

        sprintf(UART1_txBuffer, "%d;%d;%d;%d;\n", distance1, distance2, distance3, distance4);
        HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), UART_TX_TIMEOUT_MS);
//        osDelay(HCSR04_TIME_BETWEEN_NEXT_SENSOR_USAGE_MS);


        osDelay(200);
//        osThreadYield();
    }

    osThreadTerminate(NULL);
  /* USER CODE END StartSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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
    while (1) {
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
