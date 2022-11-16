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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jsmn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_BUFFER_LENGTH 50
#define UART1_TX_BUFFER_LENGTH 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int i;
int r;
int commandBufferIndex;
jsmn_parser p;
jsmntok_t t[128]; /* We expect no more than 128 tokens */
char commandBuffer[COMMAND_BUFFER_LENGTH] = {0};
int nestingLevel;
int speed;
int direction;


uint8_t UART1_rxBuffer[1] = {0};
uint8_t UART1_txBuffer[50] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_TIM3_Init(void);

static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void zeroingBuffersAndJsonParser();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    commandBufferIndex = 0;
    nestingLevel = 0;
    speed = 0;
    direction = 0;

    jsmn_init(&p);

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
//        HAL_UART_Transmit(&huart1, UART1_txBuffer, 12, 10);
//      htim3.Instance->CCR1 = 25;  // duty cycle is .5 ms
//      HAL_Delay(1200);
        htim3.Instance->CCR1 = (direction / 2) + 75;  // duty cycle is 1.5 ms
        HAL_Delay(50);
//        HAL_Delay(5000);
//      htim3.Instance->CCR1 = 125;  // duty cycle is 2.5 ms
//      HAL_Delay(2000);
//      HAL_Delay(2000);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 1680 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
    GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(UART1_rxBuffer[0] == '{') {
        commandBuffer[commandBufferIndex] = UART1_rxBuffer[0];
        commandBufferIndex++;
        nestingLevel++;
    } else if(UART1_rxBuffer[0] == '}') {
        commandBuffer[commandBufferIndex] = UART1_rxBuffer[0];
        commandBufferIndex++;
        nestingLevel--;
    } else if(UART1_rxBuffer[0] == '!') {
        if (nestingLevel != 0) {
            zeroingBuffersAndJsonParser();
            const char *error = "Invalid JSON command - wrong JSON nesting level";
            HAL_UART_Transmit(&huart1, error, strlen(error), 100);
            speed = direction = 0;
        } else {
            int temp = strlen(commandBuffer);
            r = jsmn_parse(&p, commandBuffer, strlen(commandBuffer), t, sizeof(t) / sizeof(t[0]));
            if (r < 0) {
                HAL_UART_Transmit(&huart1, "Failed to parse JSON\n", 22, 100);
            } else if (r < 1 || t[0].type != JSMN_OBJECT) {
                HAL_UART_Transmit(&huart1, "Object expected\n", 17, 100);
            } else {
                for (i = 1; i < r; i++) {
                    if (jsoneq((char *) commandBuffer, &t[i], "spd") == 0) {
                        /* We may use strndup() to fetch string value */
//                        int essa = strtol( (char*) &commandBuffer + (t[i + 1].end - t[i + 1].start),NULL, 10);
                        speed = strtol( (char*) &commandBuffer + (t[i + 1].start),NULL, 10);
                        sprintf((char *) &UART1_txBuffer, "Speed: %.*s, konw_spd: %d\n", t[i + 1].end - t[i + 1].start,
                                commandBuffer + t[i + 1].start, speed);
                        i++;
                        HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), 100);
                    } else if (jsoneq(commandBuffer, &t[i], "dir") == 0) {
                        direction = strtol( (char*) &commandBuffer + (t[i + 1].start),NULL, 10);
                        sprintf((char *) &UART1_txBuffer, "Direction: %.*s, konw_dir: %d\n", t[i + 1].end - t[i + 1].start,
                                commandBuffer + t[i + 1].start, direction);
                        i++;
                        HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), 100);
                    } else {
                        sprintf((char *) &UART1_txBuffer, "Unexpected key: %.*s\n", t[i].end - t[i].start,
                                commandBuffer + t[i].start);
                        HAL_UART_Transmit(&huart1, UART1_txBuffer, strlen(UART1_txBuffer), 100);
                    }
                }
            }
            zeroingBuffersAndJsonParser();

        }
    } else {
        commandBuffer[commandBufferIndex] = UART1_rxBuffer[0];
        commandBufferIndex++;
    }

//        for (i = 1; i < r; i++) {
//            if (jsoneq(JSON_STRING, &t[i], "user") == 0) {
//                /* We may use strndup() to fetch string value */
//                sprintf((char *) &UART1_txBuffer, "User: %.*s\n", t[i + 1].end - t[i + 1].start,
//                        JSON_STRING + t[i + 1].start);
//                i++;
//                HAL_UART_Transmit(&huart1, UART1_txBuffer, 40, 100);
//            } else if (jsoneq(JSON_STRING, &t[i], "uid") == 0) {
//                sprintf(&UART1_txBuffer, "UID: %.*s\n", t[i + 1].end - t[i + 1].start,
//                       JSON_STRING + t[i + 1].start);
//                i++;
//                HAL_UART_Transmit(&huart1, UART1_txBuffer, 40, 100);
//            } else {
//                sprintf(&UART1_txBuffer, "Unexpected key: %.*s\n", t[i].end - t[i].start,
//                       JSON_STRING + t[i].start);
//                HAL_UART_Transmit(&huart1, UART1_txBuffer, 40, 100);
//            }
//        }

    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
}

void zeroingBuffersAndJsonParser() {
    commandBufferIndex = 0;
    nestingLevel = 0;
    for(int z = 0; z < COMMAND_BUFFER_LENGTH; z++)
        commandBuffer[z] = 0;
    for(int z = 0; z < UART1_TX_BUFFER_LENGTH; z++)
        UART1_txBuffer[z] = 0;
    jsmn_init(&p);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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
