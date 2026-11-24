/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GET_LUX_FLAG (1U << 0)
#define RUN_DAC_ADC_TEST
#define ADC_MAX    4095.0f
#define R_FIXED    1600.0f	// Fixed divider resistor
#define LDR_K      4.45e4f  // Photoresistor extracted parameter
#define LDR_GAMMA  0.632f  // Photoresistor extracted parameter
#define NUM_LEDS 3
#define HIGH_LUX_THRESHOLD 60.0f
#define LOW_LUX_THRESHOLD 3.0f
#define RAMP_RATE 0.01f
#ifdef RUN_DAC_ADC_TEST
	#define STATIC_LSB_TOLERANCE 20
	#define INVERSE_GAIN_ERROR 200
	#define SELF_TEST_DONE_FLAG (1U << 0)
#endif
#define BUFFER_SIZE 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for Update_LUX_Task */
osThreadId_t Update_LUX_TaskHandle;
const osThreadAttr_t Update_LUX_Task_attributes = {
  .name = "Update_LUX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Update_PWM_Task */
osThreadId_t Update_PWM_TaskHandle;
const osThreadAttr_t Update_PWM_Task_attributes = {
  .name = "Update_PWM_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Self_Test_Task */
osThreadId_t Self_Test_TaskHandle;
const osThreadAttr_t Self_Test_Task_attributes = {
  .name = "Self_Test_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LUX_Mutex */
osMutexId_t LUX_MutexHandle;
const osMutexAttr_t LUX_Mutex_attributes = {
  .name = "LUX_Mutex"
};
/* USER CODE BEGIN PV */
uint16_t adc_buf[BUFFER_SIZE];
static float lux;
static const uint32_t LED_CH[NUM_LEDS] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static float current_brightness[NUM_LEDS];
static uint32_t arr;
#ifdef RUN_DAC_ADC_TEST
  static uint32_t failures = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC_Init(void);
void Update_LUX(void *argument);
void Update_PWM(void *argument);
void Run_Self_Test(void *argument);

/* USER CODE BEGIN PFP */
static float Avg_ADC_Value(void);
static float ADC_To_LUX(float n);
static float Clamp_To_Range(float n, float min, float max);
static void Update_LEDs(float lux);
#ifdef RUN_DAC_ADC_TEST
	static void DAC_ADC_Test(void);
	int _write(int file, char *ptr, int len);
#endif
static void LUX_Set(float n);
static float LUX_Get(void);
static void Start_Timers(void);
static void Start_ADC(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of LUX_Mutex */
  LUX_MutexHandle = osMutexNew(&LUX_Mutex_attributes);

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
  /* creation of Update_LUX_Task */
  Update_LUX_TaskHandle = osThreadNew(Update_LUX, NULL, &Update_LUX_Task_attributes);

  /* creation of Update_PWM_Task */
  Update_PWM_TaskHandle = osThreadNew(Update_PWM, NULL, &Update_PWM_Task_attributes);

  /* creation of Self_Test_Task */
  Self_Test_TaskHandle = osThreadNew(Run_Self_Test, NULL, &Self_Test_Task_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 11499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 275;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// When the DMA is updated, set a flag in the Get_LUX task so that it knows to change the value
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc == &hadc1) osThreadFlagsSet(Update_LUX_TaskHandle, GET_LUX_FLAG);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc == &hadc1) osThreadFlagsSet(Update_LUX_TaskHandle, GET_LUX_FLAG);
}

static float Avg_ADC_Value(void) {
    uint32_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
    	sum += adc_buf[i];
    }

    return sum / (float)BUFFER_SIZE;
}

static void Update_LEDs(float lux) {
	// Based on the lux thresholds, linearly interpolate the log of the current lux to get a 'darkness' value
	float darkness = (logf(HIGH_LUX_THRESHOLD / lux) / logf(HIGH_LUX_THRESHOLD / LOW_LUX_THRESHOLD));
	darkness = Clamp_To_Range(darkness, 0.0f, 1.0f) * NUM_LEDS;
	for (int i=0; i<NUM_LEDS; i++) {
		float target_brightness = Clamp_To_Range((darkness - i), 0.0f, 1.0f);
		float diff = Clamp_To_Range(target_brightness - current_brightness[i], -RAMP_RATE, RAMP_RATE);
		current_brightness[i] += diff;
		__HAL_TIM_SET_COMPARE(&htim2, LED_CH[i], (uint32_t)(current_brightness[i]*current_brightness[i]*(arr+1)));
	}
}

static float ADC_To_LUX(float n) {
    if (n < 1.0f) {
    	n = 1.0f;
    }

    if (n > 4094.0f) {
    	n = 4094.0f;
    }

    float r_ldr = R_FIXED * ((ADC_MAX / n) - 1.0f);

    if (r_ldr < 1.0f) {
    	r_ldr = 1.0f;
    }

    float lux = powf(LDR_K / r_ldr, 1.0f / LDR_GAMMA);
    // This was the characterized range so I don't trust the value outside
    lux = Clamp_To_Range(lux, 1e-3f, 1120.0f);
    return lux;
}

static float Clamp_To_Range(float n, float min, float max) {
	return (fminf(fmaxf(n, min), max));
}

static void  LUX_Set(float n) {
    osMutexAcquire(LUX_MutexHandle, osWaitForever);
    lux = n;
    osMutexRelease(LUX_MutexHandle);
}

static float LUX_Get(void) {
    osMutexAcquire(LUX_MutexHandle, osWaitForever);
    float n = lux;
    osMutexRelease(LUX_MutexHandle);
    return n;
}

static void Start_Timers(void) {
	HAL_TIM_Base_Start_IT(&htim2);

	  for (int i=0; i<3; i++) {
		  HAL_TIM_PWM_Start(&htim2, LED_CH[i]);
	  }
	  arr = __HAL_TIM_GET_AUTORELOAD(&htim2);


	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Controls ADC sampling
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Controls photoresistor divider

}

static void Start_ADC(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, BUFFER_SIZE);
}

#ifdef RUN_DAC_ADC_TEST
static void DAC_ADC_Test(void) {
	// TBH I used chat to find out about this API
	// But the idea is that I want to just use software polling for the test
	// because it makes it less error-prone and AFAIK is okay given the requirements
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	HAL_ADC_Init(&hadc1);

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4; // This is the sauce that makes hooks it up to the DAC
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//	osDelay(20);

	for (int32_t dac_code=0; dac_code<4096; dac_code++) {
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code);
		osDelay(2);
		size_t sum = 0;
		for (int i=0; i<16; i++) {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 5);
			sum += HAL_ADC_GetValue(&hadc1);
		}
		int32_t AVG_ADC_Value = sum/16;
		int32_t LSB_Tolerance = STATIC_LSB_TOLERANCE + (dac_code/INVERSE_GAIN_ERROR);

		if (abs(dac_code - AVG_ADC_Value) > LSB_Tolerance) {
			failures++;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			printf("FAIL dac_code=%lu AVG_ADC_Value=%ld\r\n", (unsigned long)dac_code, (long)AVG_ADC_Value);
		}

	}
	printf("Self-test done: %lu failures\r\n", (unsigned long)failures);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
	MX_ADC1_Init(); // Put everything back to how it was
}

int _write(int file, char *ptr, int len)
{
    while (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_BUSY) { }
    return len;
}

#endif

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Update_LUX */
/**
  * @brief  Function implementing the Update_LUX_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Update_LUX */
void Update_LUX(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	#ifdef RUN_DAC_ADC_TEST
	    osThreadFlagsWait(SELF_TEST_DONE_FLAG, osFlagsWaitAny, osWaitForever);
	#endif
	Start_Timers();
	Start_ADC();
  for(;;)
  {

	  osThreadFlagsWait(GET_LUX_FLAG, osFlagsWaitAny, osWaitForever);
	  float temp_LUX = ADC_To_LUX(Avg_ADC_Value());
	  LUX_Set(temp_LUX);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Update_PWM */
/**
* @brief Function implementing the Update_PWM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Update_PWM */
void Update_PWM(void *argument)
{
  /* USER CODE BEGIN Update_PWM */
	float latest_LUX = LOW_LUX_THRESHOLD; // start off
  /* Infinite loop */
  for(;;)
  {
	latest_LUX = LUX_Get();
	Update_LEDs(latest_LUX);
    osDelay(1);
  }
  /* USER CODE END Update_PWM */
}

/* USER CODE BEGIN Header_Run_Self_Test */
/**
* @brief Function implementing the Self_Test_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Run_Self_Test */
void Run_Self_Test(void *argument)
{
  /* USER CODE BEGIN Run_Self_Test */
	#ifdef RUN_DAC_ADC_TEST
	DAC_ADC_Test();                                           // existing function
	osThreadFlagsSet(Update_LUX_TaskHandle, SELF_TEST_DONE_FLAG);   // release the pipeline
  	#endif
	osThreadExit();

  /* USER CODE END Run_Self_Test */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
