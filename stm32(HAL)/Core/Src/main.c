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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//_______________________________________________________________________________________________________________________________________________________________________

GPIO_TypeDef * Ax = GPIOC;
const uint16_t APin = GPIO_PIN_6;
GPIO_TypeDef * Bx = GPIOC;
const uint16_t BPin = GPIO_PIN_4;
GPIO_TypeDef * Cx = GPIOC;
const uint16_t CPin = GPIO_PIN_3;
GPIO_TypeDef * Dx = GPIOC;
const uint16_t DPin = GPIO_PIN_1;
GPIO_TypeDef * Ex = GPIOC;
const uint16_t EPin = GPIO_PIN_2;
GPIO_TypeDef * Fx = GPIOC;
const uint16_t FPin = GPIO_PIN_5;
GPIO_TypeDef * Gx = GPIOC;
const uint16_t GPin = GPIO_PIN_0;
GPIO_TypeDef * DPx = GPIOC;
const uint16_t DPPin = GPIO_PIN_7;

//Zmienne programu
volatile char numersToDisplay[4] = {0, 1, 2, 3};
volatile _Bool pointPosition[4] = {0, 0, 0, 0};
const uint16_t positonSelect[4] = {GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_9, GPIO_PIN_8};
volatile char rotor = 0;

// Zmienne odpowiadające za obsługę przetwornika ADC
double raw = 0.0;
volatile uint16_t ADCValue;
volatile uint8_t ADCReady; // Flaga czy już jest nowy gotowy odczyt

// Zmienne odpowiadające za komunikację przez UART
volatile uint8_t TXReady = 1;
uint8_t TxData[10];
uint16_t TxDataSize = 0;

// Zmienne odpowiadające za sterowanie sygnałem PWM
uint16_t lastPWM = 0;
const uint16_t PPeriod = 600; // Stała proporcjonalności
const uint16_t ITime = 40; // Stała całkowania

// Funkcja wyświetlająca wybraną cyfrę(chodzi o kolejność) na wbudowanym wyświetlaczu
void displayNumer(char numer, char position, _Bool point) {
    //Wyświetlanie kropki
	if(point) HAL_GPIO_WritePin(DPx, DPPin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DPx, DPPin, GPIO_PIN_RESET);

    //Resetowanie wyświetlanego ekranu
    HAL_GPIO_WritePin(GPIOC, positonSelect[0], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, positonSelect[1], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, positonSelect[2], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, positonSelect[3], GPIO_PIN_SET);

    //Wybieranie pola w ekranie
    HAL_GPIO_WritePin(GPIOC, positonSelect[position], GPIO_PIN_RESET);

    //Wyświetlanie numeru
	switch(numer) {
	case 0:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_RESET);
	  break;
	case 1:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_RESET);
	  break;
	case 2:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	case 3:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	case 4:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	case 5:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	case 6:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	case 7:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_RESET);
	  break;
	case 8:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	case 9:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	default:
	  HAL_GPIO_WritePin(Ax, APin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Bx, BPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Cx, CPin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Dx, DPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ex, EPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Fx, FPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Gx, GPin, GPIO_PIN_SET);
	  break;
	}
}

// Funkcja pomocnicza do convertDouble dla liczb < 10000 oraz > 1000
void convertDouble10000(double numer) {
	numersToDisplay[0] = ((int)(numer) % 10000) / 1000;
    numersToDisplay[1] = ((int)(numer) % 1000) / 100;
    numersToDisplay[2] = ((int)(numer) % 100) / 10;
    numersToDisplay[3] = (int)(numer) % 10;

    pointPosition[0] = 0;
    pointPosition[1] = 0;
    pointPosition[2] = 0;
    pointPosition[3] = 0;
}

// Funkcja pomocnicza do convertDouble dla liczb < 1000 oraz > 100
void convertDouble1000(double numer) {
    numersToDisplay[0] = ((int)(numer) % 1000) / 100;
    numersToDisplay[1] = ((int)(numer) % 100) / 10;
    numersToDisplay[2] = (int)(numer) % 10;
    numersToDisplay[3] = (int)(numer * 10) % 10;

    pointPosition[0] = 0;
    pointPosition[1] = 0;
    pointPosition[2] = 1;
    pointPosition[3] = 0;
}

// Funkcja pomocnicza do convertDouble dla liczb < 100 oraz > 10
void convertDouble100(double numer) {
    numersToDisplay[0] = ((int)(numer) % 100) / 10;
    numersToDisplay[1] = (int)(numer) % 10;
    numersToDisplay[2] = (int)(numer * 10) % 10;
    numersToDisplay[3] = (int)(numer * 100) % 10;

    pointPosition[0] = 0;
    pointPosition[1] = 1;
    pointPosition[2] = 0;
    pointPosition[3] = 0;
}

// Funkcja pomocnicza do convertDouble dla liczb < 10
void convertDouble10(double numer) {
    numersToDisplay[0] = (int)(numer) % 10;
    numersToDisplay[1] = (int)(numer * 10) % 10;
    numersToDisplay[2] = (int)(numer * 100) % 10;
    numersToDisplay[3] = (int)(numer * 1000) % 10;

    pointPosition[0] = 1;
    pointPosition[1] = 0;
    pointPosition[2] = 0;
    pointPosition[3] = 0;
}

// Funkcja przepisująca liczbę od tablic odpowiadających za wyświetlanie na ekranie wbudowanym w płytkę
void convertDouble(double numer) {
    if (numer < 10) convertDouble10(numer);
    else if (numer < 100) convertDouble100(numer);
    else if (numer < 1000) convertDouble1000(numer);
    else if (numer < 10000) convertDouble10000(numer);
    else {
        //Zwracanie błędu
        numersToDisplay[0] = 10;
        numersToDisplay[1] = 10;
        numersToDisplay[2] = 10;
        numersToDisplay[3] = 10;

        pointPosition[0] = 0;
        pointPosition[1] = 0;
        pointPosition[2] = 0;
        pointPosition[3] = 0;
    }
}

// Funkcja obliczająca nową wartość PWM
uint16_t calculatePWMValue() {

	uint16_t buffor = ADCValue;

	buffor *= 12;

	if(buffor > lastPWM + PPeriod) lastPWM += PPeriod/2;
	else if(buffor < lastPWM - PPeriod) lastPWM -= PPeriod/2;
	else lastPWM = buffor;

	return lastPWM;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //_______________________________________________________________________________________________________________________________________________________________________
  // Inicjalizowanie timera odpowiadającego za działanie cykliczego przerwania funkcyjnego
  HAL_TIM_Base_Start_IT(&htim10);
  // Inicjalizowanie timera odpowiadającego za cykliczne działanie przetwornika ADC DMA
  HAL_TIM_Base_Start(&htim2);
  // Inicjalizowanie timera odpowiadającego za sterowanie pinem PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  // Inicjalizowanie pinu ADC DMA
  HAL_ADC_Start_DMA(&hadc1, &ADCValue, 1);
  // Inicjalizowanie stanu pinu LED(niebieskiego) na który wysyłamy sygnał PWM
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	HAL_Delay(ITime);
	// Aktualizowanie wartości rejestru odpowiadającego za wartość PWM
	TIM4->CCR4 = calculatePWMValue();

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 100;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 100;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA6 PA7
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 PB15
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_B_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//_______________________________________________________________________________________________________________________________________________________________________
// Funkcyjne przerwanie cykliczne
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Sprawdzanie czy przerwanie wywołał timer 10
  if (htim == &htim10 )
  {
	// Obracanie zmieną odpowiadającą za pozycję wyświetlanej cyfry(realizacja multiplexingu)
    rotor = (rotor + 1) % 4;
    // Wyświetlanie wartości na ekranie
	displayNumer(numersToDisplay[rotor], rotor, pointPosition[rotor]);

	// Update pomiaru ADC
	if(ADCReady == 1) {
		uint16_t buffor = ADCValue;
		convertDouble(buffor);
		ADCReady = 0;
		// Wysyłanie wartości portem szeregowym
		if(TXReady == 1) {
			TxDataSize = snprintf(TxData, sizeof(TxData), "%d\n", buffor);
			HAL_UART_Transmit_DMA(&huart2, TxData, TxDataSize);
			TXReady = 0;
		}
	}

  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	ADCReady = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	TXReady = 1;
}

/* USER CODE END 4 */

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
