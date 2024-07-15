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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "lib_dht22.h"
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

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SERIAL_INPUT 1 // a 0 si sans ST-Link

#define RX_BUFF_SIZE 200
#define MAIN_BUFF_SIZE 200
#define MSG_SIZE 70

#define SLEEP_TIME 30 //2min

UART_HandleTypeDef *lora = &huart4;
UART_HandleTypeDef *serial = &huart2;  //modifier si sans ST-link

uint8_t rcvbff[RX_BUFF_SIZE];
uint8_t mainbff[MAIN_BUFF_SIZE];

char message[MSG_SIZE];

int at_rdy = 0;
int joined_rdy = 0;
int message_rdy = 0;
int veille_rdy = 0;
int get_data = 0;
int error =0;
int wait_msg=0;

typedef struct{
	int Hi;
	float Ti;
	int Ho;
	float To;
	int presence ;
	int poids;
	float battery;
}DatasStruct;
DatasStruct myDatas;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	#if SERIAL_INPUT
	if(huart == serial)  //huart->Instance == USART2
	{
		memcpy(mainbff, rcvbff, Size);
		uint8_t tmpbff[strlen(rcvbff)+1];
					tmpbff[0]='>';
					memcpy(&tmpbff[1], rcvbff, strlen(rcvbff));

					HAL_UART_Transmit(serial,(uint8_t*)tmpbff,strlen(rcvbff)+1,10);

		HAL_UART_Transmit(lora,(uint8_t*)rcvbff,strlen(rcvbff),10);



		for(int i=0; i<RX_BUFF_SIZE; i++)
			rcvbff[i] = 0x00;
		rcvbff[0] = '\0';

		HAL_UARTEx_ReceiveToIdle_DMA(serial, rcvbff, RX_BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
	#endif
	if(huart == lora)  //huart->Instance == USART2
		{
			memcpy(mainbff, rcvbff, Size);


			HAL_UART_Transmit(serial,(uint8_t*)rcvbff,strlen(rcvbff),60);



			if(strstr(rcvbff, "+MSG: Done\r\n")!=NULL)
			{
				HAL_UART_Transmit(serial,(uint8_t*)"Valid data\n",11,10);
				 // pret a envoyer un nouveau message
				veille_rdy=1;
				wait_msg = 0;
			}
			if(strstr(rcvbff, "+AT: OK\r\n")!=NULL || strstr(rcvbff, "+LOWPOWER: WAKEUP\r\n")!=NULL)
			{
				at_rdy=1; // pret a envoyer un nouveau message
				wait_msg=0;
			}
			if(strstr(rcvbff, "+JOIN: Network joined\r\n")!=NULL || strstr(rcvbff, "+JOIN: Joined already\r\n")!=NULL)
			{
				joined_rdy=1; // pret a envoyer un nouveau message
				HAL_UART_Transmit(serial,(uint8_t*)"Ready to send\n",14,10);
				get_data=1;
				wait_msg=0;
			}

			HAL_UART_Transmit(serial,(uint8_t*)"____\n",5,10);

			for(int i=0; i<RX_BUFF_SIZE; i++)
				rcvbff[i] = 0x00;
			rcvbff[0] = '\0';

			HAL_UARTEx_ReceiveToIdle_DMA(lora, rcvbff, RX_BUFF_SIZE);
			__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
		}
}
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
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DHT_Init(DHT_GPIO_Port, DHT_Pin, &htim3);

  HAL_UART_Transmit(serial, (uint8_t *)"UART2 INIT\n" , 11, 10);

    //uint8_t premessage[55] = "AT+MSG\r\n";

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
    HAL_Delay(100);
    DHT_Init(DHT_GPIO_Port, DHT_Pin, &htim3);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //HAL_PWREx_EnableFlashPowerDown();
  //HAL_PWREx_EnableLowPowerRunMode();

  /* Enter Stop Mode*/

  //HAL_TIM_Base_Start_IT(&htim16);

	 //timer_val = __HAL_TIM_GET_COUNTER(&htim16);

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, SLEEP_TIME, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

  HAL_UARTEx_ReceiveToIdle_DMA(serial, rcvbff, RX_BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(lora, rcvbff, RX_BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

  //HAL_GPIO_Init(Presence_GPIO_Port, Presence_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	/*LoRa send*/

	  if(!at_rdy && !wait_msg )
	  {
		  verifcation_rep();//necessaire car sinon la premiere demande de donnée ne se fait pas
		  HAL_Delay(200);//pour espacer deux demandes au DHT22
		  HAL_UART_Transmit(lora, (uint8_t *)"AT\r\n" , 4, 10);
		  wait_msg=1;
	  }
	  else if(at_rdy && !wait_msg && !joined_rdy)
	  {
		  HAL_UART_Transmit(lora, (uint8_t *)"AT+JOIN\r\n" , 9, 10);
		  wait_msg=1;
	  }
	  else if(joined_rdy && get_data && !message_rdy)
	  {
		  //humidite
		  DHT_GetValue();
		  //if(verifcation_rep()==1)
		  //HAL_UART_Transmit(&huart2, (uint8_t *) "verif\n", 6, 20);
		  myDatas.Hi = (int)DHT_GetHumi();
		  myDatas.Ho = 0;

		  //températures
		  myDatas.Ti = DHT_GetTemp();
		  myDatas.To = 0.0;

		  //poids
		  myDatas.poids = 00;

		  //presence
		  myDatas.presence = (HAL_GPIO_ReadPin(PresPins_GPIO_Port, PresPins_Pin)!=GPIO_PIN_SET);

		  //battery
		  HAL_ADC_Init(&hadc1);
		  HAL_Delay(60);
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
		  uint16_t value_adc = 0;
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1000);
		  value_adc =HAL_ADC_GetValue(&hadc1);

		  HAL_ADC_Stop(&hadc1);
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
		  HAL_ADC_DeInit(&hadc1);

		  myDatas.battery = (value_adc*5.069/4095);

		  //set message to send
		  sprintf(message,"AT+MSG=\"Ti:%0.2f,Hi:%d,To:%0.2f,Ho:%d,Pr:%d,Pd:%0.2f,Bt:%0.2f\"\r\n",myDatas.Ti,myDatas.Hi,myDatas.To,myDatas.Ho,myDatas.presence,myDatas.poids/1000.0, myDatas.battery);
///Ti:([-]?\d*\.?\d+),Hi:(\d+),To:([-]?\d*\.?\d+),Ho:(\d+),Pr:(\d+),Pd:([-]?\d*\.?\d+),Bt:([-]?\d*\.?\d+)/

		  get_data=0;
		  message_rdy=1;

		  HAL_UART_Transmit(serial, (uint8_t *)"stp >38\n" ,7 , 10);
	  }
	  else if(joined_rdy && message_rdy)
	  {

		  HAL_UART_Transmit(serial, (uint8_t *)"send >" ,6 , 10);

		  HAL_UART_Transmit(serial, (uint8_t *)message , strlen(message), 70);
		  HAL_UART_Transmit(lora, (uint8_t *)message , strlen(message), 70);
		  message_rdy=0;
		  wait_msg=1;
	  }
	  else if(!error && joined_rdy  && !message_rdy && veille_rdy)
	  {
		  /*preSTOP mode*/

		  HAL_UART_Transmit(lora, (uint8_t *)"AT+LOWPOWER\n\r" , 13, 60);
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
		 HAL_UART_Transmit(serial, (uint8_t *)"Start sleep\n" , 12, 10);
		  HAL_Delay(100);
		HAL_SuspendTick();
		//HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 4098, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, SLEEP_TIME, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

		/* Enter STOP 2 mode */
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		SystemClock_Config();
		HAL_ResumeTick();

		  HAL_UART_Transmit(lora, (uint8_t *)"AT\n\r" , 4, 60); // reveille le lora
		 HAL_UART_Transmit(serial, (uint8_t *)"Wake up\n" , 8, 60);
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);

		  MX_ADC1_Init();
		  HAL_Delay(200);
		get_data=1;
		veille_rdy=0;
	  }
	/*green led blink*/
	/*
	HAL_Delay(615);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_Delay(285);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
*/


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeD
  * ef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA8 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB11
                           PB12 PB13 PB14 PB15
                           PB3 BatPin_Pin PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|BatPin_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PresPins_Pin */
  GPIO_InitStruct.Pin = PresPins_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PresPins_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT_Pin */
  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//callback

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim)
{
	if(tim == &htim16)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}*/
/*void HAL_TIM_PeriodElapsedCallback(LPTIM_HandleTypeDef *lptim)
{
	if(lptim == &hlptim1)
	{
		//HAL_GPIO_TogglefPin(LD2_GPIO_Port, LD2_Pin);
	}
}*/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  RTC_AlarmTypeDef sAlarm;
  HAL_RTC_GetAlarm(hrtc,&sAlarm,RTC_ALARM_A,FORMAT_BIN);
  if(sAlarm.AlarmTime.Minutes>58) {
    sAlarm.AlarmTime.Minutes=0;
  }else{
    sAlarm.AlarmTime.Minutes=sAlarm.AlarmTime.Minutes+1;
  }
    //while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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
