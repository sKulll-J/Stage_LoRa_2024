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
#include <string.h>
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
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

char reception[100]={0};
char reception_lora[100]={0};
char commande[100]="AT+MODE=TEST\r\n";
char commande_bis[50]="AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF\r\n";
char commande_ter[50]="AT+TEST=RXLRPKT\r\n";

UART_HandleTypeDef *lora = &huart1;
UART_HandleTypeDef *serial = &huart2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RX_BUFF_SIZE 200
#define MAIN_BUFF_SIZE 200

uint8_t rcvbff[RX_BUFF_SIZE];
uint8_t mainbff[MAIN_BUFF_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
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
	if(huart == lora)  //huart->Instance == USART2
		{
			memcpy(mainbff, rcvbff, Size);


			HAL_UART_Transmit(serial,(uint8_t*)rcvbff,strlen(rcvbff),10);

			if(strcmp(rcvbff, "+MSG: Done\r\n")==0)
				HAL_UART_Transmit(serial,(uint8_t*)"Valid data\n",11,10);

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
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_UART_Receive_IT(&huart2,(uint8_t *) rcvbff, sizeof(rcvbff));
  //HAL_UART_Receive_DMA (&huart2, rcvbff, 150);
  HAL_UART_Transmit(serial, (uint8_t *)"UART2 INIT\n" , 11, 10);

  char automode[55] = {0xFF,0xFF};
   strcpy(&automode[2],"AT+LOWPOWER=AUTOOFF\n\f");
  //HAL_UART_Transmit(lora, (uint8_t *)automode , strlen(automode), 60);
  char message[55] = "rcv\n";
  //uint8_t premessage[55] = "AT+MSG\r\n";

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
/*
  HAL_UART_Transmit(&huart4,message , 7, 10);
  HAL_Delay(1000);

  uint8_t message2[55] = "AT+MSG\r\n";
  HAL_UART_Transmit(&huart4,message2 , 7, 10);
  HAL_Delay(1000);*/
  //HAL_UART_Receive_DMA (serial, (uint8_t*)reception, 100);
  //HAL_UART_Receive_IT(&huart2, rcvbff, 150);

  HAL_UARTEx_ReceiveToIdle_DMA(serial, rcvbff, RX_BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(lora, rcvbff, RX_BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*if(__HAL_UART_GET_FLAG(lora,UART_FLAG_RXNE)==SET)
	  {
		  //HAL_UART_DMAStop(serial);
		  for(int i=0; i<10; i++)
		  {

			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
			  HAL_Delay(10);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
			  HAL_Delay(10);
		  }
		  HAL_UART_Receive_DMA(lora, (uint8_t*)reception_lora, 100);
	  			HAL_UART_DMAStop(lora);

	  			HAL_UART_Transmit(serial,(uint8_t*)reception_lora,strlen(reception_lora),500);

	  			HAL_UART_Receive_DMA(lora, (uint8_t*)reception_lora, 100);
	  			if(strcmp(reception_lora, "AA\n")==0)
	  				  HAL_UART_Transmit(serial,(uint8_t*)message,4,1000);
	  			for(int i=0; i<150; i++)
	  				reception_lora[i] = 0x00;
	  			reception_lora[0] = '\0';
	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);*/
	  //}

	  /*if(__HAL_UART_GET_FLAG(serial,UART_FLAG_RXNE)==SET){
		  HAL_Delay(200);
		  HAL_UART_DMAStop(serial);
		  reception[strlen(reception)-1] = '\r';
		  reception[strlen(reception)] = '\n';

//		  strcat(reception, "\r\n");
		  HAL_UART_Transmit(lora,(uint8_t*) reception,strlen(reception),500);

		  HAL_UART_Transmit(serial,(uint8_t*)reception,strlen(reception),500);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			//HAL_Delay(100);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			for(int i=0; i<150; i++)
				reception[i] = 0x00;
			reception[0] = '\0';

			HAL_UART_Receive_DMA(serial, (uint8_t*)reception, 100);
	  }*/


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
  huart1.Init.BaudRate = 9600;
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
