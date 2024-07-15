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
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Disable_Peripherals(void) {
    // Désactiver les horloges des GPIO non utilisés
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();
    __HAL_RCC_GPIOG_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();

    // Désactiver les horloges des périphériques non utilisés
    __HAL_RCC_ADC_CLK_DISABLE();
    __HAL_RCC_DAC1_CLK_DISABLE();
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_I2C2_CLK_DISABLE();
    __HAL_RCC_SPI1_CLK_DISABLE();
    __HAL_RCC_SPI2_CLK_DISABLE();
    __HAL_RCC_USART1_CLK_DISABLE();
    __HAL_RCC_USART2_CLK_DISABLE();
    __HAL_RCC_USART3_CLK_DISABLE();
    __HAL_RCC_UART4_CLK_DISABLE();
    __HAL_RCC_UART5_CLK_DISABLE();

    // Désactiver d'autres périphériques non utilisés
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_TIM2_CLK_DISABLE();
    __HAL_RCC_TIM3_CLK_DISABLE();
    __HAL_RCC_TIM4_CLK_DISABLE();
    __HAL_RCC_TIM5_CLK_DISABLE();
    __HAL_RCC_TIM6_CLK_DISABLE();
    __HAL_RCC_TIM7_CLK_DISABLE();
    __HAL_RCC_TIM8_CLK_DISABLE();

    // Si vous avez d'autres périphériques actifs, désactivez-les également
}

// Fonction pour configurer les GPIO inutilisés
void Configure_Unused_GPIO(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Désactiver toutes les horloges GPIO pour configuration
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Configuration en mode analogique pour tous les GPIO inutilisés
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // Configuration des ports GPIOA
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    __HAL_RCC_GPIOA_CLK_DISABLE();

    // Répétez pour les autres ports GPIO
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_RCC_GPIOB_CLK_DISABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    __HAL_RCC_GPIOC_CLK_DISABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    __HAL_RCC_GPIOD_CLK_DISABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    __HAL_RCC_GPIOE_CLK_DISABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    __HAL_RCC_GPIOF_CLK_DISABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    __HAL_RCC_GPIOG_CLK_DISABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    __HAL_RCC_GPIOH_CLK_DISABLE();
}

void Enter_STOP2_Mode(void) {
    // Désactiver les périphériques non essentiels
    Disable_Peripherals();

    // Configurer les GPIO inutilisés en mode analogique pour éviter les courants de fuite
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // Configurer tous les ports GPIO (exemple pour GPIOA)
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Activer l'horloge pour la configuration
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    __HAL_RCC_GPIOA_CLK_DISABLE(); // Désactiver l'horloge après configuration

    // Répétez pour les autres ports GPIO utilisés

    // Configurer le mode STOP2
    HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  	  GPIO_InitTypeDef GPIO_Init = {0};
  	  GPIO_Init.Pin = GPIO_PIN_13;
  	  GPIO_Init.Mode= GPIO_MODE_EVT_FALLING;
  	  GPIO_Init.Pull = GPIO_NOPULL;
  	  HAL_GPIO_Init(GPIOC, &GPIO_Init);

  	  HAL_SuspendTick();
  	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);

  	  HAL_ResumeTick();
  	  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_PWR_EnterSTANDBYMode();

	  /*HAL_SuspendTick();
	  HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);
	  HAL_ResumeTick();*/
	  //HAL_SuspendTick();
	  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	  //HAL_PWR_E
	  //HAL_ResumeTick();
	  //HAL_PWREx_EnableLowPowerRunMode();
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
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
