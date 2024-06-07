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
#include "i2c-lcd.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint8_t temp;
volatile uint8_t smoke;
volatile uint8_t gas;
volatile uint8_t flame;
volatile uint8_t button;
//array
volatile uint8_t array[2];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void readTemp();
uint16_t ADC_VAL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Display_Temp (float Temp)
{
	char str[40] = {0};
	lcd_put_cur(0, 0);

	sprintf (str, "TEMP:%.2f", Temp);
	lcd_send_string(str);
	lcd_send_data('C');
}
void display_smoke (uint8_t smoke1)
{
	char str[40] = {0};
	lcd_put_cur(1, 0);
	sprintf (str, "smk:%d", smoke1);
	lcd_send_string(str);
}
void display_flame (uint8_t flame1)
{
	char str[40] = {0};
	lcd_put_cur(1,6);

	sprintf (str, "flm:%d", flame1);
	lcd_send_string(str);
}
void display_gas(uint8_t gas1)
{
	char str[40] = {0};
	lcd_put_cur(1, 11);

	sprintf (str, "gs:%d", gas1);
	lcd_send_string(str);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t var=0;
float val = 0.0;

#define Avg_Slope .0025
#define V25 0.76

float Temp = 0.0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int row = 0;
int col = 0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	lcd_init();
	
	
	lcd_put_cur(0,1);
	
	lcd_send_string ("NHAT QUANG");

  HAL_Delay(1000);
	
	lcd_clear();
	
	HAL_Delay(100);
	
  lcd_put_cur(1, 0);

  lcd_send_string("CHAO THAY");
	
	HAL_Delay(1600);
	
	lcd_clear();
	
	lcd_put_cur(1,0);

	lcd_send_string("NKLOI dep trai");
	
  HAL_Delay(1900);

  lcd_clear ();
	
	lcd_put_cur(0,2);
	
	lcd_send_string("Bao cao do an 1");
	
	lcd_put_cur(1,1);
	
	lcd_send_string("xin duoc bat dau");
	
	HAL_Delay(1000);
	
	lcd_clear();
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1); 
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1); 
		smoke = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
		gas = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
		flame = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
		button = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
		readTemp();
		Display_Temp(Temp);
		display_flame(flame);
		display_gas(gas);
		display_smoke(smoke);
		if(button == 0)
		{
			//Nut khong nhan, che do tu dong bao chay
			/****Cac truong hop phat hien chay va motor se tu dong bat bao gom******/
			//**** FLAME - LED GREEN - PA10******//
			//****** SMOKE - LED BLUE - PA11****//
			//*******GAS - LED RED - PA9*****//
			lcd_put_cur(0,13);
			lcd_send_string("AUTO");
			HAL_Delay(2000);
			if(flame == 0 || smoke == 0 || gas == 0) //phat hien lua, khoi , gas
			{
			lcd_put_cur(0,13);
			lcd_send_string("ALT"); //ALT = ALERT, CANH BAO
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0); //BAT COI
				if(flame == 0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
					HAL_Delay(2000);
				}
				if(smoke == 0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
					HAL_Delay(2000);
				}
				if(gas == 0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
					HAL_Delay(2000);
				}
			}	
		}
	else //ho tro nau an
		{
			lcd_put_cur(0,11);
			lcd_send_string("cook"); //COOKING ASSISTANT MODE
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
			HAL_Delay(1000);
			
			if(flame == 0 || smoke == 0 || gas == 0) //phat hien lua, khoi , gas
			{
			lcd_put_cur(0,12);
			lcd_send_string("DTC"); //DETECT-PHAT HIEN
			if(flame == 0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
					HAL_Delay(2000);
				}
				if(smoke == 0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
					HAL_Delay(2000);
				}
				if(gas == 0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); //BAT QUAT
					HAL_Delay(2000);
				}
			}	
		}
	}
	
  }
void readTemp()
{
	var = val*4096/3.3;
	val+=0.1;
	if (val>=5) val=0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	Temp = ((5.0*ADC_VAL/4095 - V25)/Avg_Slope)+25;
	Temp = Temp*4/90+29.0;
	HAL_Delay (100);

}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA8 PA9 PA10
                           PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
