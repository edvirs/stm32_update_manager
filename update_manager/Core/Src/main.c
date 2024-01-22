/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DATA_CHUNK_SIZE	64
#define BOOT_POINTER_ADRESS	(0x08060000U)
#define PAGE_ZERO_ADRESS (0x08000000U)
// #define update_adress 0x08010000U

#define RECEIVE_ADRESS_CMD		(0xFFU) //as a page number
#define RECEIVE_SIZE_n_ERASE 	(0xFAU) //size as amount of pages
#define RECEIVE_DATA_CMD		(0xAFU)
#define FINISH_UPDATE_CMD		(0xAAU)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t run = 1;

uint8_t cmd_buffer[] = {0x00};
uint8_t adress_buffer[1];
uint8_t size_buffer[1];
uint8_t data_buffer[DATA_CHUNK_SIZE];

uint32_t update_adress = (0x00U);
uint32_t write_pointer_adress_offset = (0x00U);

void blink(uint8_t count){
	for(int i = 0; i < count; i++){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		HAL_Delay(500);
	}
}

void EraseFlash(uint32_t pageNumber, uint8_t sizePages){
	FLASH_EraseInitTypeDef FLASH_EraseInitStruct;

	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	FLASH_EraseInitStruct.Page = pageNumber;
	FLASH_EraseInitStruct.NbPages = sizePages;
	FLASH_EraseInitStruct.Banks = FLASH_BANK_1;

//	if (adress_buffer[0] <= )
//		FLASH_EraseInitStruct.Banks = bank;

	uint32_t FlashEraseFault = 0;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&FLASH_EraseInitStruct,&FlashEraseFault);
	HAL_FLASH_Lock();
}

void FlashWriteChunk64(uint32_t* adress, uint8_t* pData){
	uint64_t doubleword = 0;
	HAL_StatusTypeDef status;

	for(uint8_t i = 0; i < 64; i+=8){
		doubleword = ((uint64_t)(pData[i+7]) << 56)
                 | ((uint64_t)(pData[i+6]) << 48)
                 | ((uint64_t)(pData[i+5]) << 40)
                 | ((uint64_t)(pData[i+4]) << 32)
                 | ((uint64_t)(pData[i+3]) << 24)
                 | ((uint64_t)(pData[i+2]) << 16)
                 | ((uint64_t)(pData[i+1]) << 8)
                 | ((uint64_t)(pData[i]));


		HAL_FLASH_Unlock();
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, update_adress + write_pointer_adress_offset, doubleword);
		HAL_FLASH_Lock();
		write_pointer_adress_offset += 0x08;
	}
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){}

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

  //HAL_UART_Transmit(&huart2, &ack_byte, 1, 1);
  //blink(3);

  //HAL_Delay(1000);
  blink(2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (run)
  {
	//blink(1);
	HAL_UART_Receive(&huart2, cmd_buffer, 1, 1000000);

	switch (cmd_buffer[0]){

		case RECEIVE_ADRESS_CMD:
			HAL_UART_Receive(&huart2, adress_buffer, 1, 10000);
			update_adress = PAGE_ZERO_ADRESS + (adress_buffer[0] * 0x800);
			break;

		case RECEIVE_SIZE_n_ERASE:
			HAL_UART_Receive(&huart2, size_buffer, 1, 1000000);
			EraseFlash(adress_buffer[0], size_buffer[0]);

			break;

		case RECEIVE_DATA_CMD:

			HAL_UART_Receive(&huart2, data_buffer, DATA_CHUNK_SIZE, 10000000);

//			HAL_StatusTypeDef status;
//			HAL_FLASH_Unlock();
//			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, update_adress + write_pointer_adress_offset, *data_buffer);
//			HAL_FLASH_Lock();
//			write_pointer_adress_offset += 0x40;

			FlashWriteChunk64(&update_adress, data_buffer);


			HAL_UART_Transmit(&huart2, data_buffer, DATA_CHUNK_SIZE, 100);

			break;

		case FINISH_UPDATE_CMD:
			HAL_StatusTypeDef status;

			EraseFlash(0xC0, 1);
			HAL_FLASH_Unlock();
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_POINTER_ADRESS, (uint64_t)update_adress);
			//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_POINTER_ADRESS, (uint64_t) 0x08008000);
			HAL_FLASH_Lock();

			run = 0;

			break;

		default:
			break;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  blink(5);
  NVIC_SystemReset();
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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
