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
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG_EN
#define APPLICATION_LOCATION 		0x08008000U
#define RX_Length  					200U


#define BOOTLOADER_VERSION          1
#define FW_VERSION					2



#define AC 							0xAA
#define NAC 						0xCC



#define FW_VERSION					0x01U
#define CHIP_ID						0x02U





#define VERIFY_CRC_SUCCESS 		    0U

#define VERIFY_CRC_FAIL             1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void print(char *format, ...);
static void Application(void);
uint8_t verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host);
/*
 * function specific to bootloader
 */


void BootLoader_Rx(void);

void send_firmware_and_bl_details(uint8_t* UART_RX_BUFFER);
void send_chipId(uint8_t* UART_RX_BUFFER);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uart2[] = "Hello from debug\r\n";


uint8_t UART_RX_BUFFER[RX_Length];
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
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  print("HELLO");
	  HAL_Delay(1000);
	  int t = 0;
	  while(t<5){
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(100);
		  t++;
	  }
	  Application();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print(char *format, ...) {
#ifdef BL_DEBUG_MSG_EN
	char str[80];

	/* Extract the argument list using VA APIs */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}




void Application(){

	//TODO: dont forget to shift the vector table because it starts from 0x8000_0000 but we want to start it according to our application location

	  void (*pfunc_to_hold_reset_handler_addr)(void);
	  HAL_UART_Transmit(&huart2, (uint8_t*) uart2 , sizeof(uart2), HAL_MAX_DELAY);

	  //we have to transfer value of msp into application area
	  /*
	   * first location is for msp which was 0x8000_0000 and now for application
	   * it will become start addr of application
	   */
		uint32_t msp_value = *(volatile uint32_t*)APPLICATION_LOCATION;
	  __set_MSP(msp_value);

	  /*
	   * Second location will store the reset handler
	   *
	   */
	  uint32_t resethandler_address = *(volatile uint32_t*)(APPLICATION_LOCATION + 4);

	  pfunc_to_hold_reset_handler_addr = (void*)resethandler_address;

	  pfunc_to_hold_reset_handler_addr(); // going to reset handler
}



void BootLoader_Rx(void)
{

	size_t rx_size; // it will always equal to the amount of data we have to recieve
	while(1){
		memset(UART_RX_BUFFER , 0 , 200); // Make everything '0' first

		//read uart

		HAL_UART_Receive(&huart2, UART_RX_BUFFER, 1, HAL_MAX_DELAY); // here we will read the length first
		rx_size = UART_RX_BUFFER[0];
		HAL_UART_Receive(&huart2, &UART_RX_BUFFER[1], rx_size, HAL_MAX_DELAY); // read the remaining bytes!!


		switch(UART_RX_BUFFER[1])
		{
		case FW_VERSION:
			send_firmware_and_bl_details(UART_RX_BUFFER);
			break;
		case CHIP_ID:
			send_chipId(UART_RX_BUFFER);
			break;

		default:
			  HAL_UART_Transmit(&huart2, (uint8_t*)"Send appropiate command\r\n" ,25, HAL_MAX_DELAY);
			  break;


		}


	}

}


void send_firmware_and_bl_details(uint8_t* UART_RX_BUFFER)
{
	// check for crc not needed for demontration but still gonna use

	size_t version;
	print("Entered in bootloader mode 1. \n Command recieved 0x01");

	uint32_t command_packet_len = UART_RX_BUFFER[0] + 1;

	print("DOING CHECKSUM");

	uint32_t host_crc = *((uint32_t *) (UART_RX_BUFFER + command_packet_len - 4));

	if(!verify_crc(&UART_RX_BUFFER[0],command_packet_len - 4, host_crc)){

		print("CHECKSUM done ......");
		print("Bootloader version :  ");
		HAL_UART_Transmit(&huart2, (uint8_t*) BOOTLOADER_VERSION , 1, HAL_MAX_DELAY);
		print("Firmware version :  ");
		HAL_UART_Transmit(&huart2, (uint8_t*) FW_VERSION , 1, HAL_MAX_DELAY);
	}
	else{

		HAL_UART_Transmit(&huart2, (uint8_t*) NAC , 1, HAL_MAX_DELAY);

	}


}












uint8_t verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host) {

	uint32_t uwCRCValue = 0xFF;

	for(uint32_t i = 0; i < len; i++) {
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	/* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);

	if(uwCRCValue == crc_host) {
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
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
