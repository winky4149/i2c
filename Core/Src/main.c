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
#include "uart.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define memTemperatureBase	  0
#define memTemperatureSize 		4
#define memC02Base (memTemperatureBase+memTemperatureSize)
#define memC02Size 4
#define memDeviceName (memC02Base+memC02Size)
#define memDeviceNameSize 20



void writeEeprom(uint16_t address, uint8_t data)
{
	uint8_t txBuffer[3];
	txBuffer[0]=address >>8;
	txBuffer[1]=address & 0x00ff;
	txBuffer[2]=data;
	HAL_I2C_Master_Transmit(&hi2c1, 0xA0, txBuffer, sizeof(txBuffer), 1);
	HAL_Delay(5);
}

uint8_t readEeprom(uint16_t address)
{
	uint8_t txBuffer[2];
	uint8_t rxBuffer;
	txBuffer[0]=address >>8;
	txBuffer[1]=address & 0x00ff;
	HAL_I2C_Master_Transmit(&hi2c1, 0xA0, txBuffer, sizeof(txBuffer), 1);
	HAL_I2C_Master_Receive(&hi2c1, 0xA0, &rxBuffer, 1, 1);
	return rxBuffer;
}

void writeEeprom2(uint16_t address,uint8_t data)//another way(write)
{
	HAL_I2C_Mem_Write(&hi2c1, 0xA0, address, 2, &data, 1, 1);
}

uint8_t readEeprom2(uint16_t address)//another way(read)
{
	uint8_t rxBuffer;
	HAL_I2C_Mem_Read(&hi2c1, 0xA0, address, 2, &rxBuffer, 1, 1);
	return rxBuffer;

}

typedef struct{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}Datetime_t;

void writeRtc(Datetime_t dateTime){
	uint8_t txBuffer[7];
	memcpy(txBuffer,&dateTime,7);
	HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0, 1, &dateTime, 7, 1);//알아서 구조체�?� 값들�?� 차례로 저장�?�다.
}

void readRtc(){
	Datetime_t result;
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0, 1, &result, 7, 1);
	return result;
}

void writeEeprom4Byte(uint16_t address,uint32_t data)//32bit짜리 �?��?�터를 8bit로 쪼개서 i2c통신�? �?용
{
	uint8_t buffer[4];
	memcpy(buffer,&data,4);//instead of bit shifting
	/*buffer[0]=data>>24;
	buffer[1]=data>>16;
	buffer[2]=data>>8;
	buffer[3]=data;*/
	writeEeprom(address,buffer[0]);
	writeEeprom(address+1,buffer[1]);
	writeEeprom(address+2,buffer[2]);
	writeEeprom(address+3,buffer[3]);
}

uint32_t readEeprom4Byte(uint16_t address)//i2c를 통해 받�?� 8bit�?��?�터를 32비트로 합�?서 �?��?�
{
	uint8_t rxBuffer[4];
	uint32_t result;
	rxBuffer[0]=readEeprom(address);
	rxBuffer[1]=readEeprom(address+1);
	rxBuffer[2]=readEeprom(address+2);
	rxBuffer[3]=readEeprom(address+3);
	memcpy(&result,rxBuffer,4);
	return result;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("I2C Scanner\n");
  //reset
HAL_GPIO_WritePin(OledReset_GPIO_Port, OledReset_Pin, 0);
HAL_Delay(1);
HAL_GPIO_WritePin(OledReset_GPIO_Port, OledReset_Pin, 1);

  for(int address=0;address<256;address++)
  {
  	int result= HAL_I2C_IsDeviceReady(&hi2c1, address, 0, 1);
  	if(result==HAL_OK){
  	printf("%02x = %d\n",address,result);
  	}
  }


 /* writeEeprom4Byte(memTemperatureBase, 0x12345678);
  writeEeprom4Byte(memTemperatureBase, 0x87654321);*/

/*  //for write to EEPROM
  uint8_t txBuffer[3];
  txBuffer[0]=upperAddress;
  txBuffer[1]=lowerAddress;
  txBuffer[2]=data;
HAL_I2C_Master_Transmit(&hi2c1, 0x7f, txBuffer, 3, 1);

//for read to EEPROM
uint8_t txBuffer[2];
uint8_t rxBuffer;
txBuffer[0]=upperAddress;
txBuffer[1]=lowerAddress;
HAL_I2C_Master_Transmit(&hi2c1, 0x7f, txBuffer, 2, 1);
HAL_I2C_Master_Receive(&hi2c1, 0x7f, &rxBuffer, 1, 1);*/

  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OledReset_GPIO_Port, OledReset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OledReset_Pin */
  GPIO_InitStruct.Pin = OledReset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OledReset_GPIO_Port, &GPIO_InitStruct);

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
