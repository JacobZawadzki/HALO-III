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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_I2C_ADDRESS			0x6b
#define IMU_I2C_ADDRESS_WR		0xD7
#define IMU_WHO_AM_I_REGISTER	0xf
#define IMU_DEVICE_ID			0x70
// IMU Config Registers
#define IMU_Acc_Cntr1			0x10
#define IMU_Gyro_Cntr2			0x11
#define IMU_Status				0x1E
#define IMU_CTRL3				0x12
#define IMU_FUNC_CFG_ACCESS		0x01
#define IMU_CTRL6				0x15
// Static variables
#define GRAVITY 				9.81
#define ACCEL_SENS				0.061
#define GYRO_SENS				4.375


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t init_imu(void);
static float raw_accel[3];
static float raw_ang[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void get_imu_data(void);
void tx_all_data(float *accel, float *angular);
void tx_accel_x_y(float *accel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef status;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t buffer[12];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // IMU Initialization
  uint8_t CHECK = init_imu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (CHECK != HAL_OK) {
		  memset(buffer, 0, sizeof(buffer));
		  strcpy((char*)buffer, "Error Tx\r\n");
		  HAL_UART_Transmit(&huart1, buffer, strlen((char*)buffer), 1000);
	  } else {
		  get_imu_data();
		  //tx_all_data(raw_accel, raw_ang);
		  tx_accel_x_y(raw_accel);
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint8_t init_imu(void)
{
	uint8_t device_id = 0;
	uint8_t cntrl_1 = 0x02, cntrl_2 = 0x02, cntrl_3 = 0xC4;

	status = HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), IMU_WHO_AM_I_REGISTER, 1, &device_id, 1, HAL_MAX_DELAY);
	/* Couldn't detect the IMU over I2C */
	if ((status != HAL_OK))
		return -1;

	/* Detected the IMU on I2C */
	if ((status == HAL_OK) && (device_id == IMU_DEVICE_ID)) {
		HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDRESS_WR, IMU_CTRL3, 1, &cntrl_3, 1, HAL_MAX_DELAY); //reboot mem content
		HAL_Delay(5);
		HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDRESS_WR, IMU_Acc_Cntr1, 1, &cntrl_1, 1, HAL_MAX_DELAY); //enable accelerometer
		HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDRESS_WR, IMU_Gyro_Cntr2, 1, &cntrl_2, 1, HAL_MAX_DELAY); //enable gyro
		return 0;
	}
	return 1;
}

void get_imu_data(void) {
	int16_t raw_accel_ini[3];
	int16_t raw_ang_ini[3];
	uint8_t OUTX_L_G = 0x22, OUTX_L_A = 0x28; //Register addresses - lower 8 bits
	uint8_t OUTX_H_G = 0x23, OUTX_H_A = 0x29; //Register addresses - higher 8 bits
	uint8_t OUTY_L_G = 0x24, OUTY_L_A = 0x2A;
	uint8_t OUTY_H_G = 0x25, OUTY_H_A = 0x2B;
	uint8_t OUTZ_L_G = 0x26, OUTZ_L_A = 0x2C;
	uint8_t OUTZ_H_G = 0x27, OUTZ_H_A = 0x2D;
	uint8_t g_buff[6], a_buff[6], status_buff[8]; //Temp buffers
	uint8_t acc_stat, gyro_stat;

	//Clear Data buffers
	memset(raw_ang_ini, 0x00, 3 * sizeof(int16_t));
	memset(raw_accel_ini, 0x00, 3 * sizeof(int16_t));
	memset(status_buff, 0x00, sizeof(status_buff));
	memset(a_buff, 0x00, sizeof(a_buff));
	memset(g_buff, 0x00, sizeof(g_buff));

	//Read data status buffer
	HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), IMU_Status, 1, status_buff, 1, HAL_MAX_DELAY);

	//Set bits of data flags
	acc_stat = status_buff[0] & 0x01;
	gyro_stat = (status_buff[0] >> 1) & 0x01;

	//Acceleration data
	if (acc_stat == 1) { //If new data ready at accelerometer
		//Read accelerometer X data
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTX_L_A, 1, &a_buff[0], 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTX_H_A, 1, &a_buff[1], 1, HAL_MAX_DELAY);
		//Read accelerometer Y data
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTY_L_A, 1, &a_buff[2], 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTY_H_A, 1, &a_buff[3], 1, HAL_MAX_DELAY);
		//Read accelerometer Z data
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTZ_L_A, 1, &a_buff[4], 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTZ_H_A, 1, &a_buff[5], 1, HAL_MAX_DELAY);

		//Convert 8 bit data to 16 bit buffer
		raw_accel_ini[0] = ((uint16_t) a_buff[1] << 8) | a_buff[0]; // High | Low
		raw_accel_ini[1] = ((uint16_t) a_buff[3] << 8) | a_buff[2];
		raw_accel_ini[2] = ((uint16_t) a_buff[5] << 8) | a_buff[4];

		// If value is negative convert using 2's compliment
		if (raw_accel_ini[0] & 0x8000) {
			uint16_t temp = raw_accel_ini[0];
			temp = -((~temp + 1) & 0x7FFF);
			raw_accel_ini[0] = temp;
		}
		if (raw_accel_ini[1] & 0x8000) {
			uint16_t temp = raw_accel_ini[1];
			temp = -((~temp + 1) & 0x7FFF);
			raw_accel_ini[1] = temp;
		}
		if (raw_accel_ini[2] & 0x8000) {
			uint16_t temp = raw_accel_ini[2];
			temp = -((~temp + 1) & 0x7FFF);
			raw_accel_ini[2] = temp;
		}
	}

	//Gyroscope data
	if (gyro_stat == 1) { //If new data ready at gyro
		//Read gyroscope X data
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTX_L_G, 1, &g_buff[0], 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTX_H_G, 1, &g_buff[1], 1, HAL_MAX_DELAY);
		//Read gyroscope Y data
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTY_L_G, 1, &g_buff[2], 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTY_H_G, 1, &g_buff[3], 1, HAL_MAX_DELAY);
		//Read gyroscope Z data
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTZ_L_G, 1, &g_buff[4], 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, (IMU_I2C_ADDRESS << 1), OUTZ_H_G, 1, &g_buff[5], 1, HAL_MAX_DELAY);

		//Convert 8 bit data to 16 bit buffer
		raw_ang_ini[0] = ((uint16_t) g_buff[1] << 8) | g_buff[0]; // High | Low
		raw_ang_ini[1] = ((uint16_t) g_buff[3] << 8) | g_buff[2];
		raw_ang_ini[2] = ((uint16_t) g_buff[5] << 8) | g_buff[4];

		// If value is negative convert using 2's compliment
		if ((raw_ang_ini[0] & 0x8000)) {
			uint16_t temp = raw_ang_ini[0];
			temp = -((~temp + 1) & 0x7FFF);
			raw_ang_ini[0] = temp;
		}
		if ((raw_ang_ini[1] & 0x8000)) {
			uint16_t temp = raw_ang_ini[1];
			temp = -((~temp + 1) & 0x7FFF);
			raw_ang_ini[1] = temp;
		}
		if ((raw_ang_ini[2] & 0x8000)) {
			uint16_t temp = raw_ang_ini[2];
			temp = -((~temp + 1) & 0x7FFF);
			raw_ang_ini[2] = temp;
		}
	}
	//Convert 16 bit value to mg
	raw_accel[0] = (raw_accel_ini[0] * ACCEL_SENS) / 1000;
	raw_accel[1] = (raw_accel_ini[1] * ACCEL_SENS) / 1000;
	raw_accel[2] = (raw_accel_ini[2] * ACCEL_SENS) / 1000;
	//Convert 16 bit value to m/s^2
	raw_ang[0] = (raw_ang_ini[0] * GYRO_SENS) / 1000;
	raw_ang[1] = (raw_ang_ini[1] * GYRO_SENS) / 1000;
	raw_ang[2] = (raw_ang_ini[2] * GYRO_SENS) / 1000;
}

void tx_all_data(float *accel, float *angular) {
	static uint8_t tx_buffer[100];

	sprintf((char *)tx_buffer, "Accel rate X axis [+/-2g]: %f\r\n", accel[0]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);
	sprintf((char *)tx_buffer, "Accel rate Y axis [+/-2g]: %f\r\n", accel[1]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);
	sprintf((char *)tx_buffer, "Accel rate Z axis [+/-2g]: %f\r\n", accel[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);

	sprintf((char *)tx_buffer, "Angular rate X axis [deg/sec]: %f\r\n", angular[0]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);
	sprintf((char *)tx_buffer, "Angular rate Y axis [deg/sec]: %f\r\n", angular[1]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);
	sprintf((char *)tx_buffer, "Angular rate Z axis [deg/sec]: %f\r\n", angular[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);

	// DEBUG DELAY TO BE ABLE TO READ SERIAL TERMINAL
	HAL_Delay(500);

}

void tx_accel_x_y(float *accel) {
	/*uint8_t buffer[sizeof(float) * 2]; // Byte array to store the first two float values

	// Copy the first two float values to the byte array
	memcpy(buffer, accel, sizeof(float));

	// Transmit the byte array via UART
	HAL_UART_Transmit(&huart1, &buffer[0], sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, &buffer[1], sizeof(float), HAL_MAX_DELAY);*/
	static uint8_t tx_buffer[100];

	sprintf((char *)tx_buffer, "%f,", accel[0]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);
	sprintf((char *)tx_buffer, "%f,", accel[1]);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char const *)tx_buffer), 1000);
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
