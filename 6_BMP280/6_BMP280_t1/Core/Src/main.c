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
#include "i2c.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bmp280.h"
#include <stdio.h>
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

/* USER CODE BEGIN PV */

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

char msg[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // char msg[64];

// sprintf(msg, "I2C Scan Start...\r\n");
// HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

// for(uint8_t addr=1; addr<128; addr++)
// {
//     if(HAL_I2C_IsDeviceReady(&hi2c1, addr<<1, 1, 10) == HAL_OK)
//     {
//         sprintf(msg, "Found device: 0x%X\r\n", addr);
//         HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//     }
// }

// sprintf(msg, "Scan Done\r\n");
// HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

  uint8_t id = BMP280_ReadID(&hi2c1);

  sprintf(msg, "BMP280 ID: 0x%X\r\n", id);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

  if(id != 0x58)
    {
        sprintf(msg, "BMP280 NOT FOUND!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }

  BMP280_Init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float temperature, pressure, altitude;
  int32_t temp_int, temp_frac;
  int32_t press_int, press_frac;
  int32_t alt_int, alt_frac;

  // 初始化完成后，传感器结构体中已包含第一次读数作为IIR滤波器的“种子”
  // 我们直接将这个初始海拔设为基准零点
  BMP280_Set_Base_Altitude(&hi2c1);

  while (1)
  {
    BMP280_Read_Float(&hi2c1, &temperature, &pressure, &altitude);

    // 将温度浮点数拆分为整数和小数部分
    temp_int = (int32_t)temperature;
    temp_frac = (int32_t)((temperature - temp_int) * 100);
    if (temp_frac < 0) {
        temp_frac = -temp_frac; // 处理负温度的小数部分
    }

    // 将气压浮点数拆分为整数和小数部分
    press_int = (int32_t)pressure;
    press_frac = (int32_t)((pressure - press_int) * 100);

    // 将海拔浮点数拆分为整数和小数部分
    alt_int = (int32_t)altitude;
    alt_frac = (int32_t)((altitude - alt_int) * 100);
    if (alt_frac < 0) {
        alt_frac = -alt_frac;
    }

    sprintf(msg, "Temp: %ld.%02ld C, Press: %ld.%02ld Pa, Rel. Alt: %ld.%02ld m\r\n", temp_int, temp_frac, press_int, press_frac, alt_int, alt_frac);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(1000);
    // HAL_Delay(1000);
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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