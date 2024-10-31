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
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GetLedConfig(int ledIndex, GPIO_TypeDef **port, uint16_t *pin);
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
  MX_USART2_UART_Init();
  int x=0;
  int ledCount = 8; // Total number of LEDs connected
  int delayTime = 500;
  int reduceTime=0;
  int points[1]={0,0};
  bool point = false;
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
      {
          // Forward LED sequence
          for (int i = 0; i < ledCount; i++)
          {
              GPIO_TypeDef *currentPort;
              uint16_t currentPin;
              GetLedConfig(i, &currentPort, &currentPin);

              HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_SET);
              HAL_Delay(delayTime - reduceTime);
              HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_RESET);
          }

          point = true;
          for (int i = 0; i <= delayTime - reduceTime; i++)
          {
              HAL_Delay(1);
              if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET)
              {
                  point = false;
                  break;
              }
          }
          if (point)
          {
        	  if(points[0]>4){
        		  for (int i = ledCount - 1; i >= 0; i--)
				   {
					   GPIO_TypeDef *currentPort;
					   uint16_t currentPin;
					   GetLedConfig(i, &currentPort, &currentPin);
					   HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_SET);
				   }
        		  HAL_Delay(1000);
        		  for (int i = ledCount - 1; i >= 0; i--)
				   {
					   GPIO_TypeDef *currentPort;
					   uint16_t currentPin;
					   GetLedConfig(i, &currentPort, &currentPin);
					   HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_RESET);
				   }
        		  HAL_Delay(1000);

		  }
              reduceTime = 0;
              continue;
          }

          // Reverse LED sequence
          for (int i = ledCount - 1; i >= 0; i--)
          {
              GPIO_TypeDef *currentPort;
              uint16_t currentPin;
              GetLedConfig(i, &currentPort, &currentPin);

              HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_SET);
              HAL_Delay(delayTime - reduceTime);
              HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_RESET);
          }

          point = true;
          for (int i = 0; i <= delayTime - reduceTime; i++)
          {
              HAL_Delay(1);
              if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET)
              {
                  point = false;
                  break;
              }
          }
          if (point)
          {
              reduceTime = 0;
              continue;
          }

          if (delayTime - reduceTime > 20)
          {
              reduceTime += 10;
          }
      }
}

void GetLedConfig(int ledIndex, GPIO_TypeDef **port, uint16_t *pin)
{
    switch (ledIndex)
    {
        case 0:
            *port = GPIOB;
            *pin = GPIO_PIN_1;
            break;
        case 1:
            *port = GPIOB;
            *pin = GPIO_PIN_2;
            break;
        case 2:
            *port = GPIOB;
            *pin = GPIO_PIN_11;
            break;
        case 3:
            *port = GPIOB;
            *pin = GPIO_PIN_12;
            break;
        case 4:
            *port = GPIOA;
            *pin = GPIO_PIN_11;
            break;
        case 5:
            *port = GPIOA;
            *pin = GPIO_PIN_12;
            break;
        case 6:
            *port = GPIOC;
            *pin = GPIO_PIN_5;
            break;
        case 7:
            *port = GPIOC;
            *pin = GPIO_PIN_6;
            break;
        default:
            *port = NULL;
            *pin = 0;
            break;
    }
}

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
