/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
void GetLedConfig(int ledIndex, GPIO_TypeDef **pLedPort, uint16_t *pLedPin);
void EndGame(int playerScores[2]);
void ShowPlayerScores(int playerScores[2]);
int CheckButtonPress();
int ExecuteLedSequence(int ledTotal, int delayDuration, int speedIncrease, bool sequenceForward);
void ResetAllLeds(int ledTotal);
bool PerformEndpointAction(GPIO_TypeDef *pLedPort, uint16_t ledPin, GPIO_TypeDef *pButtonPort, uint16_t buttonPin, int delayDuration, int speedIncrease);
void IncrementSpeed(int* pSpeedIncrease, int delayDuration);
void TurnOnAllLeds(int ledTotal);
void ResetForNewRound(int ledTotal, GPIO_TypeDef *pStartPort, uint16_t startPin, int delayDuration, int speedIncrease);
void UpdatePlayerScore(int playerScores[2], int currentScore, int* pSpeedIncrease, bool* pWaitToStart);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  int ledTotal = 8;
  int delayDuration = 200;
  int speedIncrease = 0;
  int playerScores[2] = {0, 0};
  bool leftServe = true;
  bool waitToStart = true;

  while (1)
  {
      if (playerScores[0] == 0 && playerScores[1] == 0 && waitToStart == true) {
          while (1) {
              waitToStart = false;
              int serveReady = CheckButtonPress();
              if (serveReady != 0) {
                  leftServe = (serveReady == 2);
                  break;
              }
              HAL_Delay(10);
          }
      }

      ResetForNewRound(ledTotal, leftServe ? LED1_GPIO_Port : LED8_GPIO_Port, leftServe ? LED1_Pin : LED8_Pin, delayDuration, speedIncrease);

      int currentState = ExecuteLedSequence(ledTotal, delayDuration, speedIncrease, leftServe);

      if (currentState != 0 || PerformEndpointAction(leftServe ? LED8_GPIO_Port : LED1_GPIO_Port,
                                                     leftServe ? LED8_Pin : LED1_Pin,
                                                     leftServe ? R_button_GPIO_Port : L_button_GPIO_Port,
                                                     leftServe ? R_button_Pin : L_button_Pin,
                                                     delayDuration, speedIncrease)) {
          leftServe = !leftServe;
          UpdatePlayerScore(playerScores, (currentState != 0) ? currentState - 1 : 0, &speedIncrease, &waitToStart);
          continue;
      }

      currentState = ExecuteLedSequence(ledTotal, delayDuration, speedIncrease, !leftServe);

      if (currentState != 0 || PerformEndpointAction(leftServe ? LED1_GPIO_Port : LED8_GPIO_Port,
                                                     leftServe ? LED1_Pin : LED8_Pin,
                                                     leftServe ? L_button_GPIO_Port : R_button_GPIO_Port,
                                                     leftServe ? L_button_Pin : R_button_Pin,
                                                     delayDuration, speedIncrease)) {
          leftServe = !leftServe;
          UpdatePlayerScore(playerScores, (currentState != 0) ? currentState - 1 : 1, &speedIncrease, &waitToStart);
          continue;
      }

      IncrementSpeed(&speedIncrease, delayDuration);
  }
}

void ResetForNewRound(int ledTotal, GPIO_TypeDef *pStartPort, uint16_t startPin, int delayDuration, int speedIncrease) {
    ResetAllLeds(ledTotal);
    HAL_GPIO_WritePin(pStartPort, startPin, GPIO_PIN_SET);
    HAL_Delay(delayDuration - speedIncrease);
    HAL_GPIO_WritePin(pStartPort, startPin, GPIO_PIN_RESET);
}

void IncrementSpeed(int* pSpeedIncrease, int delayDuration) {
    if (delayDuration - *pSpeedIncrease > 40) {
        *pSpeedIncrease += 20;
    } else if (delayDuration - *pSpeedIncrease > 20) {
        *pSpeedIncrease += 10;
    } else if (delayDuration - *pSpeedIncrease > 5) {
        *pSpeedIncrease += 2;
    }
}

bool PerformEndpointAction(GPIO_TypeDef *pLedPort, uint16_t ledPin, GPIO_TypeDef *pButtonPort, uint16_t buttonPin, int delayDuration, int speedIncrease) {
    HAL_GPIO_WritePin(pLedPort, ledPin, GPIO_PIN_SET);

    for (int i = 0; i <= (delayDuration - speedIncrease) * 10; i++) {
        HAL_Delay(1);

        if (HAL_GPIO_ReadPin(pButtonPort, buttonPin) == GPIO_PIN_RESET) {
            while (HAL_GPIO_ReadPin(pButtonPort, buttonPin) == GPIO_PIN_RESET);
            HAL_GPIO_WritePin(pLedPort, ledPin, GPIO_PIN_RESET);
            return false;
        }
    }

    HAL_GPIO_WritePin(pLedPort, ledPin, GPIO_PIN_RESET);
    return true;
}

void ResetAllLeds(int ledTotal) {
    for (int i = 0; i < ledTotal; i++) {
        GPIO_TypeDef *pCurrentPort;
        uint16_t currentPin;
        GetLedConfig(i, &pCurrentPort, &currentPin);
        HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_RESET);
    }
}

void TurnOnAllLeds(int ledTotal) {
    for (int i = 0; i < ledTotal; i++) {
        GPIO_TypeDef *pCurrentPort;
        uint16_t currentPin;
        GetLedConfig(i, &pCurrentPort, &currentPin);
        HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_SET);
    }
}

void UpdatePlayerScore(int playerScores[2], int currentScore, int* pSpeedIncrease, bool* pWaitToStart) {
    playerScores[currentScore]++;
    if (playerScores[currentScore] >= 4) {
        playerScores[0] = 0;
        playerScores[1] = 0;
        *pWaitToStart = true;
    }
    EndGame(playerScores);
    *pSpeedIncrease = 0;
}

void EndGame(int playerScores[2]) {
    for (int i = 0; i < 4; i++) {
        TurnOnAllLeds(8);
        HAL_Delay(500);
        ResetAllLeds(8);
        HAL_Delay(500);
    }
    ShowPlayerScores(playerScores);

    while (1) {
        if (HAL_GPIO_ReadPin(R_button_GPIO_Port, R_button_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(L_button_GPIO_Port, L_button_Pin) == GPIO_PIN_RESET) {
            break;
        }
        HAL_Delay(10);
    }
}

int CheckButtonPress() {
    if (HAL_GPIO_ReadPin(R_button_GPIO_Port, R_button_Pin) == GPIO_PIN_RESET) {
        return 1;
    } else if (HAL_GPIO_ReadPin(L_button_GPIO_Port, L_button_Pin) == GPIO_PIN_RESET) {
        return 2;
    }
    return 0;
}

int ExecuteLedSequence(int ledTotal, int delayDuration, int speedIncrease, bool sequenceForward) {
    int checkStatus = 0;
    if (sequenceForward) {
        for (int i = 1; i < ledTotal - 1; i++) {
            checkStatus = CheckButtonPress();
            if (checkStatus != 0 && i < ledTotal - 1) {
                return checkStatus;
            }
            GPIO_TypeDef *pCurrentPort;
            uint16_t currentPin;
            GetLedConfig(i, &pCurrentPort, &currentPin);
            HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_SET);
            HAL_Delay(delayDuration - speedIncrease);
            HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_RESET);
        }
    } else {
        for (int i = ledTotal - 2; i >= 1; i--) {
            checkStatus = CheckButtonPress();
            if (checkStatus != 0) {
                return checkStatus;
            }
            GPIO_TypeDef *pCurrentPort;
            uint16_t currentPin;
            GetLedConfig(i, &pCurrentPort, &currentPin);
            HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_SET);
            HAL_Delay(delayDuration - speedIncrease);
            HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_RESET);
        }
    }
    return 0;
}

void ShowPlayerScores(int playerScores[2]) {
    ResetAllLeds(8);
    for (int i = 0; i < playerScores[0]; i++) {
        GPIO_TypeDef *pCurrentPort;
        uint16_t currentPin;
        GetLedConfig(i, &pCurrentPort, &currentPin);
        HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_SET);
    }
    for (int i = 7; i >= 8 - playerScores[1] && i >= 0; i--) {
        GPIO_TypeDef *pCurrentPort;
        uint16_t currentPin;
        GetLedConfig(i, &pCurrentPort, &currentPin);
        HAL_GPIO_WritePin(pCurrentPort, currentPin, GPIO_PIN_SET);
    }
}

void GetLedConfig(int ledIndex, GPIO_TypeDef **pLedPort, uint16_t *pLedPin)
{
    switch (ledIndex) {
        case 0: *pLedPort = LED1_GPIO_Port; *pLedPin = LED1_Pin; break;
        case 1: *pLedPort = LED2_GPIO_Port; *pLedPin = LED2_Pin; break;
        case 2: *pLedPort = LED3_GPIO_Port; *pLedPin = LED3_Pin; break;
        case 3: *pLedPort = LED4_GPIO_Port; *pLedPin = LED4_Pin; break;
        case 4: *pLedPort = LED5_GPIO_Port; *pLedPin = LED5_Pin; break;
        case 5: *pLedPort = LED6_GPIO_Port; *pLedPin = LED6_Pin; break;
        case 6: *pLedPort = LED7_GPIO_Port; *pLedPin = LED7_Pin; break;
        case 7: *pLedPort = LED8_GPIO_Port; *pLedPin = LED8_Pin; break;
        default: *pLedPort = LED1_GPIO_Port; *pLedPin = LED1_Pin; break;
    }
}

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
