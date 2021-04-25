/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *PINOUT: BME280
 *		  SCL PA8
 *		  SDA PA9
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm_bme_280.h"
#include "KEYPAD.h"
#include "KEYPAD_cfg.h"
#include <stdio.h>
#include <string.h>
//#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_1       0
#define KEY_2       1
#define KEY_3       2
#define KEY_F1      3
#define KEY_4       4
#define KEY_5       5
#define KEY_6       6
#define KEY_F2      7
#define KEY_7       8
#define KEY_8       9
#define KEY_9       10
#define KEY_F3      11
#define KEY_A       12
#define KEY_0       13
#define KEY_H       14
#define KEY_F4      15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t pres, temp, humi, alti;
uint16_t dl_kom;
uint8_t received[2];
uint8_t komunikat[50];
BME280_ReadedData_t BME280_Data;
uint16_t correct[4] = { 1, 1, 1, 1 };
uint8_t znak[1];
uint16_t pin[4];
uint8_t pin_counter = 0;
uint16_t message[6];
int ALLOW_FLAG = 0;
//struct lcd_disp disp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SysTick_CallBack(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int checkPin(int a[], int n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t gu8_KeyStatesArr[KEYS] = { 0 };
uint8_t SysTicks = 0;
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
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	BME280_Initial(BME280_STANDBY_MS_1000, BME280_FILTER_OFF,
			BME280_TEMP_OVERSAMPLING_X1, BME280_PRES_OVERSAMPLING_X1,
			BME280_HUM_OVERSAMPLING_X1, BME280_MODE_NORMAL);
	HAL_TIM_Base_Start_IT(&htim7);
//	disp.addr = (0x3F << 1);
//	disp.bl = true;
//	lcd_init(&disp);
//	sprintf((char *)disp.f_line, "To 1. linia");
//	sprintf((char *)disp.s_line, "a to druga linia");
//	lcd_display(&disp);
	HAL_UART_Receive_IT(&huart1, &received, 2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	KEYPAD_Init(0, gu8_KeyStatesArr);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (checkPin(correct, 4) == 1) {
			HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);
			ALLOW_FLAG = 1;
			//		 memset(pin, 0, sizeof pin);
		} else if (checkPin(correct, 4) == 0) {
			HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
			ALLOW_FLAG = 0;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void SysTick_CallBack(void) {
	SysTicks++;
	if (SysTicks == 10) // Each 5msec
			{
		KEYPAD_Scan(0);
		SysTicks = 0;
	}
}

void SysTick_Handler(void) {
	HAL_IncTick();
	SysTick_CallBack();
}

int checkPin(int a[], int n) {
	if (pin_counter < n) {

		if (gu8_KeyStatesArr[KEY_1] == KEY_PRESSED) {
			//			HAL_GPIO_TogglePin(GPIOG, LD3_Pin);
			HAL_Delay(200);
			sprintf(znak, "1\r\n");
			//		HAL_UART_Transmit(&huart1, znak, 3, 100);
			HAL_UART_Transmit_IT(&huart1, &znak, 1);
			pin[pin_counter] = 1;
			pin_counter++;
		} else if (gu8_KeyStatesArr[KEY_2] == KEY_PRESSED) {
			//			HAL_GPIO_TogglePin(GPIOG, LD4_Pin);
			sprintf(znak, "2\r\n");
			//		HAL_UART_Transmit(&huart1, znak, 3, 100);
			HAL_UART_Transmit_IT(&huart1, &znak, 1);
			pin[pin_counter] = 2;

			HAL_Delay(200);
			pin_counter++;
		} else if (gu8_KeyStatesArr[KEY_3] == KEY_PRESSED) {
//				sprintf(znak, "3\r\n");
			//		HAL_UART_Transmit(&huart1, znak, 3, 100);
//				HAL_UART_Transmit_IT(&huart1, &znak, 1);
			pin[pin_counter] = 3;
			HAL_Delay(200);
			pin_counter++;
		} else if (gu8_KeyStatesArr[KEY_F1] == KEY_PRESSED) {

			//		HAL_UART_Transmit(&huart1, znak, 3, 100);
			HAL_UART_Transmit_IT(&huart1, &message, 4);
			pin_counter = 0;
			HAL_Delay(200);
			memset(pin, 0, sizeof pin);
		} else if (gu8_KeyStatesArr[KEY_4] == KEY_PRESSED) {

			sprintf(message, "%d%d%d%d\r\n", correct[0], correct[1], correct[2],
					correct[3]);
			HAL_UART_Transmit_IT(&huart1, &message, 4);
			pin_counter = 0;
			HAL_Delay(200);
			memset(pin, 0, sizeof pin);
		} else if (pin_counter == 1) {
//			HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
		}
	}

	else {

		if (memcmp(pin, a, sizeof(pin)) == 0) {
//				pin_counter= 0;
			return 1;
		}
//			sprintf(message, "%d%d%d%d\r\n", pin[0], pin[1], pin[2], pin[3]);
		else {
			dl_kom = sprintf(komunikat, "Wrong PIN\r\n");
			HAL_UART_Transmit_IT(&huart1, &komunikat, dl_kom);
		}

//			HAL_Delay(200);
		pin_counter = 0;
		HAL_GPIO_TogglePin(GPIOG, LD4_Pin);
		memset(pin, 0, sizeof pin);
		return 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
		if (ALLOW_FLAG == 1) {

			BME280_Data.temp = BME280_ReadTemperature();
			BME280_Data.pres = BME280_ReadPressure();
			BME280_Data.humi = BME280_ReadHumidity();
			BME280_Data.alti = BME280_ReadAltitude(kSEA_LEVEL_PRESURE_PA);

			dl_kom = sprintf(komunikat,
					"T: %0.2f, P: %0.2f, H: %0.2f, A: %0.2f\r\n",
					BME280_Data.temp, BME280_Data.pres, BME280_Data.humi,
					BME280_Data.alti);
//		printf("T: %0.2f, P: %0.2f, H: %0.2f, A: %0.2f \r\n", BME280_Data.temp, BME280_Data.pres, BME280_Data.humi, BME280_Data.alti);
			HAL_UART_Transmit_IT(&huart1, komunikat, dl_kom);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (ALLOW_FLAG == 1) {
			if (strchr(received, 'r') != NULL) {
				HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
				dl_kom = sprintf(komunikat, "RED LIGHT");
			} else if (strchr(received, 'g') != NULL) {
				HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_SET);
				dl_kom = sprintf(komunikat, "GREEN LIGHT");
			} else if (strchr(received, 'e') != NULL) {
				HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);
				dl_kom = sprintf(komunikat, "GREEN LIGHT");
			} else if (strchr(received, 'f') != NULL) {
				HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_RESET);
				dl_kom = sprintf(komunikat, "GREEN LIGHT");
			}
			else if (strchr(received, 'd') != NULL) {
				switch (received[1]) {
				case 48:
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);

					break;
				case 49:
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 50);

					break;
				case 50:
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 100);
					break;
				case 51:
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 150);
					break;
				case 52:
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 200);
					break;
				}

				HAL_GPIO_TogglePin(GPIOG, LD3_Pin);
				dl_kom = sprintf("%d", &received[1]);
			}
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
		}
			HAL_UART_Receive_IT(&huart1, &received, 2);
//		HAL_UART_Transmit_IT(&huart1, &komunikat, dl_kom);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		HAL_GPIO_TogglePin(GPIOG, LD3_Pin);
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
		memset(pin, 0, sizeof pin);
	}
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == GPIO_PIN_0) {
//		HAL_GPIO_TogglePin(GPIOG, LD3_Pin);
////		BME280_mode_E sensMode = BME280_MODE_FORCED_1;
////		BME280_Initial(BME280_STANDBY_MS_1000, BME280_FILTER_OFF, BME280_TEMP_OVERSAMPLING_X1, BME280_PRES_OVERSAMPLING_X1, BME280_HUM_OVERSAMPLING_X1, BME280_MODE_FORCED_1);
//
//		BME280_Data.temp = BME280_ReadTemperature();
//		BME280_Data.pres = BME280_ReadPressure();
//		BME280_Data.humi = BME280_ReadHumidity();
//		BME280_Data.alti = BME280_ReadAltitude(kSEA_LEVEL_PRESURE_PA);
//
//		dl_kom = sprintf(komunikat, "T: %0.2f, P: %0.2f, H: %0.2f, A: %0.2f\r\n",
//				BME280_Data.temp, BME280_Data.pres, BME280_Data.humi,
//				BME280_Data.alti);
////		printf("T: %0.2f, P: %0.2f, H: %0.2f, A: %0.2f \r\n", BME280_Data.temp,
////				BME280_Data.pres, BME280_Data.humi, BME280_Data.alti);
////		HAL_UART_Transmit_IT(&huart1, komunikat, dl_kom);
//		HAL_UART_Transmit(&huart1, komunikat, dl_kom, 100);
//
////		sprintf((char*) disp.f_line, "Siemano");
////		lcd_display(&disp);
//	}
//}

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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
