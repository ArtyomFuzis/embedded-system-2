/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kb.h"
#include "sdk_uart.h"
#include "pca9538.h"
#include "oled.h"
#include "fonts.h"
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
//void KB_Test( int* timer_second, int8_t* cur_key, char* last_buffer, enum TimerState* state);
//void OLED_KB( uint8_t OLED_cur_keys[]);
void oled_Reset( void );
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  oled_Init();

  /* USER CODE END 2 */
 
 

  int timer_second = 0;
  uint8_t key[4] = {0x0, 0x0, 0x0, 0x0};
  char last_buffer[4] = {0x0, 0x0, 0x0, 0x0};
  enum TimerState state = SETTING;
  int func_tics = 0;
  while (1)
  {

	  KB_Test(&timer_second, key, last_buffer, &state, &func_tics);
	  HAL_Delay(10);

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


void update_time(char* buf, char* last_buffer){
	int changed = 0;
	for(int i = 0 ; i < 4 ; i++){
		if (buf[i] != last_buffer[i]){
			changed = 1;
			break;
		}
	}
	if (changed != 0){
		oled_Reset();
		oled_SetCursor(47, 30);
		oled_WriteChar(buf[0],  Font_7x10, White);
		oled_WriteChar(buf[1],  Font_7x10, White);
		oled_WriteChar(':',  Font_7x10, White);
		oled_WriteChar(buf[2],  Font_7x10, White);
		oled_WriteChar(buf[3],  Font_7x10, White);

		oled_UpdateScreen();
		for(int i = 0 ; i < 4 ; i++){
			last_buffer[i] = buf[i];
		}
	}
}

void build_timer_buffer(int* timer_second, char* res){
	if ((*timer_second) < 0) {
		(*timer_second) = 0;
	}
	else if ((*timer_second) >= 60*60) {
		(*timer_second) = 60*60 - 1;
	}

	int minutes = (*timer_second) / 60;
	int second = (*timer_second) % 60;

	int second_01 = second % 10;
	int second_02 = second / 10;

	int minutes_01 = minutes % 10;
	int minutes_02 = minutes / 10;

	res[3] = '0' + second_01;
	res[2] = '0' + second_02;
	res[1] = '0' + minutes_01;
	res[0] = '0' + minutes_02;
}

void start_timer(int* second){

}

/* USER CODE BEGIN 4 */
void KB_Test( int* timer_second, int8_t* key, char* last_buffer, enum TimerState* state, int* func_tics) {
	uint8_t Row[4] = {ROW1, ROW2, ROW3, ROW4};
	uint8_t cur_key;
	char buffer[4];

	build_timer_buffer(timer_second, buffer);
	update_time(buffer, last_buffer);

	(*func_tics)++;
	if((*func_tics) == 68)*func_tics = 1;

	if (*state == SETTING){
		cur_key = Check_Row(Row[0]);
		if(cur_key != key[0]){
			if (cur_key == 0x04) {
				(*timer_second)++;
			} else if ( cur_key == 0x02) {
				(*timer_second)--;
			} else if ( cur_key == 0x01) {
				(*state) = COUNTDOWN;
			}
			key[0] = cur_key;
		}

		cur_key = Check_Row(Row[1]);
		if(cur_key != key[1]){
			if (cur_key == 0x04) {
				(*timer_second)+= 10;
			} else if (cur_key == 0x02) {
				(*timer_second)-= 10;
			}
			key[1] = cur_key;
		}

		cur_key = Check_Row(Row[2]);
		if(cur_key != key[2]){
			if (cur_key == 0x04) {
				(*timer_second) += 60;
			} else if (cur_key == 0x02) {
				(*timer_second) -= 60;
			}
			else if (cur_key == 0x01) {
				(*timer_second) = 0;
			}
			key[2] = cur_key;
		}

		cur_key = Check_Row( Row[3] );
		if(cur_key != key[3]){
			if ( cur_key == 0x04) {
				(*timer_second)+= 600;
			} else if ( cur_key == 0x02) {
				(*timer_second)-= 600;
			}
			key[3] = cur_key;
		}
	}
	else if (*state == COUNTDOWN){
		if((*timer_second) == 0){
			*state = STOPPED;

		}
		else if((*func_tics) == 67){
			(*timer_second)--;
		}

		cur_key = Check_Row(Row[0]);
		if(cur_key != key[0]){
			if (cur_key == 0x01) {
				(*state) = STOPPED;
			}
			key[0] = cur_key;
		}
	}

	else if (*state == STOPPED){
		cur_key = Check_Row(Row[0]);
		if(cur_key != key[0]){
			if (cur_key == 0x01) {
				(*state) = COUNTDOWN;
			}
			key[0] = cur_key;
		}

		cur_key = Check_Row(Row[1]);
		if(cur_key != key[1]){
			if (cur_key == 0x01) {
				(*state) = SETTING;
			}
			key[1] = cur_key;
		}

		cur_key = Check_Row(Row[2]);
		if(cur_key != key[2]){
			if (cur_key == 0x01) {
				(*timer_second) = 0;
				(*state) = SETTING;
			}
			key[2] = cur_key;
		}
	}

}
//void OLED_KB( uint8_t OLED_cur_keys[12]) {
//	for (int i = 3; i >= 0; i--) {
//		oled_SetCursor(56, 5+(4-i)*10);
//		for (int j = 0; j < 3; j++) {
//			oled_WriteChar(OLED_cur_keys[j+3*i], Font_7x10, White);
//		}
//	}
//	oled_UpdateScreen();
//}
void oled_Reset( void ) {
	oled_Fill(Black);
	oled_SetCursor(0, 0);
	oled_UpdateScreen();
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
