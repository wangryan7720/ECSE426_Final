/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BUFFER_SIZE 24000
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern UART_HandleTypeDef huart5;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;
extern int txFlag;
int timFlag = 0;
int sec_counter = 0;
int isButtonPressed = 0;
int isFinished = 0;
int recordingData = 0;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//uint16_t buffer[70000];
uint8_t buffer1[BUFFER_SIZE];
/* USER CODE END 0 */

int i = 0;
int j = 0;
int bufferReady = 0;
uint8_t adcVal = 0;
uint8_t high_byte;
uint8_t low_byte;
int adcDataReady = 0;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_UART5_Init();
	HAL_UART_MspInit(&huart5);
	MX_TIM2_Init();
  MX_ADC2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_MspInit(&htim2);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_MspInit(&hadc2);

  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */
		if (isButtonPressed == 1 && isFinished == 0) {
			recordingData = 1;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}
		
		
		if (timFlag == 1) {
			adcVal = HAL_ADC_GetValue(&hadc2);
			//printf("%d\n", adcVal);
			timFlag = 0;
			adcDataReady = 1;
		}
		
		if (recordingData == 1 && adcDataReady == 1) {
			adcDataReady = 0;
			//adcVal = HAL_ADC_GetValue(&hadc2);
			//printf("%d\n", adcVal);
			timFlag = 0;
			//high_byte = adcVal >> 8;
			//low_byte = adcVal & 0x00FF;
			//timFlag = 0;		
				if (i < BUFFER_SIZE) {
					buffer1[i] = adcVal;
					//printf("buffer[%d] = %d\n", i, buffer1[i]);
					i++;
					//buffer1[i] = low_byte;
					//printf("buffer[%d] = %d\n", i, buffer1[i]);
					//i++;
					//timFlag = 0;
				} else {
					printf("Record finished");
					recordingData = 0;
					bufferReady = 1;
					isFinished = 1;
					isButtonPressed = 0;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				}			
			}
		
		if(isButtonPressed == 0) {
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {
				if(isButtonPressed == 0) {
					isButtonPressed = 1;
					sec_counter = 0;
				}
			}
		}
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				//MX_GPIO_Init();
				 // MX_ADC2_Init();
				//	HAL_ADC_Start(&hadc2);
	//HAL_ADC_MspInit(&hadc2);
				//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
			
		
		
		/*
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {
			printf("%d times\n", counter);
			counter++;
		}
			*/
		
		//if (counter > 2000) {
			//printf("Transmit or record\n");
			
			/*
			for (int i = 0; i < 80000; i++) {
				buffer[i] = HAL_ADC_GetValue(&hadc2);
				printf("%d\n", buffer[i]);
			}
			*/
		//}

		//counter = 0;
		if (txFlag == 0 && bufferReady == 1 && isFinished == 1) {
			txFlag = 1;
			//printf("ADC %d\n", adcVal);
			//printf("High %d\n", high_byte);
			//printf("Low %d\n", low_byte);
			int b = HAL_UART_Transmit_IT(&huart5, &buffer1[j], 1);
			for (int k = 0; k < 10000; k++){}
			j++;
			//printf("Sending %d\n", j);
			if (j >= BUFFER_SIZE) {
				isFinished = 0;
				printf("Transmit finished\n");
			}
		}

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
