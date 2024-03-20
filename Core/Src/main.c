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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NPT 1024 // Number of FFT points
// #define CHANNELNUM 4
// #define CHANNELNUM__2 (CHANNELNUM / 2)
#define PSC 18   // According to STM32CubeMX configuration
#define ARR 1000 // According to STM32CubeMX configuration
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ADC_BUFFER[NPT];
float32_t FFT_IN[NPT*2];
// float32_t FFT_OUT[NPT];
float32_t FFT_MAG[NPT];
// uint32_t ADC_BUFFER[NPT][CHANNELNUM__2];
// uint32_t FFT_IN[CHANNELNUM][NPT];
// uint32_t FFT_OUT[CHANNELNUM][NPT];
// uint32_t FFT_MAG[CHANNELNUM][NPT / 2];
uint16_t i = 0;
// uint8_t j = 0;
// uint8_t m = 0;
// uint8_t n = 0;
float FS = 180000000.0 / ((float)ARR * (float)PSC);
float FREQ = {0};
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim3);
    // HAL_TIM_Base_Start_IT(&htim3);
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUFFER, NPT * CHANNELNUM); // One uint32 for two channels, there are 4 channels, so we need NPT*CHANNELNUM
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUFFER, NPT * 2);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE BEGIN 4 */
// ADC-DMA interrupt callback function
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Stop ADC-DMA
    HAL_ADC_Stop_DMA(&hadc1);
    for (i = 0; i < NPT; i++)
    {
        ADC_BUFFER[i] = (ADC_BUFFER[i]>>16)+(ADC_BUFFER[i]<<16);
        FFT_IN[i*2] = (float32_t)ADC_BUFFER[i];
    }
    // i = 0;
    // memcpy(FFT_IN,ADC_BUFFER,NPT*sizeof(uint16_t));
    // FFT calculate
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S, NPT, 0, 1);
    arm_cfft_radix4_f32(&S, FFT_IN);
    arm_cmplx_mag_f32(FFT_IN, FFT_MAG, NPT / 2);
    // i = 0;
    // for (i = 0; i < NPT; i++)
    // {
    //     // for (m = 0; m < CHANNELNUM; m++)
    //     // {
    //     //         // if ((m % 2) == 0)
    //     //         // {
    //     //         //     FFT_IN[m][i] = (ADC_BUFFER[i][0] << 16);
    //     //         //     // FFT_IN[m][i] = (FFT_IN[m][i] << 16); // FFT_IN is 32-bit, first 16-bit should be real number, and the second 16-bit should be imaginary number
    //     //         // }
    //     //         // else
    //     //         // {
    //     //         //     FFT_IN[m][i] = (ADC_BUFFER[i][1] >> 16);
    //     //         //     FFT_IN[m][i] = (FFT_IN[m][i] << 16); // FFT_IN is 32-bit, first 16-bit should be real number, and the second 16-bit should be imaginary number
    //     //         // }
    //     //         // switch (m)
    //     //         // {
    //     //         // case 0:
    //     // FFT_IN[0][i] = (ADC_BUFFER[i][0] << 16);
    //     //             // FFT_IN[0][i] = (FFT_IN[0][i]<<16); // FFT_IN is 32-bit, first 16-bit should be real number, and the second 16-bit should be imaginary number
    //     //         //     break;
    //     //         // case 1:
    //     //             FFT_IN[1][i] = (ADC_BUFFER[i][0] >> 16);
    //     //             FFT_IN[1][i] = (FFT_IN[1][i] << 16); // FFT_IN is 32-bit, first 16-bit should be real number, and the second 16-bit should be imaginary number
    //     //         //     break;
    //     //         // case 2:
    //     //             FFT_IN[2][i] = (ADC_BUFFER[i][1] << 16);
    //     //             // FFT_IN[2][i] = (FFT_IN[2][i]<<16); // FFT_IN is 32-bit, first 16-bit should be real number, and the second 16-bit should be imaginary number
    //     //         //     break;
    //     //         // case 3:
    //     //             FFT_IN[3][i] = (ADC_BUFFER[i][1] >> 16);
    //     //             FFT_IN[3][i] = (FFT_IN[3][i] << 16); // FFT_IN is 32-bit, first 16-bit should be real number, and the second 16-bit should be imaginary number
    //     //             // break;
    //     //         // default:
    //     //         //     // Error
    //     //         //     break;
    //     // }
    //     // }
    // }
    // Reset i, j, m, n
    // m = 0;
    // n = 0;
    // j = 0;
    i = 0;
    // getFFT_MAG();
    // getMAX_FFT_MAG_FREQ();
    // Restart ADC-DMA
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUFFER, NPT * CHANNELNUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUFFER, NPT * 2);
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
