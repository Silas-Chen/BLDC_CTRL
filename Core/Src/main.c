/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention      : CAN configuration is based on https://blog.csdn.net/prolop87/article/details/122671441
 *                   putFLOAT is based on https://blog.csdn.net/jianfeng_zhang1990/article/details/45395987
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
#include "math.h"
#include "stm32_dsp.h"
// #include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NPT 1024     // Number of FFT points
#define CHANNELNUM 4 // Number of ADC channels
#define PSC 18       // According to STM32CubeMX configuration
#define ARR 1000     // According to STM32CubeMX configuration
// #define CAN_RX_EXT_ID 0x18FF50E5
#define CAN_TX_ID 0x110
// #define PWM_MAX 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TX_HEADER;
// CAN_RxHeaderTypeDef RX_HEADER;
// uint8_t RX_DATA[8];
// uint8_t RX_FLAG;
// uint8_t RX_BUFFER[8];
uint8_t TX_FLAG;
uint8_t TX_BUFFER[CHANNELNUM] = {0};
uint32_t ADC_BUFFER[NPT];
uint32_t FFT_IN[NPT];
uint32_t FFT_OUT[NPT];
uint32_t FFT_MAG[NPT / 2];
uint16_t i = 0;
float FS = (float)180000000.0 / ((float)ARR * (float)PSC);
float FREQ = 0;
// TIM_OC_InitTypeDef sConfig[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void getFFT_MAG(void);
void getMAX_FFT_MAG_FREQ(void);
void putFLOAT(float Fdat, unsigned char *Buf, unsigned char Pos); // See in https://blog.csdn.net/jianfeng_zhang1990/article/details/45395987
void CAN_FILTER_INIT(void);
uint8_t CAN_SEND_MSG(uint8_t *msg, uint8_t len);
// uint8_t CAN_RECEIVE_MSG(uint8_t *msg, uint8_t len);
// void TIM8_PWM_INIT(void);
// void TIM8_PWM_CFG(uint16_t duty);
// void INIT_ESC(void);
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
    MX_TIM8_Init();
    /* USER CODE BEGIN 2 */
    CAN_FILTER_INIT();
    HAL_TIM_Base_Start(&htim3);
    // HAL_TIM_Base_Start_IT(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUFFER, NPT * 2);
    // TIM8_PWM_INIT();
    // INIT_ESC();
    // TIM8_PWM_CFG(550);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // TX_FLAG = CAN_SEND_MSG(TX_BUFFER, CHANNELNUM); // Send data and check if the sending is successful
        // memset(TX_BUFFER, 0, sizeof(TX_BUFFER));
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
        ADC_BUFFER[i] = (ADC_BUFFER[i] >> 16) + (ADC_BUFFER[i] << 16);
    }
    i = 0;
    memcpy(FFT_IN, ADC_BUFFER, NPT * sizeof(uint16_t));
    for (i = 0; i < NPT; i++)
    {
        // FFT_IN[i] = ADC_BUFFER[i];
        FFT_IN[i] = (FFT_IN[i] << 16);
    }
    i = 0;
    cr4_fft_1024_stm32(FFT_OUT, FFT_IN, NPT);
    getFFT_MAG();
    getMAX_FFT_MAG_FREQ();
    if(FREQ<40){
        FREQ = 0;
    }
    putFLOAT((FREQ / 6) * 60, TX_BUFFER, 0);       // Divided by 6 to get the round per second, because the motor has 12 poles(6 pairs of poles), and then multiplied by 60 to get the round per minute
    TX_FLAG = CAN_SEND_MSG(TX_BUFFER, CHANNELNUM); // Send data and check if the sending is successful
    memset(TX_BUFFER, 0, sizeof(TX_BUFFER));
    // Restart ADC-DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUFFER, NPT * 2);
}

// Take the modulus of the FFT result
void getFFT_MAG(void)
{
    signed short lX, lY;
    float X, Y, Mag;
    unsigned short i;
    for (i = 0; i < NPT / 2; i++)
    {
        lX = (FFT_OUT[i] << 16) >> 16;
        lY = (FFT_OUT[i] >> 16);

        // Dividing by 32768 and then multiplying by 65536 is done to comply with floating-point calculation rules
        X = NPT * ((float)lX) / 32768;
        Y = NPT * ((float)lY) / 32768;
        Mag = sqrt(X * X + Y * Y) / NPT;
        if (i == 0)
        {
            FFT_MAG[i] = (unsigned long)(Mag * 32768);
        }
        else
        {
            FFT_MAG[i] = (unsigned long)(Mag * 65536);
        }
    }
    i = 0;
}

// Find the main frequency
void getMAX_FFT_MAG_FREQ(void)
{
    uint32_t maxMAG = 0;
    uint16_t maxMAG_INDEX = 0;
    for (i = 2; i < NPT / 2; i++) //  Where the number 'i' starts from may be about the wave form, like sin is from 1 and triangle is from 2.(?)
    {
        if (FFT_MAG[i] > maxMAG)
        {
            maxMAG = FFT_MAG[i];
            maxMAG_INDEX = i;
        }
    }
    FREQ = FS * (((float)maxMAG_INDEX) / ((float)NPT * 4)) + 26; // Add 2 and 4 is for correction
    i = 0;
}

void putFLOAT(float Fdat, unsigned char *Buf, unsigned char Pos)
{
    unsigned char *p;

    p = (unsigned char *)&Fdat;
    Buf[Pos] = *p;
    Buf[Pos + 1] = *(p + 1);
    Buf[Pos + 2] = *(p + 2);
    Buf[Pos + 3] = *(p + 3);
}

// CAN filter initialization
void CAN_FILTER_INIT(void)
{
    // CAN_FilterTypeDef sFilterConfig;

    // sFilterConfig.FilterBank = 0;
    // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Set the filter mode to mask mode
    // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // sFilterConfig.FilterIdHigh = (((uint32_t)CAN_RX_EXT_ID << 3) & 0xFFFF0000) >> 16;                  // Set the filter ID high 16 bits
    // sFilterConfig.FilterIdLow = (((uint32_t)CAN_RX_EXT_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; // Set the filter ID low 16 bits, the way to set the filter ID is to shift the ID 3 bits to the left and then set the ID to the extended frame and data frame
    // sFilterConfig.FilterMaskIdHigh = 0xFFFF;                                                           // Set the filter mask high 16 bits
    // sFilterConfig.FilterMaskIdLow = 0xFFFF;                                                            // Set the filter mask low 16 bits
    // sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                                 // Set the filter FIFO assignment
    // sFilterConfig.FilterActivation = ENABLE;
    // sFilterConfig.SlaveStartFilterBank = 14;

    // if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    // {
    //     /* Filter configuration Error */
    //     Error_Handler();
    // }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    // Config TX_HEADER with normal frame
    // TX_HEADER.ExtId = ; // Set the extended identifier (29-bit)
    // TX_HEADER.IDE = CAN_ID_EXT;      // Use extended frame
    // TX_HEADER.RTR = CAN_RTR_DATA;    // Set as data frame
    // TX_HEADER.DLC = 8;               // Set the data length to 8
    TX_HEADER.TransmitGlobalTime = DISABLE; // Disable the transmission time stamp
}

// CAN send message, takes an array pointer and data length as input parameters
// Returns 0 if data transmission is successful, returns 1 if there is a transmission error
uint8_t CAN_SEND_MSG(uint8_t *msg, uint8_t len)
{
    uint8_t i = 0;
    uint32_t TxMailbox;
    uint8_t message[8];

    TX_HEADER.StdId = CAN_TX_ID;     // Set the standard identifier (11-bit)
    TX_HEADER.IDE = CAN_ID_STD;      // Use standard frame
    TX_HEADER.RTR = CAN_RTR_DATA;    // Set as data frame
    TX_HEADER.DLC = len;             // Set the data length to 8

    for (i = 0; i < len; i++)
    {
        message[i] = msg[i];
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &TX_HEADER, message, &TxMailbox) != HAL_OK) // Add a message to the mailbox
    {
        return 1;
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
    {
    }
    return 0;
}

// CAN receive message
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanNum)
// {
//     uint32_t i;

//     RX_FLAG = 1; // Set the receive flag to 1
//     HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RX_HEADER, RX_DATA);
//     for (i = 0; i < RX_HEADER.DLC; i++)
//     {

//         RX_BUFFER[i] = RX_DATA[i]; // Copy the received data to the receive buffer
//     }
// }

// Initialize TIM8 PWM
// void TIM8_PWM_INIT(void)
// {
//     // htim4.Instance = TIM8;
//     // htim4.Init.Prescaler = PWM_PRESCALER - 1; // 72MHz
//     // htim4.Init.Period = PWM_PERIOD - 1;       //->500Hz(144000)              7
//     HAL_TIM_Base_Init(&htim8);
//     uint8_t i = 0;
//     for (i = 0; i < 4; i += 1)
//     {
//         sConfig[i].OCMode = TIM_OCMODE_PWM1;
//         sConfig[i].OCPolarity = TIM_OCPOLARITY_HIGH;
//         sConfig[i].OCFastMode = TIM_OCFAST_DISABLE;
//     }
//     // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//     // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//     // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
// }

// void TIM8_PWM_CFG(uint16_t duty)
// {
//     for (uint8_t i = 0; i < 4; i++)
//     {
//         if (duty <= PWM_MAX)
//         {
//             sConfig[i].Pulse = duty;
//         }
//         else
//         {
//             sConfig[i].Pulse = PWM_MAX;
//         }
//     }
//     HAL_TIM_PWM_ConfigChannel(&htim8, &sConfig[2], TIM_CHANNEL_3);
//     // HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig[1], TIM_CHANNEL_2);
//     // HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig[2], TIM_CHANNEL_3);
//     // HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig[3], TIM_CHANNEL_4);
// }

// void INIT_ESC(void)
// {
//     // GPIO_InitTypeDef GPIO_PB9;
//     // GPIO_PB9.Pin = GPIO_PIN_9;
//     // GPIO_PB9.Mode = GPIO_MODE_OUTPUT_PP;
//     // GPIO_PB9.Pull = GPIO_PULLUP;
//     // GPIO_PB9.Speed = GPIO_SPEED_FREQ_LOW;
//     // HAL_GPIO_Init(GPIOB, &GPIO_PB9);

//     // Init ESC.
//     TIM8_PWM_CFG(1000);
//     // DELAY_MS(20);
//     HAL_Delay(20);
//     TIM8_PWM_CFG(0);
//     // DELAY_MS(20);
//     HAL_Delay(20);
//     // TIM8_PWM_CFG(1000);
//     // DELAY_MS(20);
//     // HAL_Delay(20);

//     // Normal starting ESC.
//     TIM8_PWM_CFG(0);
//     // DELAY_MS(20);
//     HAL_Delay(20);
//     TIM8_PWM_CFG(1000);
//     // DELAY_MS(20);
//     HAL_Delay(20);
//     TIM8_PWM_CFG(550);
//     // DELAY_MS(5000);
//     HAL_Delay(5000);
//     TIM8_PWM_CFG(250);
//     // DELAY_MS(20);
//     HAL_Delay(20);
// }
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
