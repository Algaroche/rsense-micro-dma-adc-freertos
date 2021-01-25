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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include <stdlib.h>
#include <audio_fft.h>
#include <math.h>
#include <task.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  SM_SENSOR_STATE_RUNNING,
  SM_SENSOR_STATE_SUSPENDING,
  SM_SENSOR_STATE_SUSPENDED,
  SM_SENSOR_STATE_INITIALIZING,
}SM_Sensor_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
#define AUDIO_IN_SAMPLING_FREQUENCY 66667
#define ADC_BUF_LEN 512
uint16_t adc_buf[ADC_BUF_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void MP23ABS1_Data_Ready(uint8_t * buf, uint16_t size);
void Cambio_Escala_FFT(float *FFT, uint16_t size, double *salida);

/* USER CODE BEGIN PFP */
AUDIO_FFT_instance_t audio_fft_M1;
float * FFT_Out;
uint8_t Mitad1 = 0;
uint8_t Mitad2 = 0;
SM_Sensor_State_t estado;

osPoolId amicPool_id;
osPoolDef(amicPool, 1, uint32_t);
static osSemaphoreId fft_data_input_sem_id;
static osSemaphoreDef(fft_data_input_sem);
double FFT_log[256];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/***************************** TASK HANDLERS *********************************/

xTaskHandle Sender_HPT_Handler;
xTaskHandle Receiver_Handler;


/***************************** QUEUE HANDLERS ********************************/

xQueueHandle SimpleQueue;

/***************************** TASK FUNCTIONS ********************************/

void Sender_HPT_Task(void *argument);
void Receiver_Task(void *argument);

uint8_t Rx_data;

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
  MX_USART2_UART_Init();
  estado = SM_SENSOR_STATE_INITIALIZING;
  /* USER CODE BEGIN 2 */
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);


    /********************************* QUEUE RELATED *************************************/
    SimpleQueue = xQueueCreate(2, sizeof(FFT_Out));
    if (SimpleQueue == 0) //Queue not created
    {
  	  char *str ="Unable to create FFT_Out Queue\n\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    }
    else
    {
  	  char *str ="FFT_Out Queue created successfully\n\n";
  	  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    }

    /******************************* SEMAPHORE RELATED ***********************************/
    /* Data read complete semaphore initialization */
      fft_data_input_sem_id = osSemaphoreCreate(osSemaphore(fft_data_input_sem), 1);
      osSemaphoreWait(fft_data_input_sem_id,osWaitForever);


    /********************************* TASK RELATED *************************************/
      xTaskCreate(Receiver_Task, "Receive", 256, NULL, 3, &Receiver_Handler);
      xTaskCreate(Sender_HPT_Task, "HPT_SEND", 256, NULL, 2, &Sender_HPT_Handler);
      //  xTaskCreate(Sender_LPT_Task, "LPT_SEND", 128, (void *)111, 2, &Sender_LPT_Handler);



    amicPool_id = osPoolCreate(osPool(amicPool));



    vTaskStartScheduler();
  /* USER CODE END 2 */


  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV25;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|DCDC_2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED2_Pin|WIFI_WAKEUP_Pin|CS_DH_Pin|EX_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CS_WIFI_Pin|C_EN_Pin|CS_ADWB_Pin|STSAFE_RESET_Pin
                          |WIFI_BOOT0_Pin|CS_DHC_Pin|SEL3_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BLE_SPI_CS_Pin|SEL1_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_p2_Pin|PB11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BOOT0_PE0_Pin BLE_TEST8_Pin */
  GPIO_InitStruct.Pin = BOOT0_PE0_Pin|BLE_TEST8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB9_Pin PB8_Pin PB14_Pin CHRGB0_Pin */
  GPIO_InitStruct.Pin = PB9_Pin|PB8_Pin|PB14_Pin|CHRGB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_PE0H3_Pin */
  GPIO_InitStruct.Pin = BOOT0_PE0H3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT0_PE0H3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_MISO_Pin SPI3_MOSI_Pin SPI3_CLK_Pin */
  GPIO_InitStruct.Pin = SPI3_MISO_Pin|SPI3_MOSI_Pin|SPI3_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_D3_Pin SDMMC_D2_Pin SDMMC_D1_Pin SDMMC_CK_Pin
                           SDMMC_D0_Pin */
  GPIO_InitStruct.Pin = SDMMC_D3_Pin|SDMMC_D2_Pin|SDMMC_D1_Pin|SDMMC_CK_Pin
                          |SDMMC_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_TEST9_Pin WIFI_DRDY_Pin INT1_DHC_Pin INT_STT_Pin
                           INT1_ADWB_Pin */
  GPIO_InitStruct.Pin = BLE_TEST9_Pin|WIFI_DRDY_Pin|INT1_DHC_Pin|INT_STT_Pin
                          |INT1_ADWB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CLK_Pin */
  GPIO_InitStruct.Pin = SPI2_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EX_PWM_Pin */
  GPIO_InitStruct.Pin = EX_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(EX_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_DP_Pin OTG_FS_DM_Pin */
  GPIO_InitStruct.Pin = OTG_FS_DP_Pin|OTG_FS_DM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI1_SCK_A_Pin SAI1_MCLK_A_Pin SAI1_FS_A_DFSDM_D3_Pin SAI1_SD_A_Pin
                           SAI1_SD_B_Pin */
  GPIO_InitStruct.Pin = SAI1_SCK_A_Pin|SAI1_MCLK_A_Pin|SAI1_FS_A_DFSDM_D3_Pin|SAI1_SD_A_Pin
                          |SAI1_SD_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin DCDC_2_EN_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|DCDC_2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin WIFI_WAKEUP_Pin CS_DH_Pin EX_RESET_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|WIFI_WAKEUP_Pin|CS_DH_Pin|EX_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10_Pin PA9_Pin PA0_Pin DAC1_OUT1_Pin
                           PA1_Pin */
  GPIO_InitStruct.Pin = PA10_Pin|PA9_Pin|PA0_Pin|DAC1_OUT1_Pin
                          |PA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN5_Pin DFSDM1_D7_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN5_Pin|DFSDM1_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG12_Pin PG10_Pin PG9_Pin */
  GPIO_InitStruct.Pin = PG12_Pin|PG10_Pin|PG9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_CMD_Pin */
  GPIO_InitStruct.Pin = SDMMC_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_RST_Pin */
  GPIO_InitStruct.Pin = BLE_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RST_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SMBA_Pin I2C2_SDA_Pin I2C2_SDAF0_Pin */
  GPIO_InitStruct.Pin = I2C2_SMBA_Pin|I2C2_SDA_Pin|I2C2_SDAF0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_WIFI_Pin C_EN_Pin CS_ADWB_Pin STSAFE_RESET_Pin
                           WIFI_BOOT0_Pin CS_DHC_Pin SEL3_4_Pin */
  GPIO_InitStruct.Pin = CS_WIFI_Pin|C_EN_Pin|CS_ADWB_Pin|STSAFE_RESET_Pin
                          |WIFI_BOOT0_Pin|CS_DHC_Pin|SEL3_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C3_SDA_Pin I2C3_SCL_Pin */
  GPIO_InitStruct.Pin = I2C3_SDA_Pin|I2C3_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_SEL_Pin */
  GPIO_InitStruct.Pin = SW_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(SW_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_DHC_Pin PGOOD_Pin INT_M_Pin */
  GPIO_InitStruct.Pin = INT2_DHC_Pin|PGOOD_Pin|INT_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_MISO_Pin SPI1_MOSI_Pin SPI1_CLK_Pin */
  GPIO_InitStruct.Pin = SPI1_MISO_Pin|SPI1_MOSI_Pin|SPI1_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_SPI_CS_Pin SEL1_2_Pin */
  GPIO_InitStruct.Pin = BLE_SPI_CS_Pin|SEL1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_HTS_Pin BLE_INT_Pin */
  GPIO_InitStruct.Pin = INT_HTS_Pin|BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C4_SCL_Pin I2C4_SDA_Pin */
  GPIO_InitStruct.Pin = I2C4_SCL_Pin|I2C4_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_MISO_Pin SPI2_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI2_MISO_Pin|SPI2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_ADWB_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = INT2_ADWB_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CHRG_Pin */
  GPIO_InitStruct.Pin = CHRG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHRG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_PWR_Pin */
  GPIO_InitStruct.Pin = BUTTON_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART3_RX_Pin USART3_TX_Pin */
  GPIO_InitStruct.Pin = USART3_RX_Pin|USART3_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USART3_RTS_Pin USART3_CTS_Pin */
  GPIO_InitStruct.Pin = USART3_RTS_Pin|USART3_CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(DFSDM1_CKOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_MOSI_p2_Pin PB11_Pin */
  GPIO_InitStruct.Pin = SPI2_MOSI_p2_Pin|PB11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT2_DH_Pin */
  GPIO_InitStruct.Pin = INT2_DH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT2_DH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12_Pin */
  GPIO_InitStruct.Pin = PE12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PE12_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	  void * data_ptr = osPoolAlloc(amicPool_id);
	  data_ptr = (void *)adc_buf;
	  osMessagePut(SimpleQueue, (uint32_t)(data_ptr), osWaitForever);

}

//Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	void * data_ptr = osPoolAlloc(amicPool_id);
	data_ptr = (void *)adc_buf;
	osMessagePut(SimpleQueue, (uint32_t)(data_ptr), osWaitForever);

}



void Sender_HPT_Task (void *argument)
{
	uint32_t TickDelay = pdMS_TO_TICKS(1);
	osEvent evt;
	while (1)
	{
		if (estado == SM_SENSOR_STATE_INITIALIZING)
		{
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
			estado = SM_SENSOR_STATE_RUNNING;
		}
		else if (estado == SM_SENSOR_STATE_RUNNING)
		{
			evt = osMessageGet(SimpleQueue, osWaitForever);

//			char *str2 = "1\n\n\n";
//			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);

			if(estado == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
			{
				void * data_ptr = evt.value.p; /* void since it is independent from data format*/

//				char *str2 = "2\n\n\n";
//				HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);

				MP23ABS1_Data_Ready((uint8_t *)data_ptr, 512);
				osPoolFree(amicPool_id, data_ptr);
			}
		}
		//vTaskDelay(TickDelay);
	}
}
//		if (Mitad1 == 1)
//		{
//			Mitad1 = 0;
//			if (xQueueSend(SimpleQueue, &adc_buf, portMAX_DELAY) == pdPASS)
//			{
//				char *str2 = " Successfully sent the Mitad1 to the queue\nLeaving SENDER_HPT Task\n\n\n";
//				HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
//			}
//		}
//		if (Mitad2 == 1)
//		{
//			//HAL_ADC_Stop_DMA(&hadc1);
//			Mitad2 = 0;
//			if (xQueueSend(SimpleQueue, &adc_buf[256], portMAX_DELAY) == pdPASS)
//			{
//				char *str2 = " Successfully sent the Mitad2 to the queue\nLeaving SENDER_HPT Task\n\n\n";
//				HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
//			}
//		}
//		vTaskDelay(TickDelay);
//	}
//}

void MP23ABS1_Data_Ready(uint8_t * buf, uint16_t size)
{
	if(AUDIO_FFT_Data_Input((int16_t *)buf, size/2, &audio_fft_M1))
	{
//		char *str2 = "3\n\n\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		osSemaphoreRelease(fft_data_input_sem_id);
		Mitad1 = 1;
	}
}

void Cambio_Escala_FFT(float *FFT, uint16_t size, double *salida)
{
	for(int i=0; i<size; i++)
	{
		salida[i] = 20 * log(FFT_Out[i]/256);
	}
}

void Receiver_Task (void *argument)
{
	audio_fft_M1.sampling_frequency = AUDIO_IN_SAMPLING_FREQUENCY;
	audio_fft_M1.FFT_len = 512;
	audio_fft_M1.overlap = 0.5f;
	audio_fft_M1.win_type = AUDIO_FTT_HANNING_WIN;
	audio_fft_M1.output = MAGNITUDE;
	AUDIO_FFT_Init(&audio_fft_M1);

	/* Allocate output buffer */
	FFT_Out = calloc(audio_fft_M1.FFT_len/2 , sizeof(float));

//	uint16_t received[ADC_BUF_LEN/2];
	uint32_t TickDelay = pdMS_TO_TICKS(1);

	while (1)
	{
		osSemaphoreWait(fft_data_input_sem_id,  osWaitForever);
//		if(Mitad1 == 1)
//		{
//			Mitad1 = 0;
//			osSemaphoreWait(fft_data_input_sem_id,  osWaitForever);
//			char *str2 = "4\n\n\n";
//			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);

			AUDIO_FFT_Process(&audio_fft_M1, FFT_Out);
			Cambio_Escala_FFT(&FFT_Out, audio_fft_M1.FFT_len/2, &FFT_log);
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
//		}
//		osSemaphoreRelease(fft_data_input_sem_id);

//		vTaskDelay(TickDelay);

//		if (xQueueReceive(SimpleQueue, &received, portMAX_DELAY) == pdTRUE)
//		{
//			HAL_ADC_Stop_DMA(&hadc1);
//			taskENTER_CRITICAL();
//			char str[100];
//			strcpy (str, "Entered RECEIVER Task about to FFT\n\n");
//			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
//
//			AUDIO_FFT_Data_Input(received, ADC_BUF_LEN/2, &audio_fft_M1);
//			AUDIO_FFT_Process(&audio_fft_M1, FFT_Out);
//
//			taskEXIT_CRITICAL();
//			strcpy (str, "Exit RECEIVER Task \n\n");
//			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
//
//			vTaskDelay(TickDelay);
//		}

	}
}

/* USER CODE END 4 */



 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
