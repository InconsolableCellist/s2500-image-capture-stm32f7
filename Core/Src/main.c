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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <usbd_cdc_if.h>
#include "adc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUF_SIZE 128        // number of elements, not bytes
#define STATUS_BUF_SIZE 6   // number of elements, not bytes

// Flow control
uint8_t mode_switch_requested = 0;
uint8_t xy_pulse_fired = 0;
uint8_t y_pulse_fired = 0;
uint8_t usb_transfer_complete = 1;
uint8_t current_adc_mode = ADC_CUSTOM_SPEED_THREEQUARTERS;
volatile uint8_t xy_rising_or_falling = 0;
volatile uint8_t y_rising_or_falling = 0;
uint8_t status_buffer_ready = 1;
uint16_t items_remaining = 0;

// Data buffers
uint16_t tim7_overflow = 0;
uint16_t* buffer0;
uint16_t* buffer1;
volatile uint16_t* buf_to_fill = NULL;
volatile uint8_t buf_ready = 0;
uint16_t* status_code_buffer;

volatile uint8_t* usb_received_data_buf;
volatile uint8_t usb_received_data_buf_ready = 0;
volatile uint8_t usb_received_data_len = 64;

/**
 * Possible values sent over USB from this device:
 *
 * status_code_buffer
 *  0xFEFA - start of X pulse data
 *  0xXXXX - pulse tim7 overflows (16/sec)
 *  0xXXXX - pulse microseconds (1e6/sec)
 *  0xXXXX - scan mode (ADC_CUSTOM_SPEED_*)
 *  0xXXXX - row (image data) tim7 oveflows (16/sec)
 *  0xXXXX - row (image data) microseconds (16/sec)
 *
 * status_code_buffer
 *  0xFEFB - start of Y pulse data
 *  0xXXXX - pulse tim7 overflows (16/sec)
 *  0xXXXX - pulse microseconds (1e6/sec)
 *  0xXXXX - scan mode (ADC_CUSTOM_SPEED_*)
 *  0xXXXX - don't care
 *  0xXXXX - don't care
 *
 * status_code_buffer
 *  0xFEFC - responding to heartbeat request command
 *  0xXXXX - don't care
 *  0xXXXX - don't care
 *  0xXXXX - don't care
 *  0xXXXX - don't care
 *  0xXXXX - don't care
 *
 * Everything else sent over USB are ADC sampled values
 */

/**
 * Possible values sent over USB to this device:
 *
 * 0xA0 - Reconfigure ADC with last selected (or default) mode and restart it on the rising edge of the next pulse.
 *        Note that this could be mid-frame (and very likely will be)
 * 0xA1 - Same as above but with ADC_CUSTOM_SPEED_RAPID
 * 0xA2 - Same as above but with ADC_CUSTOM_SPEED_HALF
 * 0xA3 - Same as above but with ADC_CUSTOM_SPEED_HALF_SLOWER
 * 0xA4 - Same as above but with ADC_CUSTOM_SPEED_THREEQUARTERS
 * 0xA5 - Same as above but with ADC_CUSTOM_SPEED_THREEQUARTERS_SLOWER
 * 0xA6 - Same as above but with ADC_CUSTOM_SPEED_PHOTO
 * 0xA7 - Respond ASAP with heartbeat data (see status_code_buffer 0xFEFC). Will always be sent before the next
 *        real status_code_buffer packet.
 *
 * Everything else is received but ignored
 */
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

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */
/*
 *  htim6 = XY_Pulse bounce timer
 *  htim7 = pulse/row timer
 *  htim13 = Y_pulse bounce timer
 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HandleIncomingCommand();

void SendBuffer(uint8_t status, uint16_t len);

void SendStatusBuffer();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t status;
    buffer0 = (uint16_t *)malloc(sizeof(uint16_t) * BUF_SIZE);
    buffer1 = (uint16_t *)malloc(sizeof(uint16_t) * BUF_SIZE);
    buf_to_fill = buffer0;

    status_code_buffer = (uint16_t *)malloc(sizeof(uint16_t) * STATUS_BUF_SIZE);
    usb_received_data_buf = (uint8_t *)malloc(64);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    memset(buffer0, 0x55, BUF_SIZE * sizeof(uint16_t));
    memset(buffer1, 0x54, BUF_SIZE * sizeof(uint16_t));
    memset(status_code_buffer, 0x00, STATUS_BUF_SIZE * sizeof(uint16_t));
    status_code_buffer[0] = 0xFEFA;
    memset(usb_received_data_buf, 0x52, 64);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Stop_DMA(&hadc1);
  ADC_SwitchSamplingMode(&hadc1, current_adc_mode);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // X or Y pulse, rising or falling
      // Measures the duration of X and/or Y pulses and starts/stops data collection
      // The xy_pulse line is supposed to contain both X and Y pulses, but in 3/4 scan speed the pulse time is the same
      // We'll use the dedicated Y pulse line to detect Y pulses for sure
      if (xy_pulse_fired) {
          xy_pulse_fired = 0;
          if (HAL_GPIO_ReadPin(XY_PULSE_GPIO_Port, XY_PULSE_Pin) == GPIO_PIN_SET) {
              DEBUG_HIGH

              HAL_TIM_Base_Stop_IT(&htim7);
              status_code_buffer[1] = tim7_overflow;
              status_code_buffer[2] = ((uint16_t)__HAL_TIM_GET_COUNTER(&htim7));

              // Start timing the row
              __HAL_TIM_SET_COUNTER(&htim7, 0);
              tim7_overflow = 0;
              HAL_TIM_Base_Start_IT(&htim7);
              HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf_to_fill, BUF_SIZE);
          } else {
              DEBUG_LOW
              HAL_ADC_Stop_DMA(&hadc1);
              items_remaining = (uint16_t)hadc1.DMA_Handle->Instance->NDTR;
              HAL_TIM_Base_Stop_IT(&htim7);

              // how to calculate duration in seconds:
              // pulse_time = ((double)tim7_overflow / 16) + ((double)__HAL_TIM_GET_COUNTER(&htim7) / 1000000);
              // row_time   = ((double)tim7_overflow / 16) + ((double)__HAL_TIM_GET_COUNTER(&htim7) / 1000000);
              if (y_pulse_fired) {
                  y_pulse_fired = 0;
                  // this is a Y pulse
                  status_code_buffer[0] = 0xFEFB;
              } else {
                  // just an X pulse
                  status_code_buffer[0] = 0xFEFA;
              }
              // [1] and [2] set above (XY pulse time)
              status_code_buffer[3] = (uint16_t)current_adc_mode;
              status_code_buffer[4] = tim7_overflow; // row time
              status_code_buffer[5] = ((uint16_t) __HAL_TIM_GET_COUNTER(&htim7));

              status_buffer_ready = 1; // sent below
              if (buf_to_fill == buffer0) {
                  ADC_STATUS_HIGH // signal buffer swap
                  buf_to_fill = buffer1;
              } else {
                  ADC_STATUS_LOW
                  buf_to_fill = buffer0;
              }
              buf_ready = 1;

              __HAL_TIM_SET_COUNTER(&htim7, 0);
              tim7_overflow = 0;
              HAL_TIM_Base_Start_IT(&htim7);
          }
      }

      uint16_t len = ( ((uint16_t)BUF_SIZE) - items_remaining)*2;

      while (buf_ready) {
          SendBuffer(status, len);
      }

      while (status_buffer_ready) {
          SendStatusBuffer();
      }

      if (usb_received_data_buf_ready) {
          HandleIncomingCommand();
          usb_received_data_buf_ready = 0;
      }

    if (mode_switch_requested) {
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop_IT(&htim7);
        ADC_SwitchSamplingMode(&hadc1, current_adc_mode);
        mode_switch_requested = 0;
        // ADC restarted during XY pulse
    }

  }
  /* USER CODE END 3 */
}

void SendStatusBuffer() {
    USB_STATUS_HIGH // set low by transfer complete callback
    if (CDC_Transmit_HS((uint8_t*)status_code_buffer, STATUS_BUF_SIZE*2) != USBD_OK) {
        DEBUG_HIGH DEBUG_LOW DEBUG_HIGH DEBUG_LOW DEBUG_HIGH DEBUG_LOW
    } else {
        status_buffer_ready = 0;
    }
}

void SendBuffer(uint8_t status, uint16_t len) {// The buffer just swapped and the old one is ready to send
    usb_transfer_complete = 0;

    USB_STATUS_LOW
    USB_STATUS_HIGH // set low by transfer complete callback
    // Transfer up to BUF_SIZE samples, less the number, in bytes (2 bytes per sample)
    if (buf_to_fill == buffer0) {
        status = CDC_Transmit_HS((uint8_t*)buffer1, len);
    } else {
        status = CDC_Transmit_HS((uint8_t*)buffer0, len);
    }
    if (status != USBD_OK) {
        // signal we lost a buffer
        DEBUG_HIGH DEBUG_LOW DEBUG_HIGH DEBUG_LOW
    } else {
        usb_transfer_complete = 1;
        buf_ready = 0;
        items_remaining = 0;
    }
}

void HandleIncomingCommand() {
    for (uint8_t i=0; i < usb_received_data_len; ++i) {
        switch (usb_received_data_buf[i]) {
            case 0xA0: // restart with current ADC mode
                mode_switch_requested = 1;
                break;
            case 0xA1:
                current_adc_mode = ADC_CUSTOM_SPEED_RAPID;
                mode_switch_requested = 1;
                break;
            case 0xA2:
                current_adc_mode = ADC_CUSTOM_SPEED_HALF;
                mode_switch_requested = 1;
                break;
            case 0xA3:
                current_adc_mode = ADC_CUSTOM_SPEED_HALF_SLOWER;
                mode_switch_requested = 1;
                break;
            case 0xA4:
                current_adc_mode = ADC_CUSTOM_SPEED_THREEQUARTERS;
                mode_switch_requested = 1;
                break;
            case 0xA5:
                current_adc_mode = ADC_CUSTOM_SPEED_THREEQUARTERS_SLOWER;
                mode_switch_requested = 1;
                break;
            case 0xA6:
                current_adc_mode = ADC_CUSTOM_SPEED_PHOTO;
                mode_switch_requested = 1;
                break;
            case 0xA7: // heartbeat
            case 0x68: // 'h'
                status_code_buffer[0] = 0xFEFC;
                status_buffer_ready = 1;
                break;
            case 0xA8:
                current_adc_mode = ADC_CUSTOM_SPEED_PHOTO_SLOWEST;
                mode_switch_requested = 1;
                break;
            default:
               break;
        }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 95;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 89;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 62499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 95;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_STATUS_PIN_GPIO_Port, ADC_STATUS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_STATUS_PIN_GPIO_Port, USB_STATUS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Y_PULSE_Pin */
  GPIO_InitStruct.Pin = Y_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Y_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_PIN_Pin */
  GPIO_InitStruct.Pin = DEBUG_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_STATUS_PIN_Pin */
  GPIO_InitStruct.Pin = ADC_STATUS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_STATUS_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_SWITCH_Pin */
  GPIO_InitStruct.Pin = MODE_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_RST_Pin */
  GPIO_InitStruct.Pin = EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XY_PULSE_Pin */
  GPIO_InitStruct.Pin = XY_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(XY_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_STATUS_PIN_Pin */
  GPIO_InitStruct.Pin = USB_STATUS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_STATUS_PIN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Stop_DMA(&hadc1);
    if (!usb_transfer_complete || buf_ready) {
        // buf still being ready means it wasn't transmitted
        ADC_STATUS_HIGH // signal failure to transfer
        ADC_STATUS_LOW
        ADC_STATUS_HIGH
        ADC_STATUS_LOW
        ADC_STATUS_HIGH
        ADC_STATUS_LOW
    }
    if (buf_to_fill == buffer0) {
        ADC_STATUS_HIGH // signal buffer swap
        buf_to_fill = buffer1;
    } else {
        ADC_STATUS_LOW
        buf_to_fill = buffer0;
    }
    buf_ready = 1;
//    DMA2_Stream0->NDTR

//    if (HAL_GPIO_ReadPin(XY_PULSE_GPIO_Port, XY_PULSE_Pin) == GPIO_PIN_SET) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf_to_fill, BUF_SIZE);
//    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    // possible overrun
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
    ADC_STATUS_HIGH ADC_STATUS_LOW
//    HAL_ADC_Stop_DMA(hadc);
//    HAL_ADC_Start_DMA(hadc, (uint32_t*)buf_to_fill, BUF_SIZE);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
