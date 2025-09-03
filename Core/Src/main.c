/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "myfreertos.h"
#include "queue.h"

I2S_HandleTypeDef hi2s2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

DMA_HandleTypeDef hdma_spi2_rx;

/* 队列句柄 */
QueueHandle_t xAudioQueue;
/* 临时接收缓冲区 */
uint16_t i2s_dma_buf[AUDIO_DMA_BUF_SIZE] = {0x99, 0x99, 0x99, 0x99};

extern void AudioTask(void *params);


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
void Audio_Init(void);
void MX_DMA_Init(void);
void StartDefaultTask(void const * argument);
void print_array(uint16_t *arr, size_t len);
void uart2_print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
void uart2_print_num(uint32_t num)
{
    char buf[16];  // 足够容纳 32位整数最大值 "4294967295\0"
    int len = snprintf(buf, sizeof(buf), "%lu", (unsigned long)num);
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
}

void uart2_print_uint16_array(const uint16_t *arr, size_t len)
{
    char buffer[32];  // 临时字符串缓存（根据需要调整大小）
    for (size_t i = 0; i < len; i++)
    {
        snprintf(buffer, sizeof(buffer), "%u ", arr[i]); // 转成字符串
        uart2_print(buffer);
    }
    uart2_print("\r\n"); // 换行
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_DMA_Init();
  Audio_Init();

	
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  uart2_print("All Init Successfull!");
  osKernelStart();

  while (1)
  {
  }
}


void Audio_Init(void)
{
    /* 创建队列 */
    xAudioQueue = xQueueCreate(AUDIO_DMA_BUF_SIZE, sizeof(AudioFrame_t));
    if(xAudioQueue == NULL) Error_Handler();

    /* 创建音频处理任务 */
    xTaskCreate(AudioTask, "AudioTask", 512, NULL, 5, NULL);

    /* 启动 I2S DMA 循环接收 */
    if (HAL_I2S_Receive_DMA(&hi2s2, i2s_dma_buf, AUDIO_DMA_BUF_SIZE/2) != HAL_OK)
    {
        Error_Handler();
    }
}


void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    AudioFrame_t frame;

    for(int i = 0; i < AUDIO_I2S_FRAME_SAMPLES * AUDIO_CHANNELS; i++)
    {
        uint32_t raw = i2s_dma_buf[i];
        int32_t sample = (raw >> 8) & 0xFFFFFF;       // 左对齐24bit
        if(sample & 0x800000) sample |= 0xFF000000;   // 符号扩展
        frame.data[i] = sample;
    }
    xQueueSendFromISR(xAudioQueue, &frame, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    AudioFrame_t frame;

    for(int i=0; i<AUDIO_I2S_FRAME_SAMPLES*AUDIO_CHANNELS; i++)
    {
        uint32_t raw = i2s_dma_buf[i + AUDIO_DMA_BUF_SIZE/2];
        int32_t sample = (raw >> 8) & 0xFFFFFF;
        if(sample & 0x800000) sample |= 0xFF000000;
        frame.data[i] = sample;
    }
    xQueueSendFromISR(xAudioQueue, &frame, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}





/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi2_rx);
}


/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_DMA_Init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

    hdma_spi2_rx.Instance = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16bit
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 16bit
    hdma_spi2_rx.Init.Mode = DMA_CIRCULAR; // 循环模式
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
        Error_Handler();
    }

    /* 将 DMA 与 I2S2 绑定 */
    __HAL_LINKDMA(&hi2s2, hdmarx, hdma_spi2_rx);

    /* 配置 DMA 中断优先级并使能 */
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_5;        // PA0
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // 输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;      // 不上拉不下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  MX_USB_DEVICE_Init();
  for(;;)
  {
    osDelay(1);
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
}

#endif /* USE_FULL_ASSERT */
