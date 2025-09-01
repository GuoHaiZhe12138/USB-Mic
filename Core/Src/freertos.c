/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "myfreertos.h"
#include "queue.h"
#include "usbd_def.h"
#include "usbd_audio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size);
extern USBD_HandleTypeDef hUsbDeviceFS;

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void AudioTask(void *params)
{
    AudioFrame_t frame;

    while(1)
    {
        /* 从队列获取一帧 I2S 数据，等待 100ms */
        if(xQueueReceive(xAudioQueue, &frame, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            /* TODO: 这里做软件滤波、EQ、音量处理 */
			taskENTER_CRITICAL();
            /* USB 发送 */
            USBD_LL_Transmit(&hUsbDeviceFS, AUDIO_IN_EP, (uint8_t*)frame.data, AUDIO_FRAME_BYTES);
			taskEXIT_CRITICAL();
        }
    }
}
