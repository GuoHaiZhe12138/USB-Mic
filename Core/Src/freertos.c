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
#include "math.h"

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
    int16_t usb_buf[48];   // 48 samples per frame for 48kHz, 1ms frame
    uint32_t sample_index = 0;
    float freq = 440.0f;   // 正弦波频率 440Hz
    float amp = 30000.0f;  // 16-bit PCM 幅值

    (void)params;

    while (1)
    {
//        // 生成1帧正弦波
//        for (int i = 0; i < 48; i++)
//        {
//            usb_buf[i] = (int16_t)(amp * sinf(2.0f * 3.1415926f * freq * sample_index / 48000.0f));
//            sample_index++;
//            if (sample_index >= 48000) sample_index = 0;
//        }

//        // 发送到 USB IN 端点
//        while (USBD_LL_Transmit(&hUsbDeviceFS, AUDIO_IN_EP, (uint8_t*)usb_buf, 96) != USBD_OK)
//        {
//            vTaskDelay(pdMS_TO_TICKS(1)); // 等待上一次传输完成
//        }
//		uart2_print("FIFO:");
//		extern USBD_HandleTypeDef hUsbDeviceFS;
//		uart2_print_num(hUsbDeviceFS.ep_in[AUDIO_IN_EP & 0xFU].is_used);
//		uart2_print("\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1ms 一帧
    }
}

//void AudioTask(void *params)
//{
//    AudioFrame_t frame;
//    uint8_t usb_buf[AUDIO_FRAME_BYTES];  // 每帧 6 字节（2声道 * 3字节）
//    
//    while(1)
//    {
//        /* 从队列获取一帧 I2S 数据，等待 5ms */
//        if(xQueueReceive(xAudioQueue, &frame, pdMS_TO_TICKS(1)) == pdTRUE)
//        {
//            /* 打包 24-bit 数据到连续字节 */
//            for(int i = 0; i < AUDIO_I2S_FRAME_SAMPLES * AUDIO_CHANNELS; i++)
//            {
//                uint32_t sample = frame.data[i] & 0xFFFFFF;  // 保留低24位
//                usb_buf[i*3 + 0] = (sample >> 16) & 0xFF;   // 高字节
//                usb_buf[i*3 + 1] = (sample >> 8) & 0xFF;    // 中字节
//                usb_buf[i*3 + 2] = sample & 0xFF;           // 低字节
//            }

//            /* USB 发送 */
//            taskENTER_CRITICAL();
//            USBD_LL_Transmit(&hUsbDeviceFS, AUDIO_IN_EP, usb_buf, AUDIO_FRAME_BYTES);
//            taskEXIT_CRITICAL();
//        }
//    }
//}

