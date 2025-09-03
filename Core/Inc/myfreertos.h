#ifndef MYFREERTOS_H
#define MYFREERTOS_H

#include "FreeRTOS.h"
#include "queue.h"

#define AUDIO_I2S_FRAME_SAMPLES 1        // 每声道每帧采样数
#define AUDIO_CHANNELS 2                 // 双声道
#define AUDIO_BYTES_PER_SAMPLE 3         // 24bit = 3字节
#define AUDIO_DMA_BUF_SIZE 128  		 // DMA 双缓冲大小（总样本数，不是字节数）

#define AUDIO_DMA_BUF_SIZE 128  // 双缓冲，前半+后半各64样本

// 每帧总字节数
#define AUDIO_FRAME_BYTES  (AUDIO_I2S_FRAME_SAMPLES * AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)  

typedef struct {
    int32_t data[AUDIO_I2S_FRAME_SAMPLES * AUDIO_CHANNELS];  // 每帧包含左右声道样本
} AudioFrame_t;


// FreeRTOS 队列句柄
// 用于在 I2S 中断和 Audio 任务之间传递音频帧
extern QueueHandle_t xAudioQueue;
extern void uart2_print(const char *str);
extern void uart2_print_num(uint32_t num);

#endif
