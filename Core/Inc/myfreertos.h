#ifndef MYFREERTOS_H
#define MYFREERTOS_H

#include "queue.h"
// 每个声道每帧采样数（I2S 中断触发一次接收的采样数）
// 这里设置为 256，即每次中断接收 256 个采样
#define AUDIO_I2S_FRAME_SAMPLES  256

// 声道数，双声道立体声
#define AUDIO_CHANNELS           2

// 每个采样的字节数，这里是 3 字节，表示 24bit 音频
#define AUDIO_BYTES_PER_SAMPLE   3  

// 每帧的总字节数 = 每声道采样数 × 声道数 × 每采样字节数
// 256 * 2 * 3 = 1536 字节
#define AUDIO_FRAME_BYTES  (AUDIO_I2S_FRAME_SAMPLES * AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)  

// FreeRTOS 队列长度，用于存放音频帧
// 队列最多存放 8 帧音频数据
#define AUDIO_QUEUE_LENGTH 8

// 音频帧的数据结构
// 每帧包含 AUDIO_I2S_FRAME_BYTES/2 个 16bit 单元（因为 HAL_I2S_Receive_IT 使用 uint16_t*）
// 注意：24bit 样本会占用 2 个 16bit 单元，因此这里数组大小为 AUDIO_I2S_FRAME_BYTES / 2
typedef struct {
    uint16_t data[AUDIO_FRAME_BYTES / 2];  // 用于存放一帧 I2S 接收的数据
} AudioFrame_t;

// FreeRTOS 队列句柄
// 用于在 I2S 中断和 Audio 任务之间传递音频帧
extern QueueHandle_t xAudioQueue;


#endif
