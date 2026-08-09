#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "Board/car_config.h"

typedef enum {
    ENCODER_L = 0,      /* 左轮霍尔编码器 */
    ENCODER_R,          /* 右轮霍尔编码器 */
    ENCODER_COUNT
} EncoderIndex_t;

void Encoder_Init(void);
void Encoder_UpdateSpeed(uint32_t ms);
float Encoder_GetRPM(EncoderIndex_t id);
int16_t Encoder_GetPulse(EncoderIndex_t id);

#endif
