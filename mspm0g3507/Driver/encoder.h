/* 车轮编码器接口。 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "Board/car_config.h"

typedef enum {
    ENCODER_L = 0,      
    ENCODER_R,          
    ENCODER_COUNT
} EncoderIndex_t;

void Encoder_Init(void);
void Encoder_UpdateSpeed(uint32_t ms);
float Encoder_GetRPM(EncoderIndex_t id);
int16_t Encoder_GetPulse(EncoderIndex_t id);

#endif
