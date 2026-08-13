

/* 灰度传感器接口。 */
#ifndef GRAY_H
#define GRAY_H

#include <stdint.h>
#include <stdbool.h>
#include "Board/car_config.h"

#define GRAY_POS_MIN          0
#define GRAY_POS_MAX          160
#define GRAY_POS_CENTER       80

void Gray_Init(void);

void Gray_ReadRaw(uint8_t raw[GRAY_CHANNEL_COUNT]);

int16_t Gray_GetPosition(void);

uint8_t Gray_GetLineCount(void);

uint8_t Gray_GetRawByte(void);

#endif
