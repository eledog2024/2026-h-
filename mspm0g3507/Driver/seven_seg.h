

/* 数码管显示接口。 */
#ifndef SEVEN_SEG_H
#define SEVEN_SEG_H

#include <stdint.h>

void SEG_Init(void);

void SEG_DisplayNum(uint16_t num);

void SEG_DisplayTime(uint16_t centiseconds);

void SEG_DisplayNumColon(uint16_t num);

void SEG_DisplayOff(void);

void SEG_DisplaySigned(int16_t num);

void SEG_SetBrightness(uint8_t level);

#endif
