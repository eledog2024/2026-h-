/**
 * @file    seven_seg.h
 * @brief   TM1637 4 位数码管驱动 (软件 2 线协议)
 *
 * 接线:
 *   CLK → PB20
 *   DIO → PA28
 *   VCC → 3.3V / 5V
 *   GND → GND
 *
 * 使用:
 *   SEG_Init();
 *   SEG_DisplayTime(1234);   // 显示 "12.34" (秒.百分秒)
 *   SEG_DisplayNum(5678);    // 显示 "5678"
 *   SEG_DisplayOff();        // 关闭显示
 */

#ifndef SEVEN_SEG_H
#define SEVEN_SEG_H

#include <stdint.h>

void SEG_Init(void);

/** @brief 显示 4 位数字 (0000~9999) */
void SEG_DisplayNum(uint16_t num);

/** @brief 显示时间 "SS.CC" 格式 (秒.百分秒), 中间冒号点亮 */
void SEG_DisplayTime(uint16_t centiseconds);

/** @brief 显示 4 位数字, 中间带冒号 */
void SEG_DisplayNumColon(uint16_t num);

/** @brief 关闭显示 (低功耗) */
void SEG_DisplayOff(void);

/** @brief 显示有符号数 (-999~9999), 负号自动定位 */
void SEG_DisplaySigned(int16_t num);

/** @brief 设置亮度 0~7 (默认 2) */
void SEG_SetBrightness(uint8_t level);

#endif /* SEVEN_SEG_H */
