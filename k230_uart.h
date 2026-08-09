/**
 * @file    k230_uart.h
 * @brief   K230 视觉协处理器 — UART 通信 + ASCII 协议解析
 *
 * 硬件:  UART1, 115200 baud, 8N1
 * 接线:  PA8 (TX), PA9 (RX) — SysConfig 配置
 * 协议:  ASCII 字符串 "S<pos>,<vel>,<acc>\r\n"
 *
 * 本模块同时提供:
 *   - UART 环形缓冲 + 中断接收 (透传层)
 *   - ASCII 帧解析 + 球位置/速度/加速度 提取 (协议层)
 */

#ifndef K230_UART_H
#define K230_UART_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================== *
 *  透传层 API (环形缓冲)
 * ===================================================================== */

void     K230_UART_Init(void);
bool     K230_UART_Available(void);
uint16_t K230_UART_Read(uint8_t *buf, uint16_t max_len);
void     K230_UART_SendByte(uint8_t byte);
void     K230_UART_Send(const uint8_t *data, uint16_t len);

/* ===================================================================== *
 *  协议层 API (ASCII 解析)
 * ===================================================================== */

/**
 * @brief 从环形缓冲读取并解析一帧 K230 数据
 *
 * 每个 10ms 控制周期调用一次.
 * 内部非阻塞, 解析成功后更新内部变量.
 */
void K230_ParseStream(void);

/** @brief 球偏移位置 (mm), 相对中心 O, + = 偏向自由端 */
float K230_GetBallPos(void);

/** @brief 球速度 (mm/s) */
float K230_GetBallVel(void);

/** @brief 球加速度 (mm/s²) */
float K230_GetBallAccel(void);

/** @brief 球是否被检测到 */
bool  K230_IsBallDetected(void);

/** @brief 自上次 ParseStream 后是否有新帧 */
bool  K230_HasNewFrame(void);

#endif /* K230_UART_H */
