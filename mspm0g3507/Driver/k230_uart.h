

/* K230 串口与球状态接口。 */
#ifndef K230_UART_H
#define K230_UART_H

#include <stdint.h>
#include <stdbool.h>

void     K230_UART_Init(void);
bool     K230_UART_Available(void);
uint16_t K230_UART_Read(uint8_t *buf, uint16_t max_len);
void     K230_UART_SendByte(uint8_t byte);
void     K230_UART_Send(const uint8_t *data, uint16_t len);

void K230_ParseStream(void);

float K230_GetBallPos(void);

float K230_GetBallVel(void);

float K230_GetBallAccel(void);

bool  K230_IsBallDetected(void);

bool  K230_HasNewFrame(void);

#endif
