/**
 * @file    k230_uart.c
 * @brief   K230 UART 通信驱动 + ASCII 协议解析
 *
 * 透传层: UART1 中断接收 → 256 字节环形缓冲
 * 协议层: ASCII 行解析 "S<pos>,<vel>,<acc>\r\n" → 球状态
 */

#include "k230_uart.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"

/* ===================================================================== *
 *  透传层 — UART 环形缓冲 (ISR 写, 主循环读)
 * ===================================================================== */

#define K230_RING_SIZE              256U
#define K230_RING_MASK              (K230_RING_SIZE - 1U)

static volatile uint8_t g_rx_ring[K230_RING_SIZE];
static volatile uint8_t g_rx_head;
static          uint8_t g_rx_tail;

void UART1_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_K230_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(UART_K230_INST);
        uint8_t next = (g_rx_head + 1U) & K230_RING_MASK;
        if (next == g_rx_tail) {
            g_rx_tail = (g_rx_tail + 1U) & K230_RING_MASK;
        }
        g_rx_ring[g_rx_head] = byte;
        g_rx_head = next;
    }
}

static bool ring_get(uint8_t *byte)
{
    if (g_rx_head == g_rx_tail) return false;
    *byte = g_rx_ring[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1U) & K230_RING_MASK;
    return true;
}

void K230_UART_Init(void)
{
    g_rx_head = 0;
    g_rx_tail = 0;
    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

bool K230_UART_Available(void)
{
    return (g_rx_head != g_rx_tail);
}

uint16_t K230_UART_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    if (!buf || !max_len) return 0;
    while (count < max_len && ring_get(&buf[count])) count++;
    return count;
}

void K230_UART_SendByte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_K230_INST, byte);
}

void K230_UART_Send(const uint8_t *data, uint16_t len)
{
    if (!data || !len) return;
    for (uint16_t i = 0; i < len; i++)
        DL_UART_Main_transmitDataBlocking(UART_K230_INST, data[i]);
}

/* ===================================================================== *
 *  协议层 — ASCII 帧解析
 *
 *  协议: S<pos>,<vel>,<acc>\r\n
 *    pos = 球偏移位置 (mm), 浮点数
 *    vel = 球速度 (mm/s),   浮点数
 *    acc = 球加速度 (mm/s²), 浮点数
 * ===================================================================== */

#define K230_LINE_BUF_SIZE  32U

static char    g_line_buf[K230_LINE_BUF_SIZE];
static uint8_t g_line_idx;
static bool    g_in_frame;

static float g_ball_pos_raw;
static float g_ball_vel_raw;
static float g_ball_acc_raw;
static bool  g_ball_detected;
static bool  g_new_frame;

/* ── 手工 atof (无 libc 依赖) ── */

static float atof_simple(const char *s, const char **end)
{
    float val = 0.0f;
    bool  neg = false;

    if (*s == '-') { neg = true; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') {
            val += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }
    if (end) *end = s;
    return neg ? -val : val;
}

/* ── 单行解析 ── */

static void parse_k230_line(void)
{
    const char *p;
    const char *end;
    float pos, vel, acc;

    if (g_line_idx == 0 || g_line_buf[0] != 'S') return;

    p = &g_line_buf[1];
    pos = atof_simple(p, &end);
    if (end == p || *end != ',') return;
    p = end + 1;

    vel = atof_simple(p, &end);
    if (end == p || *end != ',') return;
    p = end + 1;

    acc = atof_simple(p, &end);
    (void)end;

    if (g_line_idx < 5) return;

    g_ball_pos_raw  = pos;
    g_ball_vel_raw  = vel;
    g_ball_acc_raw  = acc;
    g_ball_detected = true;
    g_new_frame     = true;
}

/* ── 流解析 (每 10ms 调用) ── */

void K230_ParseStream(void)
{
    uint8_t buf[32];
    uint16_t n = K230_UART_Read(buf, sizeof(buf));

    for (uint16_t i = 0; i < n; i++) {
        char c = (char)buf[i];

        if (c == 'S') {
            g_line_idx = 0;
            g_in_frame = true;
            g_line_buf[g_line_idx++] = c;
            continue;
        }
        if (c == '\n') {
            if (g_in_frame && g_line_idx > 1 && g_line_idx < K230_LINE_BUF_SIZE) {
                g_line_buf[g_line_idx] = '\0';
                parse_k230_line();
            }
            g_in_frame = false;
            g_line_idx = 0;
            continue;
        }
        if (c == '\r') continue;

        if (g_in_frame && g_line_idx < (K230_LINE_BUF_SIZE - 1U)) {
            g_line_buf[g_line_idx++] = c;
        } else if (g_line_idx >= (K230_LINE_BUF_SIZE - 1U)) {
            g_in_frame = false;
            g_line_idx = 0;
        }
    }
}

/* ── 数据访问器 ── */

float K230_GetBallPos(void)    { return g_ball_pos_raw; }
float K230_GetBallVel(void)    { return g_ball_vel_raw; }
float K230_GetBallAccel(void)  { return g_ball_acc_raw; }
bool  K230_IsBallDetected(void) { return g_ball_detected; }
bool  K230_HasNewFrame(void)    { return g_new_frame; }
