/* K230 串口通信与球状态解析。 */
#include "k230_uart.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"

#define K230_RING_SIZE 256U
#define K230_RING_MASK (K230_RING_SIZE - 1U)
#define K230_LINE_BUF_SIZE 32U

static volatile uint8_t g_rx_ring[K230_RING_SIZE];
static volatile uint8_t g_rx_head;
static uint8_t g_rx_tail;
static char g_line_buf[K230_LINE_BUF_SIZE];
static uint8_t g_line_idx;
static bool g_in_frame;
static float g_ball_pos_raw;
static float g_ball_vel_raw;
static float g_ball_acc_raw;
static bool g_ball_detected;
static bool g_new_frame;

/* 接收 K230 串口数据。 */
void UART1_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_K230_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(UART_K230_INST);
        uint8_t next = (g_rx_head + 1U) & K230_RING_MASK;
        if (next == g_rx_tail) g_rx_tail = (g_rx_tail + 1U) & K230_RING_MASK;
        g_rx_ring[g_rx_head] = byte;
        g_rx_head = next;
    }
}

/* 读取一字节缓存数据。 */
static bool ring_get(uint8_t *byte)
{
    if (g_rx_head == g_rx_tail) return false;
    *byte = g_rx_ring[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1U) & K230_RING_MASK;
    return true;
}

/* 初始化 K230 串口。 */
void K230_UART_Init(void)
{
    g_rx_head = 0;
    g_rx_tail = 0;
    g_ball_detected = false;
    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

/* 判断串口缓存是否有数据。 */
bool K230_UART_Available(void) { return g_rx_head != g_rx_tail; }

/* 读取串口缓存数据。 */
uint16_t K230_UART_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    if (!buf || !max_len) return 0;
    while (count < max_len && ring_get(&buf[count])) count++;
    return count;
}

/* 发送一个串口字节。 */
void K230_UART_SendByte(uint8_t byte) { DL_UART_Main_transmitDataBlocking(UART_K230_INST, byte); }

/* 发送串口数据块。 */
void K230_UART_Send(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (!data || !len) return;
    for (i = 0; i < len; i++) DL_UART_Main_transmitDataBlocking(UART_K230_INST, data[i]);
}

/* 解析简易浮点数。 */
static float atof_simple(const char *s, const char **end)
{
    float value = 0.0f, frac = 0.1f;
    bool neg = false;
    if (*s == '-') { neg = true; s++; } else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') { value = value * 10.0f + (float)(*s++ - '0'); }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { value += (float)(*s++ - '0') * frac; frac *= 0.1f; }
    }
    if (end) *end = s;
    return neg ? -value : value;
}

/* 解析一行球状态数据。 */
static void parse_k230_line(void)
{
    const char *p = &g_line_buf[1];
    const char *end;
    float pos, vel, acc;
    if (g_line_idx == 0 || g_line_buf[0] != 'S') return;
    pos = atof_simple(p, &end); if (end == p || *end != ',') return;
    vel = atof_simple(end + 1, &end); if (*end != ',') return;
    acc = atof_simple(end + 1, &end);
    g_ball_pos_raw = pos;
    g_ball_vel_raw = vel;
    g_ball_acc_raw = acc;
    g_ball_detected = true;
    g_new_frame = true;
}

/* 解析 K230 数据流。 */
void K230_ParseStream(void)
{
    uint8_t buf[32];
    uint16_t i, n = K230_UART_Read(buf, sizeof(buf));
    g_new_frame = false;
    for (i = 0; i < n; i++) {
        char c = (char)buf[i];
        if (c == 'N') { g_ball_detected = false; g_in_frame = false; g_line_idx = 0; continue; }
        if (c == 'S') { g_line_idx = 0; g_in_frame = true; g_line_buf[g_line_idx++] = c; continue; }
        if (c == '\n') {
            if (g_in_frame && g_line_idx > 1) { g_line_buf[g_line_idx] = '\0'; parse_k230_line(); }
            g_in_frame = false; g_line_idx = 0; continue;
        }
        if (c == '\r') continue;
        if (g_in_frame && g_line_idx < K230_LINE_BUF_SIZE - 1U) g_line_buf[g_line_idx++] = c;
        else if (g_in_frame) { g_in_frame = false; g_line_idx = 0; }
    }
}

/* 读取球位置。 */
float K230_GetBallPos(void) { return g_ball_pos_raw; }
/* 读取球速度。 */
float K230_GetBallVel(void) { return g_ball_vel_raw; }
/* 读取球加速度。 */
float K230_GetBallAccel(void) { return g_ball_acc_raw; }
/* 判断是否检测到球。 */
bool K230_IsBallDetected(void) { return g_ball_detected; }
/* 判断是否收到新帧。 */
bool K230_HasNewFrame(void) { return g_new_frame; }