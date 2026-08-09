/**
 * @file    imu.c
 * @brief   HWT906 9轴 IMU 传感器驱动 (UART 接口, WitMotion 协议)
 *
 * 传感器: WitMotion HWT906 (MPU-9250 + 片载卡尔曼滤波)
 * 通信:   UART3, 115200 baud, 8N1, 连续输出模式
 *
 * 接线: PB2 (UART3_TX), PB3 (UART3_RX)
 *       (引脚映射由 SysConfig 管理，见 empty.syscfg)
 *
 * WitMotion UART 协议 (连续输出):
 *   帧格式: 0x55 + Tag + Data[0..N-1] + Checksum
 *   Checksum = (Tag + SUM(Data)) & 0xFF
 *
 *   0x51 (加速度帧): 0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM
 *                   共 8 数据字节, 加速度在 data[0..5]
 *                   SUM = 0x55+0x51+AxL+AxH+AyL+AyH+AzL+AzH+TL+TH (取低8位)
 *                   转换: accel = raw / 32768 * 16g (m/s²)
 *
 *   数据格式: 低字节在前 (little-endian), 有符号 int16
 */

#include "imu.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

/* ===================================================================== *
 *  常量
 * ===================================================================== */

/* raw int16 → m/s² 的转换系数: 16.0f * 9.8f / 32768.0f */
#define IMU_RAW_TO_MS2             0.00478515625f

/* 环形缓冲区 (1024B ≈ 85帧, 921600下~850ms缓冲) */
#define IMU_RING_SIZE              1024U
#define IMU_RING_MASK              (IMU_RING_SIZE - 1U)

/* 0x51 帧数据字节数: Ax(2)+Ay(2)+Az(2)+TL(1)+TH(1) = 8 */
#define IMU_FRAME_DATA_LEN         8U

/* ===================================================================== *
 *  帧解析器状态
 * ===================================================================== */

typedef enum {
    IMU_PARSE_SYNC,       /* 等待 0x55 帧头 */
    IMU_PARSE_TAG,        /* 读取标签字节 */
    IMU_PARSE_DATA,       /* 收集数据字节 + 校验 */
} imu_parse_state_t;

/* ===================================================================== *
 *  静态变量
 * ===================================================================== */

/* ---- UART RX 环形缓冲区 (ISR 写入, 主循环读取) ---- */
static volatile uint8_t g_rx_ring[IMU_RING_SIZE];
static volatile uint8_t g_rx_head;       /* ISR 写入索引 */
static volatile uint8_t g_rx_tail;       /* 解析器读取索引 (ISR 溢出时修改) */

/* ---- 帧解析器 ---- */
static imu_parse_state_t g_state     = IMU_PARSE_SYNC;
static uint8_t           g_tag;
static uint8_t           g_data[IMU_FRAME_DATA_LEN];
static uint8_t           g_data_idx;
static uint8_t           g_checksum;

/* ---- 最新解析的加速度 (m/s²) ---- */
static volatile float  g_accel_x;
static volatile float  g_accel_y;
static volatile float  g_accel_z;
static volatile int16_t g_raw_accel_x;
static volatile int16_t g_raw_accel_y;
static volatile int16_t g_raw_accel_z;

/* ---- 初始化标志 ---- */
static bool  g_ready;
static volatile bool g_frame_valid;   /* 标记是否成功解析过至少一帧 */

/* ---- 帧统计 (调试用) ---- */
static volatile uint32_t g_cnt_51;
static volatile uint32_t g_cnt_53;
static volatile uint32_t g_cnt_other;
static volatile uint8_t  g_last_tag;

/* ===================================================================== *
 *  UART RX 中断服务例程
 * ===================================================================== */

/**
 * @brief UART3 RX 中断: 将收到的字节存入环形缓冲区
 *
 * @note  使能了 UART RX 超时中断 (DL_UART_MAIN_INTERRUPT_RX),
 *        每个接收字节触发一次, 也可以在 FIFO 达到阈值时触发.
 *        这里用 FIFO 非空轮询方式读出所有可用字节.
 */
void UART3_IRQHandler(void)
{
    /*
     * 循环读取直到 RX FIFO 为空, 避免每次中断只读一个字节.
     * DL_UART_Main_receiveData() 会清空中断标志.
     */
    while (DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST) == false) {
        uint8_t byte = DL_UART_Main_receiveData(UART_IMU_INST);
        uint8_t next  = (g_rx_head + 1U) & IMU_RING_MASK;

        /* 如果环形缓冲区满了, 丢弃最老的数据 (tail 向前移) */
        if (next == g_rx_tail) {
            g_rx_tail = (g_rx_tail + 1U) & IMU_RING_MASK;
        }
        g_rx_ring[g_rx_head] = byte;
        g_rx_head = next;
    }
}

/* ===================================================================== *
 *  环形缓冲区辅助
 * ===================================================================== */

/**
 * @brief 从环形缓冲区取一个字节
 * @param byte 输出: 读取到的字节
 * @return true=成功, false=缓冲区空
 */
static bool ring_get(uint8_t *byte)
{
    if (g_rx_head == g_rx_tail) {
        return false;  /* 缓冲区空 */
    }
    *byte = g_rx_ring[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1U) & IMU_RING_MASK;
    return true;
}

/* ===================================================================== *
 *  帧解析器
 * ===================================================================== */

/**
 * @brief 解析 0x51 加速度帧, 更新全局加速度变量
 *
 *        0x51 帧格式: 0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM
 *        数据字节共 8 个, Ax/Ay/Az 在 data[0..5]
 */
static void imu_parse_frame(void)
{
    int16_t raw_ax, raw_ay, raw_az;

    /* Ax=D[0,1], Ay=D[2,3], Az=D[4,5] */
    raw_ax = (int16_t)(((uint16_t)g_data[1] << 8) | (uint16_t)g_data[0]);
    raw_ay = (int16_t)(((uint16_t)g_data[3] << 8) | (uint16_t)g_data[2]);
    raw_az = (int16_t)(((uint16_t)g_data[5] << 8) | (uint16_t)g_data[4]);

    /* 保存原始 int16 值 */
    g_raw_accel_x = raw_ax;
    g_raw_accel_y = raw_ay;
    g_raw_accel_z = raw_az;

    /* 转换为工程单位 (m/s²): accel = raw / 32768 * 16 * 9.8 */
    g_accel_x = IMU_RAW_TO_MS2 * (float)raw_ax;
    g_accel_y = IMU_RAW_TO_MS2 * (float)raw_ay;
    g_accel_z = IMU_RAW_TO_MS2 * (float)raw_az;
    g_frame_valid = true;
    g_cnt_51++;
}

/**
 * @brief 从环形缓冲区读取并解析字节流
 *
 *        状态机:
 *          SYNC → (0x55) → TAG → (0x51) → DATA → (checksum OK) → parse → SYNC
 *               ↓                    ↓                    ↓
 *              (其他)             (非0x51)          (checksum 失败)
 *               └─ 继续等待        └─→ SYNC           └─→ SYNC
 *
 * @note  此函数由公开 API 调用, 在调用上下文中处理.
 *        非原子操作, 应在主循环/非 ISR 上下文调用.
 */
static void imu_parse_stream(void)
{
    uint8_t byte;

    while (ring_get(&byte)) {
        switch (g_state) {

        case IMU_PARSE_SYNC:
            if (byte == 0x55U) {
                g_state    = IMU_PARSE_TAG;
                g_checksum = 0x55U;          /* SUM 包含 0x55 帧头 */
            }
            break;

        case IMU_PARSE_TAG:
            g_tag       = byte;
            g_checksum += byte;               /* SUM = 0x55 + tag + data... */
            g_data_idx  = 0U;

            g_last_tag = byte;
            if (byte == 0x51U) {
                /* 处理加速度帧 */
                g_state = IMU_PARSE_DATA;
            } else {
                /* 记录其他标签, 回到同步状态 */
                if (byte == 0x53U) {
                    g_cnt_53++;
                } else {
                    g_cnt_other++;
                }
                g_state = IMU_PARSE_SYNC;
                continue;
            }
            break;

        case IMU_PARSE_DATA:
            if (g_data_idx < IMU_FRAME_DATA_LEN) {
                /* 收集数据字节 */
                g_data[g_data_idx] = byte;
                g_checksum += byte;
                g_data_idx++;
            } else {
                /* 数据收齐, 这是校验和字节 */
                if ((g_checksum & 0xFFU) == byte) {
                    /* 校验通过 → 解析帧 */
                    imu_parse_frame();
                }
                /* 无论校验是否通过, 都回到 SYNC 准备下一个帧 */
                g_state = IMU_PARSE_SYNC;
            }
            break;
        }
    }
}

/* ===================================================================== *
 *  公开 API
 * ===================================================================== */

/**
 * @brief 初始化 UART 外设并等待 HWT906 上电稳定
 *
 *        前置条件: SYSCFG_DL_init() 已完成引脚、电源、时钟配置。
 *        本函数的工作:
 *        1. 使能 UART RX 中断和 NVIC
 *        2. 等待 HWT906 上电稳定并收到首帧有效数据
 */
void IMU_Init(void)
{
    /* ---- 1. 使能 UART RX 中断 ---- */
    DL_UART_Main_enableInterrupt(UART_IMU_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);

    /* ---- 2. 初始化加速度变量 ---- */
    g_accel_x     = 0.0f;
    g_accel_y     = 0.0f;
    g_accel_z     = 0.0f;
    g_raw_accel_x = 0;
    g_raw_accel_y = 0;
    g_raw_accel_z = 0;
    g_frame_valid = false;

    /*
     * ---- 3. 等待 HWT906 上电稳定并收到首帧有效数据 ----
     *
     * HWT906 上电后需要 500~1000ms 才能开始稳定输出连续数据.
     * 这里在 ~1000ms 延时内持续解析, 直到成功收到至少一帧.
     *
     * 注意: 如果 HWT906 只输出 0x53 (角度) 而没有使能 0x51 (加速度),
     * 此处会超时返回, 后续所有加速度读数恒为 0.
     * 解决方法: 用 MiniIMU 上位机软件, 连接模块后勾选"加速度"输出并保存.
     */
    {
        uint32_t timeout = 4000000U;  /* ~1000ms @80MHz */
        while (!g_frame_valid && timeout > 0U) {
            imu_parse_stream();
            timeout--;
        }
    }

    g_ready = true;
}

/**
 * @brief 读取三轴加速度
 */
void IMU_ReadAccel(float *ax, float *ay, float *az)
{
    /* 解析最新的帧数据 */
    imu_parse_stream();

    if (ax != NULL) {
        *ax = g_accel_x;
    }
    if (ay != NULL) {
        *ay = g_accel_y;
    }
    if (az != NULL) {
        *az = g_accel_z;
    }
}

/**
 * @brief 读取 Y 轴加速度 (小车前进方向)
 * @return Y 轴加速度 (m/s²)
 */
float IMU_ReadAccelY(void)
{
    imu_parse_stream();
    return g_accel_y;
}

/**
 * @brief 读取最近一次解析的原始 int16 加速度值 (调试用)
 */
void IMU_ReadRawAccel(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az)
{
    /* 先解析最新帧 */
    imu_parse_stream();

    if (raw_ax != NULL) { *raw_ax = g_raw_accel_x; }
    if (raw_ay != NULL) { *raw_ay = g_raw_accel_y; }
    if (raw_az != NULL) { *raw_az = g_raw_accel_z; }
}

/**
 * @brief 调试: 获取帧统计信息
 */
void IMU_GetFrameStats(uint32_t *cnt_51, uint32_t *cnt_53,
                       uint32_t *cnt_other, uint8_t *last_tag)
{
    if (cnt_51   != NULL) { *cnt_51   = g_cnt_51; }
    if (cnt_53   != NULL) { *cnt_53   = g_cnt_53; }
    if (cnt_other != NULL) { *cnt_other = g_cnt_other; }
    if (last_tag != NULL) { *last_tag = g_last_tag; }
}
