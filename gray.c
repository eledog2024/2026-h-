/**
 * @file    gray.c
 * @brief   8路灰度传感器驱动实现 (串行接口)
 * @note    通过 2 线串行协议 (CLK + DAT) 读取灰度传感器,
 *          计算红线位置.
 *
 *  协议: 自定义同步串行 (类 74HC165 移位寄存器)
 *    - CLK (输出): MCU 发出时钟
 *      · 高电平: 传感器将数据位更新到 DAT 线 (需 ≥5μs)
 *      · 低电平: MCU 读取 DAT 线
 *    - DAT (输入): 传感器串行输出, 上拉模式 (配合传感器开漏)
 *    - 每帧 8 位: 第1个时钟→通道0(bit0), ... 第8个时钟→通道7(bit7)
 *    - 帧同步: 读完 8 位后 CLK 保持低 >1ms, 传感器内部自动归零
 *
 *  接线:
 *    GRAY_CLK (PA12) → 传感器 CLK
 *    GRAY_DAT (PA13) → 传感器 DAT
 *    GND             → 传感器 GND
 *    VCC (3.3V/5V)   → 传感器供电 (取决于辅助板)
 *
 *  适用场景: 白底红线循迹
 *    传感器通过调节阈值电位器区分红色和白色:
 *    - 红线反射率低于白底 → 传感器输出 1 (深色/检测到线)
 *    - 白底反射率高        → 传感器输出 0 (亮色/背景)
 *
 *  如果传感器逻辑相反 (0=线, 1=背景), 定义 GRAY_INVERT_LOGIC 即可.
 *
 *  位置算法 (最长连续段 + 中心优先, 8路):
 *    1. 扫描 raw[0..7], 找出所有最长连续段
 *    2. 仅1个最长段 → 用它计算加权平均 (孤立误判被忽略)
 *    3. 多个等长段 → 选最靠近传感器中心 (索引 3.5) 的段
 *    4. 多个段到中心距离也相同 → 退化为全部检测点加权平均
 *    5. position = Σ(i × s[i]) / Σs[i], 映射: (avg/7) × 160
 *
 *  示例 (●=红线, ○=背景):
 *    ○ ○ ● ● ○ ○ ● ○  → 最长段[3,4], 忽略通道7, pos≈57
 *    ● ● ○ ○ ● ● ○ ○  → [1,2]vs[4,5], [4,5]更近中心, pos≈80
 *    ● ● ○ ○ ○ ○ ● ●  → [1,2]vs[7,8]等距, 退化全部4点, pos≈80
 *
 *  滤波:
 *    低通滤波 (EMA α=0.5) 平滑位置输出, 抑制传感器离散性抖动
 *
 *  红线 vs 黑线注意事项:
 *    红线与白底的反射率差异小于黑线与白底, 需仔细调节每个通道的
 *    阈值电位器, 确保红线能被可靠识别. 如信号不稳定, 可适当增大
 *    GRAY_POS_FILTER_ALPHA 降低滤波响应速度来提高稳定性.
 */

#include "gray.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ===================================================================== *
 *  静态变量                                                                *
 * ===================================================================== */

/** 最近一次原始传感器值 (位图: bit0=CH0, ..., bit7=CH7) */
static uint8_t g_gray_raw = 0x00;

/** 上一次有效位置 (用于无线时保持) */
static int16_t g_last_position = GRAY_POS_CENTER;

/* ===================================================================== *
 *  辅助宏                                                                  *
 * ===================================================================== */

/** 位置低通滤波系数 → 见 Board/car_config.h [B] 节 */
#define GRAY_POS_FILTER_ALPHA   CAR_GRAY_FILTER_ALPHA

/** CLK 高电平持续时间 (μs) → 见 Board/car_config.h [B] 节 */
#define GRAY_CLK_HIGH_US        CAR_GRAY_CLK_HIGH_US

/** CLK 低电平持续时间 (μs) → 见 Board/car_config.h [B] 节 */
#define GRAY_CLK_LOW_US         CAR_GRAY_CLK_LOW_US

/** 帧间隔 (μs) → 见 Board/car_config.h [B] 节 */
#define GRAY_FRAME_GAP_US       CAR_GRAY_FRAME_GAP_US

/* ===================================================================== *
 *  微秒延时 (32MHz 主频)                                                    *
 * ===================================================================== */

/**
 * @brief 阻塞微秒延时
 * @param us 微秒数
 * @note  32MHz: 每微秒约 32 个时钟周期
 *        使用 __NOP() + 循环, 粗略但满足时序要求
 */
static inline void delay_us(uint32_t us)
{
    /*
     * 32MHz: 1 cycle ≈ 31.25ns
     * 粗略估算: 每次循环约 4 cycles (decrement + branch + 2xNOP)
     * us * 8 ≈ us * 32 / 4
     */
    for (uint32_t i = 0; i < (us * 8); i++) {
        __NOP();
    }
}

/* ===================================================================== *
 *  串行协议读取                                                             *
 * ===================================================================== */

/**
 * @brief 通过串行协议读取 8 通道数据
 * @return 8 位位图, bit0=通道0(最左), bit7=通道7(最右)
 *
 * @note  协议时序:
 *        ┌──┐     ┌──┐     ┌──┐              ┌──────────────── >1ms 归零
 *   CLK  ┘  └─────┘  └─────┘  └── ... ──────┘
 *         ↑  ↑    ↑  ↑    ↑  ↑
 *         │  └读  │  └读  │  └读 DAT (bit2~bit7)
 *         │  DAT  │  DAT  │
 *         │(bit0) │(bit1)│
 *         └≥5μs   └≥5μs  └≥5μs
 *         传感器  传感器  传感器
 *         更新数据 更新数据 更新数据
 */
static uint8_t Gray_ReadSerial(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        /* Step 1: CLK 拉高 → 传感器更新数据到 DAT 线 (需 ≥5μs) */
        DL_GPIO_setPins(GRAY_CLK_PORT, GRAY_CLK_PIN);
        delay_us(GRAY_CLK_HIGH_US);

        /* Step 2: CLK 拉低 → 稳定后 MCU 读取 DAT 线 */
        DL_GPIO_clearPins(GRAY_CLK_PORT, GRAY_CLK_PIN);
        delay_us(GRAY_CLK_LOW_US);

        /* Step 3: 读 DAT, bit0 对应第 1 个时钟 (通道 0) */
        if (DL_GPIO_readPins(GRAY_DAT_PORT, GRAY_DAT_PIN) != 0) {
            data |= (uint8_t)(1 << i);
        }
    }

    /* 帧间隔 >1ms: 触发传感器内部移位寄存器归零, 下一帧从通道0重新开始 */
    delay_us(GRAY_FRAME_GAP_US);

    return data;
}

/* ===================================================================== *
 *  API 实现                                                                *
 * ===================================================================== */

/**
 * @brief 初始化灰度传感器串行接口 GPIO
 * @note  CLK (PA12): 推挽输出, 初始低电平 — 由 SysConfig GPIO_GRAY 实例配置
 *        DAT (PA13): 上拉输入 — 由 SysConfig GPIO_GRAY 实例配置
 *        引脚初始化在 SYSCFG_DL_GPIO_init() 中自动完成,
 *        本函数仅复位软件状态.
 */
void Gray_Init(void)
{
    /* CLK 初始低电平 (SYSCFG_DL_GPIO_init 已配置为输出) */
    DL_GPIO_clearPins(GRAY_CLK_PORT, GRAY_CLK_PIN);

    /* 复位状态 */
    g_gray_raw = 0x00;
    g_last_position = GRAY_POS_CENTER;
}

/**
 * @brief 读取 8 通道原始值 (串行协议)
 */
void Gray_ReadRaw(uint8_t raw[GRAY_CHANNEL_COUNT])
{
    uint8_t data = Gray_ReadSerial();

#ifdef GRAY_INVERT_LOGIC
    data = ~data;
#endif

    g_gray_raw = data;

    for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        raw[i] = (data & (1 << i)) ? 1 : 0;
    }
}

/**
 * @brief 获取检测到红线的通道数
 */
uint8_t Gray_GetLineCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        if (g_gray_raw & (1 << i)) {
            count++;
        }
    }
    return count;
}

/**
 * @brief 计算红线中心位置 (最长连续段 + 中心优先, 8路)
 *
 * @note  抗误判策略:
 *        1. 找最长连续段, 孤立误判点被自动忽略
 *        2. 多个等长最长段 → 优先选靠近传感器中心的段
 *        3. 若多个段到中心距离也相同 → 全部检测点加权平均
 *
 *  示例 (●=红线, ○=背景):
 *    ○ ○ ● ● ○ ○ ● ○  → 最长段=[3,4], 忽略通道7, pos≈57
 *    ● ● ○ ○ ● ● ○ ○  → 等长[1,2]vs[4,5], [4,5]更靠近中心, pos≈86
 *    ● ● ○ ○ ○ ○ ● ●  → 等长[1,2]vs[7,8], 等距, 退化全部4点加权
 *    ○ ○ ● ● ● ○ ○ ○  → 最长段=[3,4,5], pos≈57
 */
int16_t Gray_GetPosition(void)
{
    uint8_t raw[GRAY_CHANNEL_COUNT];
    Gray_ReadRaw(raw);

    /* ---- 统计全部检测到的通道数 ---- */
    uint8_t total_count = Gray_GetLineCount();

    /* ---- 寻找所有最长连续段 ---- */
    uint8_t best_starts[GRAY_CHANNEL_COUNT]; /* 最长段起始位置列表 */
    uint8_t best_len = 0;
    uint8_t best_cnt = 0;                    /* 最长段的数量 */
    uint8_t in_seg = 0;
    uint8_t seg_start = 0;

    for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        if (raw[i]) {
            if (!in_seg) {
                seg_start = i;
                in_seg = 1;
            }
        } else {
            if (in_seg) {
                uint8_t seg_len = i - seg_start;
                if (seg_len > best_len) {
                    best_len = seg_len;
                    best_starts[0] = seg_start;
                    best_cnt = 1;
                } else if (seg_len == best_len && seg_len > 0) {
                    if (best_cnt < GRAY_CHANNEL_COUNT) {
                        best_starts[best_cnt] = seg_start;
                        best_cnt++;
                    }
                }
                in_seg = 0;
            }
        }
    }
    /* 处理末尾的连续段 */
    if (in_seg) {
        uint8_t seg_len = GRAY_CHANNEL_COUNT - seg_start;
        if (seg_len > best_len) {
            best_len = seg_len;
            best_starts[0] = seg_start;
            best_cnt = 1;
        } else if (seg_len == best_len && seg_len > 0) {
            if (best_cnt < GRAY_CHANNEL_COUNT) {
                best_starts[best_cnt] = seg_start;
                best_cnt++;
            }
        }
    }

    /* ---- 计算位置 ---- */
    int16_t position = GRAY_POS_CENTER;
    uint16_t weighted_sum = 0;
    uint8_t  count = 0;

    if (total_count == 0) {
        /* 没有检测到红线: 保持上次位置 */
        position = g_last_position;
    } else if (best_cnt == 1) {
        /* 唯一最长连续段: 直接使用 */
        for (uint8_t i = best_starts[0]; i < best_starts[0] + best_len; i++) {
            weighted_sum += (uint16_t)i * 100;
            count++;
        }
    } else {
        /*
         * 多个等长最长段 → 选最靠近传感器中心 (索引 3.5) 的段
         * 段中心 = seg_start + (best_len - 1) * 0.5
         */
        const float CENTER = 3.5f;
        float best_dist = 255.0f;
        uint8_t closest_cnt = 0;
        uint8_t closest_starts[GRAY_CHANNEL_COUNT];

        for (uint8_t s = 0; s < best_cnt; s++) {
            float seg_center = (float)best_starts[s] + (float)(best_len - 1) * 0.5f;
            float dist = (seg_center > CENTER) ? (seg_center - CENTER)
                                               : (CENTER - seg_center);
            if (dist < best_dist - 0.01f) {
                best_dist = dist;
                closest_starts[0] = best_starts[s];
                closest_cnt = 1;
            } else if (dist < best_dist + 0.01f) {
                /* 距离相同 (对称分布, 如 [1,2] 和 [7,8]) */
                if (closest_cnt < GRAY_CHANNEL_COUNT) {
                    closest_starts[closest_cnt] = best_starts[s];
                    closest_cnt++;
                }
            }
        }

        if (closest_cnt == 1) {
            /* 唯一最靠近中心的段: 仅用它, 其他等长段视为误判 */
            uint8_t start = closest_starts[0];
            for (uint8_t i = start; i < start + best_len; i++) {
                weighted_sum += (uint16_t)i * 100;
                count++;
            }
        } else {
            /*
             * 多个段到中心距离相同 → 全部检测点加权平均
             * (如 [1,2] 和 [7,8] 对称, 无法确定哪段是真线)
             */
            for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++) {
                if (raw[i]) {
                    weighted_sum += (uint16_t)i * 100;
                    count++;
                }
            }
        }
    }

    if (count > 0) {
        float avg = (float)weighted_sum / (float)(count * 100);
        /* 8路通道索引范围 0~7, 分母=7, 映射到 0~160 (中心=80) */
        position = (int16_t)((avg / 7.0f) * 160.0f);

        if (position < GRAY_POS_MIN) position = GRAY_POS_MIN;
        if (position > GRAY_POS_MAX) position = GRAY_POS_MAX;
    }

    /* ---- 低通滤波平滑 ---- */
    static float filtered_pos = (float)GRAY_POS_CENTER;
    filtered_pos = GRAY_POS_FILTER_ALPHA * (float)position
                 + (1.0f - GRAY_POS_FILTER_ALPHA) * filtered_pos;
    position = (int16_t)filtered_pos;

    /* 保存有效位置 */
    if (count > 0) {
        g_last_position = position;
    }

    return position;
}

/**
 * @brief 获取最近一次读取的原始传感器值
 */
uint8_t Gray_GetRawByte(void)
{
    return g_gray_raw;
}
