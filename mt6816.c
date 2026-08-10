/**
 * @file    mt6816.c
 * @brief   MT6816 磁编码器驱动实现
 *
 * 架构:
 *   - AB 相: GPIO 双边沿中断 + 软件 4X 正交解码
 *   - PWM:   GPIO 双边沿中断 + TIMG0 时间戳 → 占空比 → 角度
 *   - Z 相:  GPIO 上升沿中断 → 校准零点
 *
 * 中断: 所有引脚共用 PORTA GROUP1 中断,
 *       MT6816_IRQHandler() 由 GROUP1_IRQHandler (encoder.c) 调用.
 */

#include "mt6816.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

/* ===================================================================== *
 *  硬件常量
 * ===================================================================== */

/* ----- GPIO (SysConfig 生成, 跨 GPIOA+GPIOB 双端口) ----- */
#define ENC_ALL_PINS \
    (GPIO_MT6816_PIN_ENC_A_PIN | GPIO_MT6816_PIN_ENC_B_PIN | \
     GPIO_MT6816_PIN_ENC_PWM_PIN | GPIO_MT6816_PIN_ENC_Z_PIN)

/* GPIOA: ENC_A (PIN_31) + ENC_Z (PIN_0) */
#define ENC_PINS_PORTA \
    (GPIO_MT6816_PIN_ENC_A_PIN | GPIO_MT6816_PIN_ENC_Z_PIN)

/* GPIOB: ENC_B (PIN_6) + ENC_PWM (PIN_7) */
#define ENC_PINS_PORTB \
    (GPIO_MT6816_PIN_ENC_B_PIN | GPIO_MT6816_PIN_ENC_PWM_PIN)

/* 时间戳定时器: TIMG0 @ 32MHz 自由运行 (16-bit wrap) */
#define ENC_TS_TIMER                  TIMG0
#define ENC_TS_MAX                    0xFFFFU

/* ===================================================================== *
 *  内部状态
 * ===================================================================== */

/* ── 正交编码器 ── */
static volatile int32_t  g_enc_count;
static volatile bool     g_z_detected;
static volatile int32_t  g_enc_at_z;

/* ── PWM 测量 ── */
static volatile uint16_t g_pwm_rise_ts;
static volatile uint16_t g_pwm_high_ticks;
static volatile uint16_t g_pwm_period_ticks;
static volatile bool     g_pwm_fresh;

/* ── 计算输出 ── */
static float   g_angle;
static float   g_velocity;
static float   g_pwm_raw;
static int32_t g_enc_prev;
static int32_t g_delta_prev;
static bool    g_ready;

/* ===================================================================== *
 *  内部: 16-bit 时间差 (自动处理 wrap)
 * ===================================================================== */

/** 两次读数间隔 < 32767 ticks (~1ms), PWM 周期 ~257us, 安全 */
static inline uint16_t ts_diff(uint16_t now, uint16_t prev)
{
    return (uint16_t)(now - prev);
}

/* ===================================================================== *
 *  GPIOA 中断分发 (由 GROUP1_IRQHandler 调用)
 * ===================================================================== */

void MT6816_IRQHandler(void)
{
    uint32_t stat_a;
    uint32_t stat_b;
    uint32_t stat_pwm;
    uint32_t stat_z;
    bool a_high;
    bool b_high;
    bool pwm_high;
    uint16_t ts;

    /* 快照时间戳 (降低延迟抖动)
     * MT6816 引脚跨 GPIOA + GPIOB, 分端口读取中断状态和引脚电平 */
    ts = (uint16_t)DL_TimerG_getTimerCount(ENC_TS_TIMER);
    stat_a = DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_A_PORT,
                                               GPIO_MT6816_PIN_ENC_A_PIN);
    stat_b = DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_B_PORT,
                                               GPIO_MT6816_PIN_ENC_B_PIN);
    stat_pwm = DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_PWM_PORT,
                                                 GPIO_MT6816_PIN_ENC_PWM_PIN);
    stat_z = DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_Z_PORT,
                                               GPIO_MT6816_PIN_ENC_Z_PIN);

    a_high = DL_GPIO_readPins(GPIO_MT6816_PIN_ENC_A_PORT,
                              GPIO_MT6816_PIN_ENC_A_PIN) != 0U;
    b_high = DL_GPIO_readPins(GPIO_MT6816_PIN_ENC_B_PORT,
                              GPIO_MT6816_PIN_ENC_B_PIN) != 0U;
    pwm_high = DL_GPIO_readPins(GPIO_MT6816_PIN_ENC_PWM_PORT,
                                GPIO_MT6816_PIN_ENC_PWM_PIN) != 0U;

    /*
     * ── A 相双边沿 → 4X 正交解码 ──
     * A↑ B=H → CW(+), A↑ B=L → CCW(-); A↓ B=H → CCW(-), A↓ B=L → CW(+)
     * B↑ A=H → CCW(-), B↑ A=L → CW(+); B↓ A=H → CW(+), B↓ A=L → CCW(-)
     */
    if (stat_a != 0U) {
        if (a_high) {
            g_enc_count += b_high ? 1 : -1;
        } else {
            g_enc_count += b_high ? -1 : 1;
        }
    }

    if (stat_b != 0U) {
        if (b_high) {
            g_enc_count += a_high ? -1 : 1;
        } else {
            g_enc_count += a_high ? 1 : -1;
        }
    }

    /* ── PWM 双边沿 → 占空比 ── */
    if (stat_pwm != 0U) {
        if (pwm_high) {
            uint16_t period = ts_diff(ts, g_pwm_rise_ts);
            if (period > 100U) {
                g_pwm_period_ticks = period;
            }
            g_pwm_rise_ts = ts;
        } else {
            g_pwm_high_ticks = ts_diff(ts, g_pwm_rise_ts);
            g_pwm_fresh      = true;
        }
    }

    /* ── Z 相上升沿 → 索引 ── */
    if (stat_z != 0U) {
        g_z_detected = true;
        g_enc_at_z   = g_enc_count;
    }

    /* 清除已处理标志 (分端口) */
    if (stat_a != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_MT6816_PIN_ENC_A_PORT,
                                     GPIO_MT6816_PIN_ENC_A_PIN);
    }
    if (stat_b != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_MT6816_PIN_ENC_B_PORT,
                                     GPIO_MT6816_PIN_ENC_B_PIN);
    }
    if (stat_pwm != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_MT6816_PIN_ENC_PWM_PORT,
                                     GPIO_MT6816_PIN_ENC_PWM_PIN);
    }
    if (stat_z != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_MT6816_PIN_ENC_Z_PORT,
                                     GPIO_MT6816_PIN_ENC_Z_PIN);
    }
}

/* ===================================================================== *
 *  公开 API
 * ===================================================================== */

void MT6816_Init(void)
{
    /* ── 1. TIMG0: 自由运行时间戳定时器 (@32MHz, 31.25ns 分辨) ── */
    DL_TimerG_reset(ENC_TS_TIMER);
    DL_TimerG_enablePower(ENC_TS_TIMER);

    {
        DL_TimerG_ClockConfig clk = {
            .clockSel    = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
            .prescale    = 0U
        };
        DL_TimerG_setClockConfig(ENC_TS_TIMER, &clk);
    }

    {
        DL_TimerG_TimerConfig tmr = {
            .timerMode    = DL_TIMER_TIMER_MODE_PERIODIC_UP,
            .period       = ENC_TS_MAX,
            .startTimer   = DL_TIMER_START,
            .genIntermInt = DL_TIMER_INTERM_INT_DISABLED,
            .counterVal   = 0U
        };
        DL_TimerG_initTimerMode(ENC_TS_TIMER, &tmr);
    }

    DL_TimerG_enableClock(ENC_TS_TIMER);

    /* This driver must not depend on Encoder_Init() to enable its IRQ. */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /* ── 2. GPIO 配置 (已由 SYSCFG_DL_GPIO_init() 完成) ──
     *     SysConfig GPIO_MT6816 模块生成:
     *       - 方向 INPUT + 内部电阻 NONE
     *       - Polarity: A/B/PWM=RISE_FALL, Z=RISE
     *       - 中断使能 (DL_GPIO_enableInterrupt)
     *       - 中断标志清除
     *     此处仅使能 NVIC 中断线. */

    /* NVIC: GPIOA 和 GPIOB 共享 GROUP1_INT_IRQn,
     * encoder.c 已调用 NVIC_EnableIRQ(GPIOB_INT_IRQn),
     * 同一条 NVIC 线无需重复调用. */

    /* ── 4. 状态 ── */
    g_enc_count       = 0;
    g_z_detected      = false;
    g_enc_at_z        = 0;
    g_pwm_rise_ts     = 0;
    g_pwm_high_ticks  = 0;
    g_pwm_period_ticks= 0xFFFF;
    g_pwm_fresh       = false;
    g_angle           = 0.0f;
    g_velocity        = 0.0f;
    g_pwm_raw         = 0.0f;
    g_enc_prev        = 0;
    g_delta_prev      = 0;
    g_ready           = true;
}

void MT6816_Update(void)
{
    int32_t  count, delta;
    float    pwm_deg;
    bool     pwm_ok;
    uint16_t high, period;

    if (!g_ready) return;

    /* ── 编码器快照 ── */
    count = (int32_t)g_enc_count;

    /* ── PWM 绝对角度 (原子性读取) ── */
    high   = g_pwm_high_ticks;
    period = g_pwm_period_ticks;

    pwm_ok = (g_pwm_fresh && period > 100U && high <= period);
    if (pwm_ok) {
        pwm_deg = (float)high / (float)period * 360.0f;
        if (pwm_deg > 360.0f) pwm_deg = 360.0f;
        g_pwm_raw    = pwm_deg;
        g_pwm_fresh  = false;
    } else {
        pwm_deg = g_pwm_raw;
    }

    /* ── 角度融合 (PWM 绝对角 EMA + 编码器增量跟踪) ── */
    if (pwm_ok || period > 100U) {
        g_angle += 0.30f * (pwm_deg - g_angle);
        if (g_angle < 0.0f)      g_angle += 360.0f;
        if (g_angle >= 360.0f)   g_angle -= 360.0f;
    }

    /* ── 角速度 (10ms 周期差分 + EMA) ── */
    delta = count - g_enc_prev;
    g_enc_prev = count;

    {
        float raw = (float)delta * (360.0f / (float)MT6816_CPR_4X) * 100.0f;
        float a   = ((g_delta_prev == 0 && delta == 0) ? 0.80f : 0.50f);
        g_velocity += a * (raw - g_velocity);
        g_delta_prev = delta;
    }
}

int32_t MT6816_GetRawCount(void)
{
    return (int32_t)g_enc_count;
}

float MT6816_GetAngleDeg(void)
{
    return g_angle;
}

float MT6816_GetVelocityDegS(void)
{
    return g_velocity;
}

float MT6816_GetPWMAngleDeg(void)
{
    return g_pwm_raw;
}

bool MT6816_IsZDetected(void)
{
    return g_z_detected;
}

void MT6816_ResetPosition(void)
{
    /* 在控制循环上下文中调用, 中断竞争不影响功能正确性 */
    g_enc_count   = 0;
    g_enc_at_z    = 0;
    g_z_detected  = false;
    g_enc_prev    = 0;
    g_angle       = 0.0f;
    g_velocity    = 0.0f;
}

int32_t MT6816_GetRevolutions(void)
{
    return (int32_t)g_enc_count / (int32_t)MT6816_CPR_4X;
}
