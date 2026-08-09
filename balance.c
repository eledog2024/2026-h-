/**
 * @file    balance.c
 * @brief   步进电机 + 曲柄摆杆球平衡 (双环控制 + 编码器反馈)
 *
 * 控制周期 (分频 3:1):
 *   外环: 10ms (100Hz) — K230 视觉 + PD 位置控制
 *   内环: ~3.33ms (300Hz) — MT6816 编码器 + P 角度跟踪
 *
 * 分频实现:
 *   - 默认: StepperCrank_Control() 内部 3x 软件子循环
 *     (每次 10ms 调用: 外环×1 + 内环×3, encoder 在每次内环迭代刷新)
 *   - 可选: 启用 SCR_USE_TIMG1_INNER_LOOP 后, TIMG1 硬件定时器
 *     产生 300Hz 中断, ISR 中调用 StepperCrank_ControlInner()
 *
 * 架构:
 *   K230 → 球位置/速度 (mm, mm/s)
 *   MT6816 → 曲柄绝对角度 (°), 直接对应, 无需换算
 *
 *   外环 PD:  球位置误差 + 速度阻尼 → 目标曲柄角度 (°)
 *   内环 P:   角度误差 → PWM 占空比 (速度)
 *   前馈:     小车加速度 → 补偿速度指令
 *
 * 编码器 PWM 上电即给出绝对角度, 无需回零.
 * 限位开关仅作安全保护.
 */

#include "balance.h"
#include "Board/hw_config.h"
#include "Board/car_config.h"
#include "Board/mt6816.h"
#include "Driver/k230_uart.h"
#include "ti_msp_dl_config.h"

/* ===================================================================== *
 *  底层: 步进电机驱动 (PWM+DIR, TIMA0 CCP0 on PA0)
 *
 *  SysConfig PWM_STEPPER 已生成:
 *    SYSCFG_DL_PWM_STEPPER_init() → TIMA0 @ 20kHz (period=1600)
 * ===================================================================== */

static void stepper_enable(bool en)
{
    if (en) {
        DL_GPIO_clearPins(SCR_EN_PORT, SCR_EN_PIN);   /* LOW = 使能 */
    } else {
        DL_GPIO_setPins(SCR_EN_PORT, SCR_EN_PIN);     /* HIGH = 脱机 */
    }
}

static void stepper_set_dir(int8_t dir)
{
    if (dir > 0) DL_GPIO_setPins(SCR_DIR_PORT, SCR_DIR_PIN);
    else         DL_GPIO_clearPins(SCR_DIR_PORT, SCR_DIR_PIN);
}

/**
 * @brief Start stepper PWM with duty = 0 (motor stopped).
 *
 * SYSCFG_DL_PWM_STEPPER_init() was already called by SYSCFG_DL_init().
 * It configured TIMA0 @ 20kHz (period=1600) but left the timer STOPPED.
 * We reset duty to 0 and start the counter so the PWM output is live.
 */
static void stepper_pwm_init(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_STEPPER_INST, 0,
        GPIO_PWM_STEPPER_C0_IDX);
    DL_TimerA_startCounter(PWM_STEPPER_INST);
}

/**
 * @brief Set stepper speed via PWM duty cycle + direction.
 *        duty 0 = stop, duty SCR_PWM_MAX_DUTY = max speed.
 */
static void stepper_set_pwm_duty(uint16_t duty, int8_t dir)
{
    if (duty > (uint16_t)SCR_PWM_MAX_DUTY) duty = (uint16_t)SCR_PWM_MAX_DUTY;
    DL_TimerA_setCaptureCompareValue(PWM_STEPPER_INST, duty,
        GPIO_PWM_STEPPER_C0_IDX);
    stepper_set_dir(dir);
}

/* ===================================================================== *
 *  系统状态
 * ===================================================================== */

static bool     g_stepper_enabled;
static float    g_angle_zero;          /* 编码器角度偏移 (init 时刻) */

/* ── K230 滤波值 ── */
static float    g_ball_pos_filt;
static float    g_ball_vel_filt;

/* ── 控制状态 ── */
static int16_t  g_target_mm;           /* 目标球偏移 (mm) */
static float    g_target_angle_deg;    /* 外环输出的目标曲柄角 (°) */
static float    g_feedforward_cmd;     /* 前馈速度指令 (外环计算, 内环使用) */

/* ── 编码器反馈 ── */
static float    g_enc_angle_deg;       /* 当前曲柄角 (编码器测量, 相对零位) */
static float    g_enc_velocity_ds;     /* 角速度 (°/s) */

/* ── 内环高频触发 (TIMG1 硬件定时器模式) ── */
#if SCR_USE_TIMG1_INNER_LOOP
volatile bool g_inner_tick;            /* TIMG1 ISR 置位, 外部轮询消费 */
#endif

/* ===================================================================== *
 *  TIMG1 初始化 — 内环 300Hz 定时器 (可选)
 *
 *  当 SCR_USE_TIMG1_INNER_LOOP=1 时, TIMG1 产生 300Hz 中断.
 *  主循环中检查 g_inner_tick 标志, 调用 StepperCrank_ControlInner().
 *
 *  TIMG1 时钟: BUSCLK 32MHz, prescale=0, divide=1
 *  period = 32,000,000 / 300 - 1 ≈ 106,666 - 1 = 106,665
 *  实际频率: 32,000,000 / 106,666 = 300.001Hz, 周期 3.333ms
 * ===================================================================== */

#if SCR_USE_TIMG1_INNER_LOOP
static void inner_timer_init(void)
{
    /* TIMG1 @ 32MHz, 产生 300Hz 周期中断 */
    DL_TimerG_reset(TIMG1);
    DL_TimerG_enablePower(TIMG1);

    {
        DL_TimerG_ClockConfig clk = {
            .clockSel    = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
            .prescale    = 0U
        };
        DL_TimerG_setClockConfig(TIMG1, &clk);
    }

    {
        /*
         * 32,000,000 / 300 ≈ 106,667 ticks
         * period = 106,667 - 1 = 106,666 (0-indexed)
         */
        DL_TimerG_TimerConfig tmr = {
            .timerMode    = DL_TIMER_TIMER_MODE_PERIODIC_UP,
            .period       = (uint32_t)(CPUCLK_FREQ / 300U) - 1U,
            .startTimer   = DL_TIMER_START,
            .genIntermInt = DL_TIMER_INTERM_INT_ENABLED,
            .counterVal   = 0U
        };
        DL_TimerG_initTimerMode(TIMG1, &tmr);
    }

    DL_TimerG_enableClock(TIMG1);
    NVIC_EnableIRQ(TIMG1_INT_IRQn);
}

/** @brief TIMG1 ISR — 内环 300Hz 触发 */
void TIMG1_IRQHandler(void)
{
    g_inner_tick = true;
}
#endif /* SCR_USE_TIMG1_INNER_LOOP */

/* ===================================================================== *
 *  传感器更新: K230 + EMA
 * ===================================================================== */

static void sensor_update(void)
{
    static bool first = true;

    if (first) {
        g_ball_pos_filt = K230_GetBallPos();
        g_ball_vel_filt = K230_GetBallVel();
        first = false;
        return;
    }

    g_ball_pos_filt += K230_EMA_ALPHA
                     * (K230_GetBallPos() - g_ball_pos_filt);
    g_ball_vel_filt  = K230_GetBallVel();
}

/* ===================================================================== *
 *  公开 API
 * ===================================================================== */

void StepperCrank_Init(void)
{
    /* ── 步进电机 ──
     *     SysConfig PWM_STEPPER: PA0=TIMA0 CCP0 (PWM, 20kHz)
     *     SysConfig GPIO_STEPPER: PA1=DIR, PB14=EN(SET=高=脱机)
     *     SYSCFG_DL_init() 已完成所有初始化. */
    DL_GPIO_clearPins(SCR_DIR_PORT, SCR_DIR_PIN);
    stepper_enable(false);
    stepper_pwm_init();

    /* ── MT6816 编码器 ── */
    MT6816_Init();

    /* ── 内环定时器 (可选, TIMG1 @ 300Hz) ── */
#if SCR_USE_TIMG1_INNER_LOOP
    inner_timer_init();
#endif

    /* ── 以当前编码器角度为零位 (上电即知绝对位置) ── */
    MT6816_Update();
    g_angle_zero = MT6816_GetAngleDeg();

    /* ── 状态清零 ── */
    g_target_mm           = 0;
    g_target_angle_deg    = 0.0f;
    g_ball_pos_filt       = 0.0f;
    g_ball_vel_filt       = 0.0f;
    g_enc_angle_deg       = 0.0f;
    g_enc_velocity_ds     = 0.0f;
}

void StepperCrank_SetTarget(int16_t target_mm)
{
    if (target_mm < -125) target_mm = -125;
    if (target_mm >  125) target_mm =  125;
    g_target_mm = target_mm;
}

int16_t StepperCrank_GetTarget(void)
{
    return g_target_mm;
}

/* ===================================================================== *
 *  外环 PD (100Hz): 球位置+速度 → 目标曲柄角度
 *
 *  调用频率: 每 10ms 一次 (SysTick)
 *
 *  内部: K230 解析 → EMA 滤波 → PD 计算 → 写入 g_target_angle_deg
 *       同时计算前馈分量 g_feedforward_cmd (供内环使用).
 *
 *  物理含义: 球偏右 → 摆杆左倾 → 球滚回中心
 *           正值 target_angle = 自由端升高 (球向铰链端滚)
 * ===================================================================== */
void StepperCrank_ControlOuter(float car_accel_mms2)
{
    float pos_err;
    float target_angle;

    /* ── 0. 传感器 ── */
    K230_ParseStream();
    sensor_update();

    /* ── 球丢失检测 ── */
    if (!K230_IsBallDetected()) {
        return;
    }

    /* ── 首次检测到球: 自动使能电机 ── */
    if (!g_stepper_enabled) {
        g_stepper_enabled = true;
        stepper_enable(true);
    }

    /* ── 1. 外环 PD ── */
    pos_err      = (float)g_target_mm - g_ball_pos_filt;
    target_angle = SCR_OUTER_KP * pos_err - SCR_OUTER_KD * g_ball_vel_filt;

    if (target_angle >  SCR_OUTER_ANGLE_MAX) target_angle =  SCR_OUTER_ANGLE_MAX;
    if (target_angle < -SCR_OUTER_ANGLE_MAX) target_angle = -SCR_OUTER_ANGLE_MAX;

    g_target_angle_deg = target_angle;

    /* ── 前馈分量 (常数, 供内环每次迭代使用) ── */
    g_feedforward_cmd = SCR_ACC_FF * car_accel_mms2;
}

/* ===================================================================== *
 *  内环 P (300Hz): 角度误差 → PWM 速度指令
 *
 *  调用频率: 每 ~3.33ms 一次 (TIMG1 硬件定时器 / 软件 3x 子循环)
 *
 *  读取 g_target_angle_deg (外环输出), 编码器反馈,
 *  计算 P 控制 + 前馈, 输出 PWM duty.
 *
 *  编码器角度直接对应曲柄角, 无需运动学换算.
 * ===================================================================== */
void StepperCrank_ControlInner(void)
{
    float actual_angle;
    float angle_err;
    float step_cmd;

    if (!g_stepper_enabled) return;

    /* ── 1. 编码器更新 ── */
    MT6816_Update();
    g_enc_angle_deg   = MT6816_GetAngleDeg() - g_angle_zero;
    g_enc_velocity_ds = MT6816_GetVelocityDegS();

    /* 角度归一化至 [-180, 180] */
    while (g_enc_angle_deg >  180.0f) g_enc_angle_deg -= 360.0f;
    while (g_enc_angle_deg < -180.0f) g_enc_angle_deg += 360.0f;

    actual_angle = g_enc_angle_deg;

    /* ── 2. 内环 P ── */
    angle_err = g_target_angle_deg - actual_angle;
    step_cmd  = SCR_INNER_KP * angle_err;

    /* ── 3. 前馈 (外环计算, 常数) ── */
    step_cmd += g_feedforward_cmd;

    /* ── 4. 限幅 ── */
    if (step_cmd >  SCR_PWM_SPEED_MAX) step_cmd =  SCR_PWM_SPEED_MAX;
    if (step_cmd < -SCR_PWM_SPEED_MAX) step_cmd = -SCR_PWM_SPEED_MAX;

    /* ── 5. step_cmd → PWM duty (TIMA0 CCP0 on PA0) ── */
    {
        float    abs_cmd = (step_cmd > 0.0f) ? step_cmd : -step_cmd;
        float    duty_f  = abs_cmd / SCR_PWM_SPEED_MAX * (float)SCR_PWM_MAX_DUTY;
        uint16_t duty    = (uint16_t)duty_f;
        int8_t   dir     = (step_cmd >= 0.0f) ? 1 : -1;
        stepper_set_pwm_duty(duty, dir);
    }

    /* ── 6. Z 相校准 (每圈自动归零, 消除累积漂移) ── */
    if (MT6816_IsZDetected()) {
        g_angle_zero = MT6816_GetAngleDeg();
    }
}

/* ===================================================================== *
 *  双环控制 (兼容接口): 外环 1× + 内环 SCR_INNER_OUTER_RATIO ×
 *
 *  调用频率: 每 10ms 一次 (SysTick)
 *
 *  内部流程:
 *    1. 外环 PD (100Hz): 传感器 → 球位置/速度 → 目标角度
 *    2. 内环 P  (300Hz): 编码器 → 角度 → PWM, 执行 3 次
 *       (每次迭代刷新编码器, 等效 3× 角度跟踪带宽)
 *
 *  @note 若已启用 TIMG1 硬件内环 (SCR_USE_TIMG1_INNER_LOOP=1),
 *        则只执行外环; 内环由 TIMG1 ISR 驱动, 不在此函数内执行.
 *        此时调用者还需在主循环中轮询 g_inner_tick 标志:
 *          if (g_inner_tick) { g_inner_tick = false; StepperCrank_ControlInner(); }
 * ===================================================================== */
void StepperCrank_Control(float car_accel_mms2)
{
    uint32_t i;

    /* ── 外环 (100Hz) ── */
    StepperCrank_ControlOuter(car_accel_mms2);

#if SCR_USE_TIMG1_INNER_LOOP
    /* 硬件定时器模式: 内环在 TIMG1 ISR 或主循环轮询中执行.
     * 此处处理可能积压的内环 tick (最多 SCR_INNER_OUTER_RATIO 次). */
    for (i = 0; i < SCR_INNER_OUTER_RATIO; i++) {
        if (g_inner_tick) {
            g_inner_tick = false;
            StepperCrank_ControlInner();
        }
    }
#else
    /* 软件模式: 内环 SCR_INNER_OUTER_RATIO×, 逐次刷新编码器 */
    for (i = 0; i < SCR_INNER_OUTER_RATIO; i++) {
        StepperCrank_ControlInner();
    }
#endif
}

void StepperCrank_Stop(void)
{
    stepper_set_pwm_duty(0, 0);
    stepper_enable(false);
    g_stepper_enabled  = false;
    g_feedforward_cmd  = 0.0f;
}

bool StepperCrank_AtTarget(uint16_t tolerance_mm)
{
    float err = (float)g_target_mm - g_ball_pos_filt;
    if (err < 0.0f) err = -err;
    return (err <= (float)tolerance_mm);
}

/* ===================================================================== *
 *  Task 3: ±5cm 往复球平衡状态机
 *
 *  复用双环控制. 小车静止 (car_accel=0).
 *  状态机仅负责目标切换 + 保持计时 + 完成判定.
 * ===================================================================== */

typedef enum {
    T3_GO_PLUS5,
    T3_HOLD_PLUS5,
    T3_GO_MINUS5,
    T3_HOLD_MINUS5,
    T3_DONE,
} T3State_t;

static T3State_t g_t3_state      = T3_GO_PLUS5;
static uint16_t  g_t3_hold_ticks;

void StepperCrank_Task3Reset(void)
{
    g_t3_state      = T3_GO_PLUS5;
    g_t3_hold_ticks = 0;
}

bool StepperCrank_Task3Control(void)
{
    float  target;
    float  error;

    /* ── 设定目标 ── */
    switch (g_t3_state) {
    case T3_GO_PLUS5:
    case T3_HOLD_PLUS5:
        target = 50.0f;  break;
    case T3_GO_MINUS5:
    case T3_HOLD_MINUS5:
        target = -50.0f; break;
    case T3_DONE:
        StepperCrank_Stop();
        return true;
    default:
        target = 0.0f;   break;
    }
    StepperCrank_SetTarget((int16_t)target);

    /* ── 使能并执行控制 (car_accel=0, 静止) ── */
    stepper_enable(true);
    g_stepper_enabled = true;
    StepperCrank_Control(0.0f);

    if (!K230_IsBallDetected()) {
        stepper_enable(false);
        g_stepper_enabled = false;
        return false;
    }

    /* ── 状态转移 ── */
    switch (g_t3_state) {

    case T3_GO_PLUS5:
        error = g_ball_pos_filt - 50.0f;
        if (error < 0.0f) error = -error;
        if (error <= SCR_T3_TOLERANCE) {
            g_t3_state      = T3_HOLD_PLUS5;
            g_t3_hold_ticks = 0;
        }
        break;

    case T3_HOLD_PLUS5:
        g_t3_hold_ticks++;
        if (g_t3_hold_ticks >= (SCR_T3_HOLD_PLUS5_MS / 10U)) {
            g_t3_state      = T3_GO_MINUS5;
            g_t3_hold_ticks = 0;
        }
        break;

    case T3_GO_MINUS5:
        error = g_ball_pos_filt - (-50.0f);
        if (error < 0.0f) error = -error;
        if (error <= SCR_T3_TOLERANCE) {
            g_t3_state      = T3_HOLD_MINUS5;
            g_t3_hold_ticks = 0;
        }
        break;

    case T3_HOLD_MINUS5:
        g_t3_hold_ticks++;
        error = g_ball_pos_filt - (-50.0f);
        if (error < 0.0f) error = -error;
        if (g_t3_hold_ticks >= (SCR_T3_HOLD_MINUS5_MS / 10U)
            && error <= SCR_T3_TOLERANCE) {
            g_t3_state = T3_DONE;
        }
        break;

    default: break;
    }

    return false;
}

/* ===================================================================== *
 *  调试接口
 * ===================================================================== */

float   SCR_GetBallPosMm(void)           { return g_ball_pos_filt; }
float   SCR_GetBallVelMms(void)          { return g_ball_vel_filt; }
float   SCR_GetEncoderAngleDeg(void)     { return g_enc_angle_deg; }
float   SCR_GetEncoderVelocityDegS(void)  { return g_enc_velocity_ds; }
