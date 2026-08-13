/* 钢球平衡双环控制。 */
#include "balance.h"
#include "../Board/hw_config.h"
#include "../Board/car_config.h"
#include "../Board/mt6816.h"
#include "k230_uart.h"
#include "ti_msp_dl_config.h"

/* 控制步进驱动使能。 */
static void stepper_enable(bool en)
{
    if (en) {
        DL_GPIO_clearPins(SCR_EN_PORT, SCR_EN_PIN);
    } else {
        DL_GPIO_setPins(SCR_EN_PORT, SCR_EN_PIN);
    }
}

/* 设置步进旋转方向。 */
static void stepper_set_dir(int8_t dir)
{
    if (dir > 0) DL_GPIO_setPins(SCR_DIR_PORT, SCR_DIR_PIN);
    else         DL_GPIO_clearPins(SCR_DIR_PORT, SCR_DIR_PIN);
}

/* 启动步进 PWM 输出。 */
static void stepper_pwm_init(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_STEPPER_INST, 0,
        GPIO_PWM_STEPPER_C0_IDX);
    DL_TimerA_startCounter(PWM_STEPPER_INST);
}

/* 设置步进速度和方向。 */
static void stepper_set_pwm_duty(uint16_t duty, int8_t dir)
{
    if (duty > (uint16_t)SCR_PWM_MAX_DUTY) duty = (uint16_t)SCR_PWM_MAX_DUTY;
    DL_TimerA_setCaptureCompareValue(PWM_STEPPER_INST, duty,
        GPIO_PWM_STEPPER_C0_IDX);
    stepper_set_dir(dir);
}

static bool     g_stepper_enabled;
static float    g_angle_zero;
static float    g_ball_pos_filt;
static float    g_ball_vel_filt;
static int16_t  g_target_mm;
static float    g_target_angle_deg;
static float    g_feedforward_cmd;
static float    g_enc_angle_deg;
static float    g_enc_velocity_ds;

#if SCR_USE_TIMG1_INNER_LOOP
volatile bool g_inner_tick;
#endif

#if SCR_USE_TIMG1_INNER_LOOP
/* 初始化内环定时器。 */
static void inner_timer_init(void)
{
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

/* 标记一次内环控制。 */
void TIMG1_IRQHandler(void)
{
    g_inner_tick = true;
}
#endif

/* 更新并滤波球状态。 */
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
    g_ball_vel_filt = K230_GetBallVel();
}

/* 初始化平衡执行机构。 */
void StepperCrank_Init(void)
{
    DL_GPIO_clearPins(SCR_DIR_PORT, SCR_DIR_PIN);
    stepper_enable(false);
    stepper_pwm_init();

    MT6816_Init();

#if SCR_USE_TIMG1_INNER_LOOP
    inner_timer_init();
#endif

    MT6816_Update();
    g_angle_zero = MT6816_GetAngleDeg();

    g_target_mm        = 0;
    g_target_angle_deg = 0.0f;
    g_ball_pos_filt    = 0.0f;
    g_ball_vel_filt    = 0.0f;
    g_enc_angle_deg    = 0.0f;
    g_enc_velocity_ds  = 0.0f;
}

/* 设置并限制球目标位置。 */
void StepperCrank_SetTarget(int16_t target_mm)
{
    if (target_mm < -125) target_mm = -125;
    if (target_mm >  125) target_mm =  125;
    g_target_mm = target_mm;
}

/* 读取球目标位置。 */
int16_t StepperCrank_GetTarget(void)
{
    return g_target_mm;
}

/* 外环计算目标倾角。 */
void StepperCrank_ControlOuter(float car_accel_mms2)
{
    float pos_err;
    float target_angle;

    K230_ParseStream();
    sensor_update();

    if (!K230_IsBallDetected()) {
        return;
    }

    if (!g_stepper_enabled) {
        g_stepper_enabled = true;
        stepper_enable(true);
    }

    pos_err      = (float)g_target_mm - g_ball_pos_filt;
    target_angle = SCR_OUTER_KP * pos_err - SCR_OUTER_KD * g_ball_vel_filt;

    if (target_angle >  SCR_OUTER_ANGLE_MAX)
        target_angle = SCR_OUTER_ANGLE_MAX;
    if (target_angle < -SCR_OUTER_ANGLE_MAX)
        target_angle = -SCR_OUTER_ANGLE_MAX;

    g_target_angle_deg = target_angle;
    g_feedforward_cmd = SCR_ACC_FF * car_accel_mms2;
}

/* 内环跟踪目标倾角。 */
void StepperCrank_ControlInner(void)
{
    float actual_angle;
    float angle_err;
    float step_cmd;

    if (!g_stepper_enabled) return;

    MT6816_Update();
    g_enc_angle_deg   = MT6816_GetAngleDeg() - g_angle_zero;
    g_enc_velocity_ds = MT6816_GetVelocityDegS();

    while (g_enc_angle_deg >  180.0f) g_enc_angle_deg -= 360.0f;
    while (g_enc_angle_deg < -180.0f) g_enc_angle_deg += 360.0f;

    actual_angle = g_enc_angle_deg;

    angle_err = g_target_angle_deg - actual_angle;
    step_cmd  = SCR_INNER_KP * angle_err;

    step_cmd += g_feedforward_cmd;

    if (step_cmd >  SCR_PWM_SPEED_MAX) step_cmd =  SCR_PWM_SPEED_MAX;
    if (step_cmd < -SCR_PWM_SPEED_MAX) step_cmd = -SCR_PWM_SPEED_MAX;

    {
        float    abs_cmd = (step_cmd > 0.0f) ? step_cmd : -step_cmd;
        float    duty_f  = abs_cmd / SCR_PWM_SPEED_MAX * (float)SCR_PWM_MAX_DUTY;
        uint16_t duty    = (uint16_t)duty_f;
        int8_t   dir     = (step_cmd >= 0.0f) ? 1 : -1;
        stepper_set_pwm_duty(duty, dir);
    }

    if (MT6816_IsZDetected()) {
        g_angle_zero = MT6816_GetAngleDeg();
    }
}

/* 执行一次双环控制。 */
void StepperCrank_Control(float car_accel_mms2)
{
    uint32_t i;

    StepperCrank_ControlOuter(car_accel_mms2);

#if SCR_USE_TIMG1_INNER_LOOP
    for (i = 0; i < SCR_INNER_OUTER_RATIO; i++) {
        if (g_inner_tick) {
            g_inner_tick = false;
            StepperCrank_ControlInner();
        }
    }
#else
    for (i = 0; i < SCR_INNER_OUTER_RATIO; i++) {
        StepperCrank_ControlInner();
    }
#endif
}

/* 停止平衡执行机构。 */
void StepperCrank_Stop(void)
{
    stepper_set_pwm_duty(0, 0);
    stepper_enable(false);
    g_stepper_enabled = false;
    g_feedforward_cmd = 0.0f;
}

/* 判断球是否到达目标。 */
bool StepperCrank_AtTarget(uint16_t tolerance_mm)
{
    float err = (float)g_target_mm - g_ball_pos_filt;
    if (err < 0.0f) err = -err;
    return (err <= (float)tolerance_mm);
}

typedef enum {
    T3_GO_PLUS5,
    T3_HOLD_PLUS5,
    T3_GO_MINUS5,
    T3_HOLD_MINUS5,
    T3_DONE,
} T3State_t;

static T3State_t g_t3_state = T3_GO_PLUS5;
static uint16_t  g_t3_hold_ticks;

/* 复位任务 3 状态机。 */
void StepperCrank_Task3Reset(void)
{
    g_t3_state      = T3_GO_PLUS5;
    g_t3_hold_ticks = 0;
}

/* 执行任务 3 往返控制。 */
bool StepperCrank_Task3Control(void)
{
    float target;
    float error;

    switch (g_t3_state) {
    case T3_GO_PLUS5:
    case T3_HOLD_PLUS5:
        target = 50.0f;
        break;

    case T3_GO_MINUS5:
    case T3_HOLD_MINUS5:
        target = -50.0f;
        break;

    case T3_DONE:
        StepperCrank_Stop();
        return true;

    default:
        target = 0.0f;
        break;
    }

    StepperCrank_SetTarget((int16_t)target);

    stepper_enable(true);
    g_stepper_enabled = true;
    StepperCrank_Control(0.0f);

    if (!K230_IsBallDetected()) {
        stepper_enable(false);
        g_stepper_enabled = false;
        return false;
    }

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

    default:
        break;
    }

    return false;
}

/* 读取球位置。 */
float SCR_GetBallPosMm(void)          { return g_ball_pos_filt; }
/* 读取球速度。 */
float SCR_GetBallVelMms(void)         { return g_ball_vel_filt; }
/* 读取编码器角度。 */
float SCR_GetEncoderAngleDeg(void)    { return g_enc_angle_deg; }
/* 读取编码器角速度。 */
float SCR_GetEncoderVelocityDegS(void){ return g_enc_velocity_ds; }
