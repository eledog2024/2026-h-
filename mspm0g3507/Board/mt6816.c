

/* MT6816 磁编码器采集与角度融合。 */
#include "mt6816.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

#define ENC_ALL_PINS \
    (GPIO_MT6816_PIN_ENC_A_PIN | GPIO_MT6816_PIN_ENC_B_PIN | \
     GPIO_MT6816_PIN_ENC_PWM_PIN | GPIO_MT6816_PIN_ENC_Z_PIN)

#define ENC_PINS_PORTA \
    (GPIO_MT6816_PIN_ENC_A_PIN | GPIO_MT6816_PIN_ENC_Z_PIN)

#define ENC_PINS_PORTB \
    (GPIO_MT6816_PIN_ENC_B_PIN | GPIO_MT6816_PIN_ENC_PWM_PIN)

#define ENC_TS_TIMER                  TIMG0
#define ENC_TS_MAX                    0xFFFFU

static volatile int32_t  g_enc_count;
static volatile bool     g_z_detected;
static volatile int32_t  g_enc_at_z;

static volatile uint16_t g_pwm_rise_ts;
static volatile uint16_t g_pwm_high_ticks;
static volatile uint16_t g_pwm_period_ticks;
static volatile bool     g_pwm_fresh;

static float   g_angle;
static float   g_velocity;
static float   g_pwm_raw;
static int32_t g_enc_prev;
static int32_t g_delta_prev;
static bool    g_ready;

static inline uint16_t ts_diff(uint16_t now, uint16_t prev)
{
    return (uint16_t)(now - prev);
}

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

    
    if (stat_z != 0U) {
        g_z_detected = true;
        g_enc_at_z   = g_enc_count;
    }

    
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

void MT6816_Init(void)
{
    
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

    
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    

    

    
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

    
    count = (int32_t)g_enc_count;

    
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

    
    if (pwm_ok || period > 100U) {
        g_angle += 0.30f * (pwm_deg - g_angle);
        if (g_angle < 0.0f)      g_angle += 360.0f;
        if (g_angle >= 360.0f)   g_angle -= 360.0f;
    }

    
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
