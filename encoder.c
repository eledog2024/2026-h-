#include "encoder.h"
#include "Board/hw_config.h"
#include "Board/mt6816.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

static volatile int16_t pulse[ENCODER_COUNT];
static int16_t last[ENCODER_COUNT];
static float rpm[ENCODER_COUNT];

/**
 * @brief 根据A相中断读取B相电平, 判断方向并累加脉冲
 * @param id 编码器索引 (ENCODER_L 或 ENCODER_R)
 */

static void count(EncoderIndex_t id)
{
    bool high;

    if (id == ENCODER_L) {
        high = DL_GPIO_readPins(GPIO_HALL_PORT,
                                GPIO_HALL_PIN_HALL_L_B_PIN) != 0;
        pulse[id] += high ? 1 : -1;
    } else {
        /*
         * 右电机和左电机面对面安装, 车子前进时右电机物理旋转方向相反,
         * 导致编码器 A/B 相超前/滞后关系反转. 因此右轮方向取反,
         * 保证 "前进 = 正脉冲" 对两个轮子一致.
         */
        high = DL_GPIO_readPins(GPIO_HALL_PORT,
                               GPIO_HALL_PIN_HALL_R_B_PIN) != 0;
        pulse[id] += high ? -1 : 1;
    }
}

/**
 * @brief GROUP1 中断处理: 分发 GPIOA/GPIOB 中断到编码器
 *
 * MSPM0G3507 的中断架构:
 *   GPIOA_INT → VEC 17 (GROUP1_IRQHandler), IIDX = DL_INTERRUPT_GROUP1_IIDX_GPIOA
 *   GPIOB_INT → VEC 17 (GROUP1_IRQHandler), IIDX = DL_INTERRUPT_GROUP1_IIDX_GPIOB
 *
 * 在 GROUP1_IRQHandler 中根据 IIDX 区分是哪个 GPIO 端口触发,
 * 再检查具体引脚的中断状态来确认是哪个编码器的A相脉冲.
 */
void GROUP1_IRQHandler(void)
{
    /* ── GPIOB: 霍尔编码器 (左/右轮 A 相) ── */
    if (DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_L_A_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_L_A_PIN);
        count(ENCODER_L);
    }

    if (DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_R_A_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_R_A_PIN);
        count(ENCODER_R);
    }

    /* ── GPIOA + GPIOB: MT6816 编码器 (A/B/PWM/Z 跨双端口) ── */
    if (DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_A_PORT,
            GPIO_MT6816_PIN_ENC_A_PIN) ||
        DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_B_PORT,
            GPIO_MT6816_PIN_ENC_B_PIN) ||
        DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_PWM_PORT,
            GPIO_MT6816_PIN_ENC_PWM_PIN) ||
        DL_GPIO_getEnabledInterruptStatus(GPIO_MT6816_PIN_ENC_Z_PORT,
            GPIO_MT6816_PIN_ENC_Z_PIN)) {
        MT6816_IRQHandler();
    }
}

void Encoder_Init(void)
{
    uint8_t i;

    for (i = 0; i < ENCODER_COUNT; i++) {
        pulse[i] = 0;
        last[i] = 0;
        rpm[i] = 0.0f;
    }

    /*
     * 使能 NVIC 中断线 — DL_GPIO_enableInterrupt() 只设了 GPIO 外设的
     * IMASK, 不会自动开 NVIC. 不加这行 GROUP1_IRQHandler 永远不会被调用,
     * 编码器脉冲计数始终为 0.
     */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Encoder_UpdateSpeed(uint32_t ms)
{
    uint8_t i;
    float k;
//rpm:转 / 每分钟
    if (ms == 0U) return;
    k = 60000.0f / ((float)ENCODER_PPR_OUTPUT * (float)ms);
//ms：采样周期（两次调用该函数间隔多少毫秒）
    for (i = 0; i < ENCODER_COUNT; i++) {
        int16_t d = pulse[i] - last[i];
        float raw = (float)d * k;

        last[i] = pulse[i];
        rpm[i] += ENCODER_SPEED_FILTER_ALPHA * (raw - rpm[i]);
    }
}

float Encoder_GetRPM(EncoderIndex_t id)
{
    return id < ENCODER_COUNT ? rpm[id] : 0.0f;
}

int16_t Encoder_GetPulse(EncoderIndex_t id)
{
    return id < ENCODER_COUNT ? pulse[id] : 0;
}
