/* 车轮霍尔编码器采集。 */
#include "encoder.h"
#include "Board/hw_config.h"
#include "Board/mt6816.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

static volatile int16_t pulse[ENCODER_COUNT];
static int16_t last[ENCODER_COUNT];
static float rpm[ENCODER_COUNT];

/* 累计单个轮子的脉冲。 */
static void count(EncoderIndex_t id)
{
    bool high;

    if (id == ENCODER_L) {
        high = DL_GPIO_readPins(GPIO_HALL_PORT,
                                GPIO_HALL_PIN_HALL_L_B_PIN) != 0;
        pulse[id] += high ? 1 : -1;
    } else {
        
        high = DL_GPIO_readPins(GPIO_HALL_PORT,
                               GPIO_HALL_PIN_HALL_R_B_PIN) != 0;
        pulse[id] += high ? -1 : 1;
    }
}

/* 分发编码器 GPIO 中断。 */
void GROUP1_IRQHandler(void)
{
    
    if (DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_L_A_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_L_A_PIN);
        count(ENCODER_L);
    }

    if (DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_R_A_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_HALL_PIN_HALL_R_A_PIN);
        count(ENCODER_R);
    }

    
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

/* 初始化车轮编码器。 */
void Encoder_Init(void)
{
    uint8_t i;

    for (i = 0; i < ENCODER_COUNT; i++) {
        pulse[i] = 0;
        last[i] = 0;
        rpm[i] = 0.0f;
    }

    
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/* 计算并滤波轮速。 */
void Encoder_UpdateSpeed(uint32_t ms)
{
    uint8_t i;
    float k;
    if (ms == 0U) return;
    k = 60000.0f / ((float)ENCODER_PPR_OUTPUT * (float)ms);
    for (i = 0; i < ENCODER_COUNT; i++) {
        int16_t d = pulse[i] - last[i];
        float raw = (float)d * k;

        last[i] = pulse[i];
        rpm[i] += ENCODER_SPEED_FILTER_ALPHA * (raw - rpm[i]);
    }
}

/* 读取指定轮速。 */
float Encoder_GetRPM(EncoderIndex_t id)
{
    return id < ENCODER_COUNT ? rpm[id] : 0.0f;
}

/* 读取指定脉冲数。 */
int16_t Encoder_GetPulse(EncoderIndex_t id)
{
    return id < ENCODER_COUNT ? pulse[id] : 0;
}
