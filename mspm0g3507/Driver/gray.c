

/* 八路灰度传感器采样与位置计算。 */
#include "gray.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

static uint8_t g_gray_raw = 0x00;

static int16_t g_last_position = GRAY_POS_CENTER;

#define GRAY_POS_FILTER_ALPHA   CAR_GRAY_FILTER_ALPHA

#define GRAY_CLK_HIGH_US        CAR_GRAY_CLK_HIGH_US

#define GRAY_CLK_LOW_US         CAR_GRAY_CLK_LOW_US

#define GRAY_FRAME_GAP_US       CAR_GRAY_FRAME_GAP_US

static inline void delay_us(uint32_t us)
{
    
    for (uint32_t i = 0; i < (us * 8); i++) {
        __NOP();
    }
}

static uint8_t Gray_ReadSerial(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        
        DL_GPIO_setPins(GRAY_CLK_PORT, GRAY_CLK_PIN);
        delay_us(GRAY_CLK_HIGH_US);

        
        DL_GPIO_clearPins(GRAY_CLK_PORT, GRAY_CLK_PIN);
        delay_us(GRAY_CLK_LOW_US);

        
        if (DL_GPIO_readPins(GRAY_DAT_PORT, GRAY_DAT_PIN) != 0) {
            data |= (uint8_t)(1 << i);
        }
    }

    
    delay_us(GRAY_FRAME_GAP_US);

    return data;
}

/* 初始化灰度传感器。 */
void Gray_Init(void)
{
    
    DL_GPIO_clearPins(GRAY_CLK_PORT, GRAY_CLK_PIN);

    
    g_gray_raw = 0x00;
    g_last_position = GRAY_POS_CENTER;
}

/* 读取八路灰度原始值。 */
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

/* 统计检测到的线路数。 */
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

/* 计算循迹位置。 */
int16_t Gray_GetPosition(void)
{
    uint8_t raw[GRAY_CHANNEL_COUNT];
    Gray_ReadRaw(raw);

    
    uint8_t total_count = Gray_GetLineCount();

    
    uint8_t best_starts[GRAY_CHANNEL_COUNT]; 
    uint8_t best_len = 0;
    uint8_t best_cnt = 0;                    
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

    
    int16_t position = GRAY_POS_CENTER;
    uint16_t weighted_sum = 0;
    uint8_t  count = 0;

    if (total_count == 0) {
        
        position = g_last_position;
    } else if (best_cnt == 1) {
        
        for (uint8_t i = best_starts[0]; i < best_starts[0] + best_len; i++) {
            weighted_sum += (uint16_t)i * 100;
            count++;
        }
    } else {
        
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
                
                if (closest_cnt < GRAY_CHANNEL_COUNT) {
                    closest_starts[closest_cnt] = best_starts[s];
                    closest_cnt++;
                }
            }
        }

        if (closest_cnt == 1) {
            
            uint8_t start = closest_starts[0];
            for (uint8_t i = start; i < start + best_len; i++) {
                weighted_sum += (uint16_t)i * 100;
                count++;
            }
        } else {
            
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
        
        position = (int16_t)((avg / 7.0f) * 160.0f);

        if (position < GRAY_POS_MIN) position = GRAY_POS_MIN;
        if (position > GRAY_POS_MAX) position = GRAY_POS_MAX;
    }

    
    static float filtered_pos = (float)GRAY_POS_CENTER;
    filtered_pos = GRAY_POS_FILTER_ALPHA * (float)position
                 + (1.0f - GRAY_POS_FILTER_ALPHA) * filtered_pos;
    position = (int16_t)filtered_pos;

    
    if (count > 0) {
        g_last_position = position;
    }

    return position;
}

/* 读取灰度位图。 */
uint8_t Gray_GetRawByte(void)
{
    return g_gray_raw;
}
