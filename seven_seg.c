/**
 * @file    seven_seg.c
 * @brief   TM1637 4 位数码管驱动
 *
 * TM1637 协议 (类 I2C, 但不同):
 *   Start: CLK=H, DIO=H → DIO=L
 *   Stop:  CLK=H, DIO=L → DIO=H
 *   Bit:   CLK=L, 设 DIO → CLK=H → 锁存
 *   ACK:   第 9 个 CLK 时 DIO 由 TM1637 拉低
 */

#include "seven_seg.h"
#include "ti_msp_dl_config.h"

/* ===================================================================== *
 *  引脚 (请根据实际接线修改)
 * ===================================================================== */

#define SEG_CLK_PORT    GPIOB
#define SEG_CLK_PIN     DL_GPIO_PIN_20   /* PB20 */
#define SEG_DIO_PORT    GPIOA
#define SEG_DIO_PIN     DL_GPIO_PIN_28   /* PA28 */

#define CLK_H()  DL_GPIO_setPins(SEG_CLK_PORT, SEG_CLK_PIN)
#define CLK_L()  DL_GPIO_clearPins(SEG_CLK_PORT, SEG_CLK_PIN)
#define DIO_H()  DL_GPIO_setPins(SEG_DIO_PORT, SEG_DIO_PIN)
#define DIO_L()  DL_GPIO_clearPins(SEG_DIO_PORT, SEG_DIO_PIN)
#define DIO_RD() DL_GPIO_readPins(SEG_DIO_PORT, SEG_DIO_PIN)

/* TM1637 命令 */
#define TM1637_DATA_CMD   0x40   /* 写数据到显示寄存器, 自动地址递增 */
#define TM1637_DISP_CMD   0x80   /* 显示控制 (或上亮度 0~7) */
#define TM1637_ADDR_FIXED 0x44   /* 写数据, 固定地址 */
#define TM1637_ADDR_C0    0xC0   /* 起始地址 C0H */

/* ===================================================================== *
 *  段码表 (共阳, TM1637 标准)
 *
 *    --A--
 *   |     |
 *   F     B
 *   |     |
 *    --G--
 *   |     |
 *   E     C
 *   |     |
 *    --D--   .DP
 *
 *   bit: DP G F E D C B A
 * ===================================================================== */

static const uint8_t seg_table[] = {
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
    0x77, /* A */
    0x7C, /* b */
    0x39, /* C */
    0x5E, /* d */
    0x79, /* E */
    0x71, /* F */
    0x00, /* 空 (16) */
};

#define SEG_BLANK   16
#define SEG_DASH    0x40   /* '-' = G 段 */

static uint8_t g_brightness = 2;

/* ===================================================================== *
 *  底层时序
 * ===================================================================== */

static void tm1637_delay(void)
{
    for (volatile uint8_t i = 0; i < 20; i++) { __NOP(); }
}

static void tm1637_start(void)
{
    CLK_H();
    DIO_H();
    tm1637_delay();
    DIO_L();
    tm1637_delay();
    CLK_L();
}

static void tm1637_stop(void)
{
    CLK_L();
    tm1637_delay();
    DIO_L();
    tm1637_delay();
    CLK_H();
    tm1637_delay();
    DIO_H();
    tm1637_delay();
}

static void tm1637_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        CLK_L();
        tm1637_delay();
        if (data & 0x01) DIO_H(); else DIO_L();
        tm1637_delay();
        CLK_H();
        tm1637_delay();
        data >>= 1;
    }
    /* ACK (第 9 个时钟) */
    CLK_L();
    tm1637_delay();
    DIO_H();                    /* 释放总线, 让 TM1637 拉低 */
    tm1637_delay();
    CLK_H();
    tm1637_delay();
    /* 读 ACK (忽略, TM1637 一定会 ACK) */
    CLK_L();
    tm1637_delay();
}

/* ===================================================================== *
 *  API
 * ===================================================================== */

void SEG_Init(void)
{
    /* 引脚初始化为推挽输出 */
    DL_GPIO_initDigitalOutput(GPIO_SEG_PIN_SEG_CLK_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_SEG_PIN_SEG_DIO_IOMUX);
    CLK_H();
    DIO_H();

    /* 配置 TM1637: 自动地址递增 + 写数据 */
    tm1637_start();
    tm1637_write_byte(TM1637_DATA_CMD);
    tm1637_stop();

    /* 亮度 + 开启显示 */
    tm1637_start();
    tm1637_write_byte(TM1637_DISP_CMD | (g_brightness & 0x07) | 0x08);
    tm1637_stop();
}

/**
 * @brief 发送 4 字节段码到 TM1637
 */
static void seg_send_raw(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    /*
     * 注意: C0H → 最左位, C3H → 最右位 (模块 GRID 排列不同)
     * 调用方: d3=千位, d2=百位, d1=十位, d0=个位
     * 发送顺序 = 从右到左: d0(C3H) → d1(C2H) → d2(C1H) → d3(C0H)
     */
    tm1637_start();
    tm1637_write_byte(TM1637_ADDR_C0);
    tm1637_write_byte(d3);  /* 千位 → C0H (最左) */
    tm1637_write_byte(d2);  /* 百位 → C1H */
    tm1637_write_byte(d1);  /* 十位 → C2H */
    tm1637_write_byte(d0);  /* 个位 → C3H (最右) */
    tm1637_stop();
}

void SEG_DisplayNum(uint16_t num)
{
    if (num > 9999) num = 9999;

    uint8_t d3 = (num / 1000) % 10;
    uint8_t d2 = (num / 100)  % 10;
    uint8_t d1 = (num / 10)   % 10;
    uint8_t d0 = num % 10;

    seg_send_raw(seg_table[d0],
                 seg_table[d1],
                 seg_table[d2],
                 seg_table[d3]);
}

void SEG_DisplayTime(uint16_t centiseconds)
{
    if (centiseconds > 9999) centiseconds = 9999;

    uint8_t sec = centiseconds / 100;
    uint8_t cs  = centiseconds % 100;

    uint8_t d3 = seg_table[sec / 10];
    uint8_t d2 = seg_table[sec % 10];
    uint8_t d1 = seg_table[cs / 10];
    uint8_t d0 = seg_table[cs % 10];

    /* 第二位 (分钟个位) 加小数点 = 冒号效果 (实际是第2位 DP) */
    d2 |= 0x80;  /* 点亮中间冒号 */

    seg_send_raw(d0, d1, d2, d3);
}

void SEG_DisplayNumColon(uint16_t num)
{
    if (num > 9999) num = 9999;

    uint8_t d3 = seg_table[(num / 1000) % 10];
    uint8_t d2 = seg_table[(num / 100)  % 10];
    uint8_t d1 = seg_table[(num / 10)   % 10];
    uint8_t d0 = seg_table[num % 10];

    d2 |= 0x80;  /* 冒号 */

    seg_send_raw(d0, d1, d2, d3);
}

void SEG_DisplayOff(void)
{
    tm1637_start();
    tm1637_write_byte(TM1637_DISP_CMD | (g_brightness & 0x07));  /* bit3=0 → 关 */
    tm1637_stop();
}

void SEG_DisplaySigned(int16_t num)
{
    bool neg = (num < 0);
    uint16_t abs_val;

    if (neg) {
        if (num < -999) num = -999;
        abs_val = (uint16_t)(-num);
    } else {
        if (num > 9999) num = 9999;
        abs_val = (uint16_t)num;
    }

    uint8_t d0 = seg_table[abs_val % 10];          /* 个位 */
    uint8_t d1 = seg_table[(abs_val / 10) % 10];   /* 十位 */
    uint8_t d2 = seg_table[(abs_val / 100) % 10];  /* 百位 */
    uint8_t d3 = seg_table[(abs_val / 1000) % 10]; /* 千位 */

    if (neg) {
        /* 负号放在最高有效数字左边 */
        if (abs_val >= 100) {
            d3 = SEG_DASH;                          /* -123 */
        } else if (abs_val >= 10) {
            d3 = seg_table[SEG_BLANK];
            d2 = SEG_DASH;                          /*  -12 */
        } else {
            d3 = seg_table[SEG_BLANK];
            d2 = seg_table[SEG_BLANK];
            d1 = SEG_DASH;                          /*   -1 */
        }
    } else {
        /* 正数: 消隐前导零 */
        if (abs_val < 1000) d3 = seg_table[SEG_BLANK];
        if (abs_val < 100)  d2 = seg_table[SEG_BLANK];
        if (abs_val < 10)   d1 = seg_table[SEG_BLANK];
    }

    seg_send_raw(d0, d1, d2, d3);
}

void SEG_SetBrightness(uint8_t level)
{
    if (level > 7) level = 7;
    g_brightness = level;

    tm1637_start();
    tm1637_write_byte(TM1637_DISP_CMD | (g_brightness & 0x07) | 0x08);
    tm1637_stop();
}
