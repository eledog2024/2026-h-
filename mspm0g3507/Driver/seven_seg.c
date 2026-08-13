

/* TM1637 数码管驱动。 */
#include "seven_seg.h"
#include "ti_msp_dl_config.h"

#define SEG_CLK_PORT    GPIOB
#define SEG_CLK_PIN     DL_GPIO_PIN_20   
#define SEG_DIO_PORT    GPIOA
#define SEG_DIO_PIN     DL_GPIO_PIN_28   

#define CLK_H()  DL_GPIO_setPins(SEG_CLK_PORT, SEG_CLK_PIN)
#define CLK_L()  DL_GPIO_clearPins(SEG_CLK_PORT, SEG_CLK_PIN)
#define DIO_H()  DL_GPIO_setPins(SEG_DIO_PORT, SEG_DIO_PIN)
#define DIO_L()  DL_GPIO_clearPins(SEG_DIO_PORT, SEG_DIO_PIN)
#define DIO_RD() DL_GPIO_readPins(SEG_DIO_PORT, SEG_DIO_PIN)

#define TM1637_DATA_CMD   0x40   
#define TM1637_DISP_CMD   0x80   
#define TM1637_ADDR_FIXED 0x44   
#define TM1637_ADDR_C0    0xC0   

static const uint8_t seg_table[] = {
    0x3F, 
    0x06, 
    0x5B, 
    0x4F, 
    0x66, 
    0x6D, 
    0x7D, 
    0x07, 
    0x7F, 
    0x6F, 
    0x77, 
    0x7C, 
    0x39, 
    0x5E, 
    0x79, 
    0x71, 
    0x00, 
};

#define SEG_BLANK   16
#define SEG_DASH    0x40   

static uint8_t g_brightness = 2;

/* 提供 TM1637 时序延时。 */
static void tm1637_delay(void)
{
    for (volatile uint8_t i = 0; i < 20; i++) { __NOP(); }
}

/* 发送 TM1637 起始信号。 */
static void tm1637_start(void)
{
    CLK_H();
    DIO_H();
    tm1637_delay();
    DIO_L();
    tm1637_delay();
    CLK_L();
}

/* 发送 TM1637 停止信号。 */
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

/* 发送一个显示字节。 */
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
    
    CLK_L();
    tm1637_delay();
    DIO_H();                    
    tm1637_delay();
    CLK_H();
    tm1637_delay();
    
    CLK_L();
    tm1637_delay();
}

/* 初始化数码管。 */
void SEG_Init(void)
{
    
    DL_GPIO_initDigitalOutput(GPIO_SEG_PIN_SEG_CLK_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_SEG_PIN_SEG_DIO_IOMUX);
    CLK_H();
    DIO_H();

    
    tm1637_start();
    tm1637_write_byte(TM1637_DATA_CMD);
    tm1637_stop();

    
    tm1637_start();
    tm1637_write_byte(TM1637_DISP_CMD | (g_brightness & 0x07) | 0x08);
    tm1637_stop();
}

/* 发送四位显示数据。 */
static void seg_send_raw(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    
    tm1637_start();
    tm1637_write_byte(TM1637_ADDR_C0);
    tm1637_write_byte(d3);  
    tm1637_write_byte(d2);  
    tm1637_write_byte(d1);  
    tm1637_write_byte(d0);  
    tm1637_stop();
}

/* 显示四位无符号数。 */
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

/* 显示计时结果。 */
void SEG_DisplayTime(uint16_t centiseconds)
{
    if (centiseconds > 9999) centiseconds = 9999;

    uint8_t sec = centiseconds / 100;
    uint8_t cs  = centiseconds % 100;

    uint8_t d3 = seg_table[sec / 10];
    uint8_t d2 = seg_table[sec % 10];
    uint8_t d1 = seg_table[cs / 10];
    uint8_t d0 = seg_table[cs % 10];

    
    d2 |= 0x80;  

    seg_send_raw(d0, d1, d2, d3);
}

/* 显示带冒号数字。 */
void SEG_DisplayNumColon(uint16_t num)
{
    if (num > 9999) num = 9999;

    uint8_t d3 = seg_table[(num / 1000) % 10];
    uint8_t d2 = seg_table[(num / 100)  % 10];
    uint8_t d1 = seg_table[(num / 10)   % 10];
    uint8_t d0 = seg_table[num % 10];

    d2 |= 0x80;  

    seg_send_raw(d0, d1, d2, d3);
}

/* 关闭数码管显示。 */
void SEG_DisplayOff(void)
{
    tm1637_start();
    tm1637_write_byte(TM1637_DISP_CMD | (g_brightness & 0x07));  
    tm1637_stop();
}

/* 显示带符号数字。 */
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

    uint8_t d0 = seg_table[abs_val % 10];          
    uint8_t d1 = seg_table[(abs_val / 10) % 10];   
    uint8_t d2 = seg_table[(abs_val / 100) % 10];  
    uint8_t d3 = seg_table[(abs_val / 1000) % 10]; 

    if (neg) {
        
        if (abs_val >= 100) {
            d3 = SEG_DASH;                          
        } else if (abs_val >= 10) {
            d3 = seg_table[SEG_BLANK];
            d2 = SEG_DASH;                          
        } else {
            d3 = seg_table[SEG_BLANK];
            d2 = seg_table[SEG_BLANK];
            d1 = SEG_DASH;                          
        }
    } else {
        
        if (abs_val < 1000) d3 = seg_table[SEG_BLANK];
        if (abs_val < 100)  d2 = seg_table[SEG_BLANK];
        if (abs_val < 10)   d1 = seg_table[SEG_BLANK];
    }

    seg_send_raw(d0, d1, d2, d3);
}

/* 设置数码管亮度。 */
void SEG_SetBrightness(uint8_t level)
{
    if (level > 7) level = 7;
    g_brightness = level;

    tm1637_start();
    tm1637_write_byte(TM1637_DISP_CMD | (g_brightness & 0x07) | 0x08);
    tm1637_stop();
}
