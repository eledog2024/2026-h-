

/* 状态 LED 控制。 */
#include "led.h"
#include "ti_msp_dl_config.h"

/* 初始化红灯。 */
void LED_Red_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_LED_PIN_RED_IOMUX);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

/* 点亮红灯。 */
void LED_Red_On(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

/* 熄灭红灯。 */
void LED_Red_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

/* 翻转红灯状态。 */
void LED_Red_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

/* 初始化绿灯。 */
void LED_Green_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_LED_PIN_GREEN_IOMUX);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

/* 点亮绿灯。 */
void LED_Green_On(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

/* 熄灭绿灯。 */
void LED_Green_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

/* 翻转绿灯状态。 */
void LED_Green_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

/* 初始化板载灯。 */
void LED_Board_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_LED_PIN_BOARD_IOMUX);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

/* 点亮板载灯。 */
void LED_Board_On(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

/* 熄灭板载灯。 */
void LED_Board_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

/* 翻转板载灯状态。 */
void LED_Board_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

/* 初始化全部指示灯。 */
void LED_All_Init(void)
{
    LED_Red_Init();
    LED_Green_Init();
    LED_Board_Init();
}

/* 点亮全部指示灯。 */
void LED_All_On(void)
{
    LED_Red_On();
    LED_Green_On();
    LED_Board_On();
}

/* 熄灭全部指示灯。 */
void LED_All_Off(void)
{
    LED_Red_Off();
    LED_Green_Off();
    LED_Board_Off();
}
