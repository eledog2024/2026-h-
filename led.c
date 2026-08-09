/**
 * @file    led.c
 * @brief   板载 LED 控制 —— 红灯、绿灯、Board 蓝灯
 *
 * 硬件连接 (MSPM0G3507)：
 *   - RED:   GPIOB.26
 *   - GREEN: GPIOB.27
 *   - BOARD: GPIOB.22
 */

#include "led.h"
#include "ti_msp_dl_config.h"

/* ================================================================ *
 *  红灯
 * ================================================================ */

void LED_Red_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_LED_PIN_RED_IOMUX);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

void LED_Red_On(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

void LED_Red_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

void LED_Red_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_RED_PIN);
}

/* ================================================================ *
 *  绿灯
 * ================================================================ */

void LED_Green_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_LED_PIN_GREEN_IOMUX);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

void LED_Green_On(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

void LED_Green_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

void LED_Green_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_GREEN_PIN);
}

/* ================================================================ *
 *  板载蓝灯
 * ================================================================ */

void LED_Board_Init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_LED_PIN_BOARD_IOMUX);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

void LED_Board_On(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

void LED_Board_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

void LED_Board_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_BOARD_PIN);
}

/* ================================================================ *
 *  全部
 * ================================================================ */

void LED_All_Init(void)
{
    LED_Red_Init();
    LED_Green_Init();
    LED_Board_Init();
}

void LED_All_On(void)
{
    LED_Red_On();
    LED_Green_On();
    LED_Board_On();
}

void LED_All_Off(void)
{
    LED_Red_Off();
    LED_Green_Off();
    LED_Board_Off();
}
