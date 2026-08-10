#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "ti_msp_dl_config.h"

/* 左轮 PWM： (TIMA0_CC0) */
#define PWM_L_PORT                     GPIO_PWM_MOTOR_C0_PORT   
#define PWM_L_PIN                      GPIO_PWM_MOTOR_C0_PIN  
#define PWM_L_IOMUX                    GPIO_PWM_MOTOR_C0_IOMUX 
#define PWM_L_IOMUX_FUNC               GPIO_PWM_MOTOR_C0_IOMUX_FUNC  
#define PWM_L_CC_INDEX                 GPIO_PWM_MOTOR_C0_IDX  

/* 右轮 PWM： (TIMA0_CC1) */
#define PWM_R_PORT                      GPIO_PWM_MOTOR_C1_PORT  
#define PWM_R_PIN                       GPIO_PWM_MOTOR_C1_PIN  
#define PWM_R_IOMUX                     GPIO_PWM_MOTOR_C1_IOMUX 
#define PWM_R_IOMUX_FUNC               GPIO_PWM_MOTOR_C1_IOMUX_FUNC  
#define PWM_R_CC_INDEX                  GPIO_PWM_MOTOR_C1_IDX  

/* 灰度传感器：SysConfig 中的 PA12/PA13 */
#define GRAY_DAT_PORT                  GPIO_GRAY_PORT
#define GRAY_DAT_PIN                   GPIO_GRAY_PIN_GRAY_DAT_PIN
#define GRAY_DAT_IOMUX                 GPIO_GRAY_PIN_GRAY_DAT_IOMUX
#define GRAY_CLK_PORT                  GPIO_GRAY_PORT
#define GRAY_CLK_PIN                   GPIO_GRAY_PIN_GRAY_CLK_PIN
#define GRAY_CLK_IOMUX                 GPIO_GRAY_PIN_GRAY_CLK_IOMUX


/* ===================================================================== *
 *  IMU HWT906 UART (引脚 PB2/PB3 = UART3, 115200, 见 empty.syscfg)
 * ===================================================================== */
#define IMU_UART_INST                  UART_IMU_INST
#define IMU_UART_BAUD                  115200U

/* ===================================================================== *
 *  K230 通信 UART (引脚 PA9/PA8 = UART1, 115200, 见 empty.syscfg)
 * ===================================================================== */
#define K230_UART_INST                  UART_K230_INST
#define K230_UART_BAUD                  UART_K230_BAUD_RATE

/* ===================================================================== *
 *  MT6816 磁编码器 (引脚 PA4/PA5/PA6/PA7, 见 empty.syscfg)
 *
 *  PA4 → ENC_A   (正交 A 相, 双边沿中断, 4X 解码)
 *  PA5 → ENC_B   (正交 B 相, 双边沿中断, 4X 解码)
 *  PA6 → ENC_PWM (PWM 绝对角,  双边沿中断, 占空比→角度)
 *  PA7 → ENC_Z   (索引 Z 相,   上升沿中断, 每圈校准)
 * ===================================================================== */
#endif
