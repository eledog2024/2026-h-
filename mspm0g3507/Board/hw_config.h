/* 板级外设别名。 */
#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "ti_msp_dl_config.h"

#define PWM_L_PORT                     GPIO_PWM_MOTOR_C0_PORT   
#define PWM_L_PIN                      GPIO_PWM_MOTOR_C0_PIN  
#define PWM_L_IOMUX                    GPIO_PWM_MOTOR_C0_IOMUX 
#define PWM_L_IOMUX_FUNC               GPIO_PWM_MOTOR_C0_IOMUX_FUNC  
#define PWM_L_CC_INDEX                 GPIO_PWM_MOTOR_C0_IDX  

#define PWM_R_PORT                      GPIO_PWM_MOTOR_C1_PORT  
#define PWM_R_PIN                       GPIO_PWM_MOTOR_C1_PIN  
#define PWM_R_IOMUX                     GPIO_PWM_MOTOR_C1_IOMUX 
#define PWM_R_IOMUX_FUNC               GPIO_PWM_MOTOR_C1_IOMUX_FUNC  
#define PWM_R_CC_INDEX                  GPIO_PWM_MOTOR_C1_IDX  

#define GRAY_DAT_PORT                  GPIO_GRAY_PORT
#define GRAY_DAT_PIN                   GPIO_GRAY_PIN_GRAY_DAT_PIN
#define GRAY_DAT_IOMUX                 GPIO_GRAY_PIN_GRAY_DAT_IOMUX
#define GRAY_CLK_PORT                  GPIO_GRAY_PORT
#define GRAY_CLK_PIN                   GPIO_GRAY_PIN_GRAY_CLK_PIN
#define GRAY_CLK_IOMUX                 GPIO_GRAY_PIN_GRAY_CLK_IOMUX

#define IMU_UART_INST                  UART_IMU_INST
#define IMU_UART_BAUD                  115200U

#define K230_UART_INST                  UART_K230_INST
#define K230_UART_BAUD                  UART_K230_BAUD_RATE

#endif
