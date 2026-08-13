

/* 车辆与平衡控制参数。 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

#define CAR_BASE_SPEED                 30.0f    
#define CAR_TURN_SPEED_DIFF_RPM        50.0f    

#define CAR_PID_OUTER_KP               0.15f
#define CAR_PID_OUTER_KI               0.00f
#define CAR_PID_OUTER_KD               0.50f

#define CAR_SPEED_PID_KP               1.0f
#define CAR_SPEED_PID_KI               0.2f
#define CAR_SPEED_PID_KD               0.05f
#define CAR_SPEED_PID_OUTPUT_LIMIT     50.0f
#define CAR_SPEED_OUT_FILTER_ALPHA     0.5f

#define CAR_RPM_TO_PWM_L               0.219f
#define CAR_RPM_TO_PWM_R               0.231f

#define ENCODER_PPR_OUTPUT             ((uint16_t)390)
#define ENCODER_SPEED_FILTER_ALPHA     0.3f

#define CAR_WHEEL_BASE_CM              21.0f
#define CAR_WHEEL_DIAMETER_CM          6.6f

#define TURN_LEFT_DIR                  0U
#define TURN_RIGHT_DIR                 1U

#define CAR_TURN_LEFT_CALIBRATION      0.92f
#define CAR_TURN_RIGHT_CALIBRATION     0.92f

#define GRAY_CHANNEL_COUNT             8U
#define CAR_GRAY_CLK_HIGH_US           5U
#define CAR_GRAY_CLK_LOW_US            5U
#define CAR_GRAY_FRAME_GAP_US          1200U
#define CAR_GRAY_FILTER_ALPHA          0.4f

#define GRAY_INVERT_LOGIC

#define CAR_FINISH_LINE_COUNT          7U
#define CAR_LEAVE_LINE_COUNT           3U

#define K230_EMA_ALPHA                 0.3f    

#define SCR_STEPS_PER_REV             200      
#define SCR_MICROSTEP                 16       
#define SCR_STEPS_PER_REV_EFF         (SCR_STEPS_PER_REV * SCR_MICROSTEP) 

#define SCR_PWM_FREQ_HZ               20000U
#define SCR_PWM_PERIOD_TICKS          1600U
#define SCR_PWM_MAX_DUTY              SCR_PWM_PERIOD_TICKS

#define SCR_DIR_PORT                  GPIO_STEPPER_PORT
#define SCR_DIR_PIN                   GPIO_STEPPER_PIN_DIR_PIN
#define SCR_EN_PORT                   GPIO_STEPPER_PORT
#define SCR_EN_PIN                    GPIO_STEPPER_PIN_EN_PIN
#define SCR_DIR_CW                    1
#define SCR_DIR_CCW                   0

#define SCR_STEP_PORT                 GPIO_PWM_STEPPER_C0_PORT   
#define SCR_STEP_PIN                  GPIO_PWM_STEPPER_C0_PIN    
#define SCR_STEP_PULSE_US             10U    
#define SCR_MAX_STEPS_PER_CYCLE       200U   
#define SCR_STEP_MAX                  200.0f 

#define SCR_LIMIT_PORT                GPIOB
#define SCR_LIMIT_PIN                 DL_GPIO_PIN_12  

#define SCR_CRANK_RADIUS_MM           15.0f   
#define SCR_ROD_LENGTH_MM             120.0f  
#define SCR_PIVOT_DISTANCE_MM         80.0f   
#define SCR_THETA_MAX_DEG             12.0f   

#define MT6816_PPR                    1024
#define MT6816_CPR_4X                 (MT6816_PPR * 4)       
#define MT6816_COUNTS_PER_DEG         ((float)MT6816_CPR_4X / 360.0f)  
#define MT6816_DEG_PER_COUNT          (360.0f / (float)MT6816_CPR_4X)  

#define SCR_OUTER_KP                  0.06f   
#define SCR_OUTER_KD                  0.12f   
#define SCR_OUTER_ANGLE_MAX           10.0f   

#define SCR_OUTER_PERIOD_MS           10U     
#define SCR_INNER_OUTER_RATIO         3U      
#define SCR_INNER_PERIOD_US           3333U   
#define SCR_USE_TIMG1_INNER_LOOP      0       

#define SCR_INNER_KP                  3.0f    

#define SCR_ACC_FF                    0.02f   

#define SCR_PWM_SPEED_MAX             20.0f   

#define SCR_ANGLE_EMA_ALPHA           0.3f    

#define SCR_T3_TOLERANCE              10.0f   
#define SCR_T3_HOLD_PLUS5_MS          5000U   
#define SCR_T3_HOLD_MINUS5_MS         5000U   

#endif
