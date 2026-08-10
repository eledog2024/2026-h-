/**
 * @file    car_config.h
 * @brief   小车所有可调参数 (集中管理, 无分散常量)
 *
 * 参数按模块组织:
 *   [A] 直流电机 + 霍尔编码器 (循迹)
 *   [B] 灰度传感器 (循迹)
 *   [C] K230 球位置传感器
 *   [D] 步进电机 + 曲柄机构
 *   [E] MT6816 编码器 (电机轴反馈)
 *   [F] 双环平衡控制 (外环 PD + 内环 P + 前馈)
 *   [G] Task3 球往复 ±5cm
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/* ===================================================================== *
 *  [A] 直流电机 + 霍尔编码器 (小车驱动 — 循迹)
 * ===================================================================== */

#define CAR_BASE_SPEED                 30.0f    /* 循迹基准速度 (RPM) */
#define CAR_TURN_SPEED_DIFF_RPM        50.0f    /* 转弯差速 (RPM) */

/* 位置 PID (外环) */
#define CAR_PID_OUTER_KP               0.15f
#define CAR_PID_OUTER_KI               0.00f
#define CAR_PID_OUTER_KD               0.50f

/* 轮速 PID (内环) */
#define CAR_SPEED_PID_KP               1.0f
#define CAR_SPEED_PID_KI               0.2f
#define CAR_SPEED_PID_KD               0.05f
#define CAR_SPEED_PID_OUTPUT_LIMIT     50.0f
#define CAR_SPEED_OUT_FILTER_ALPHA     0.5f

/* RPM → PWM 标定系数 (实测) */
#define CAR_RPM_TO_PWM_L               0.219f
#define CAR_RPM_TO_PWM_R               0.231f

/* 霍尔编码器 */
#define ENCODER_PPR_OUTPUT             ((uint16_t)390)
#define ENCODER_SPEED_FILTER_ALPHA     0.3f

/* 车轮几何 (cm) */
#define CAR_WHEEL_BASE_CM              21.0f
#define CAR_WHEEL_DIAMETER_CM          6.6f

/* 转弯方向 */
#define TURN_LEFT_DIR                  0U
#define TURN_RIGHT_DIR                 1U

/* 转弯标定系数 */
#define CAR_TURN_LEFT_CALIBRATION      0.92f
#define CAR_TURN_RIGHT_CALIBRATION     0.92f

/* ===================================================================== *
 *  [B] 灰度传感器 (8 通道 — 循迹)
 * ===================================================================== */

#define GRAY_CHANNEL_COUNT             8U
#define CAR_GRAY_CLK_HIGH_US           5U
#define CAR_GRAY_CLK_LOW_US            5U
#define CAR_GRAY_FRAME_GAP_US          1200U
#define CAR_GRAY_FILTER_ALPHA          0.4f

/* 逻辑反转: 定义后 0=线, 1=背景 */
#define GRAY_INVERT_LOGIC

/* 起点/终点横线检测 */
#define CAR_FINISH_LINE_COUNT          7U
#define CAR_LEAVE_LINE_COUNT           3U

/* ===================================================================== *
 *  [C] K230 球位置传感器
 *
 *  K230 通过 UART1 发送 ASCII 帧: "S<pos>,<vel>,<acc>\r\n"
 *  pos: 球偏移中心距离 (mm), 正=偏向自由端
 *  vel: 球速度 (mm/s)
 *  acc: 球加速度 (mm/s²)
 * ===================================================================== */

#define K230_EMA_ALPHA                 0.3f    /* 球位置低通滤波, 越小越平滑 */

/* ===================================================================== *
 *  [D] 步进电机 + 曲柄摆杆机构
 *
 *  步进: 1.8°/步, 16 微步细分 → 3200 步/圈 (有效)
 *  曲柄: R=15mm, 连杆 L=120mm, 支点距 D=80mm
 *  摆杆: 250mm 长, 中心 O=125mm, 倾角 ±12°
 *
 *  硬件: PA0(PWM=TIMA0_CCP0) PA1(DIR) PB14(EN)
 * ===================================================================== */

/* ── 电机参数 ── */
#define SCR_STEPS_PER_REV             200      /* 步进电机基础步数/圈 (1.8°) */
#define SCR_MICROSTEP                 16       /* 微步细分 (驱动器设置) */
#define SCR_STEPS_PER_REV_EFF         (SCR_STEPS_PER_REV * SCR_MICROSTEP) /* 3200 */

/* ── Stepper PWM (SysConfig PWM_STEPPER: TIMA0, period=1600 → 20kHz) ── */
#define SCR_PWM_FREQ_HZ               20000U
#define SCR_PWM_PERIOD_TICKS          1600U
#define SCR_PWM_MAX_DUTY              SCR_PWM_PERIOD_TICKS

/* ── 引脚 (SysConfig GPIO_STEPPER) ── */
#define SCR_DIR_PORT                  GPIO_STEPPER_PORT
#define SCR_DIR_PIN                   GPIO_STEPPER_PIN_DIR_PIN
#define SCR_EN_PORT                   GPIO_STEPPER_PORT
#define SCR_EN_PIN                    GPIO_STEPPER_PIN_EN_PIN
#define SCR_DIR_CW                    1
#define SCR_DIR_CCW                   0

/* ── 步进脉冲 (测试文件用 STEP/DIR 脉冲模式) ── */
#define SCR_STEP_PORT                 GPIO_PWM_STEPPER_C0_PORT   /* PA0, SysConfig PWM_STEPPER */
#define SCR_STEP_PIN                  GPIO_PWM_STEPPER_C0_PIN    /* PA0 */
#define SCR_STEP_PULSE_US             10U    /* 步进脉冲宽度 (us) */
#define SCR_MAX_STEPS_PER_CYCLE       200U   /* 每控制周期最大步数 */
#define SCR_STEP_MAX                  200.0f /* 步数指令限幅 */

/* ── 限位开关 (安全保护, 可选) ── */
#define SCR_LIMIT_PORT                GPIOB
#define SCR_LIMIT_PIN                 DL_GPIO_PIN_12  /* PB12, 需在 SysConfig 中添加 GPIO_LIMIT */


/* ── 机械参数 ── */
#define SCR_CRANK_RADIUS_MM           15.0f   /* 曲柄半径 R */
#define SCR_ROD_LENGTH_MM             120.0f  /* 连杆长度 L */
#define SCR_PIVOT_DISTANCE_MM         80.0f   /* 摆杆连接点距铰链距离 */
#define SCR_THETA_MAX_DEG             12.0f   /* 摆杆最大倾角 */

/* ===================================================================== *
 *  [E] MT6816 编码器 (电机轴反馈)
 *
 *  1024 PPR × 4X = 4096 CPR (脉冲/圈)
 *  PWM 占空比 0~100% → 0~360° 绝对角度
 *  Z 相每圈一次索引
 *
 *  接线: PA4(ENC_A) PA5(ENC_B) PA6(ENC_PWM) PA7(ENC_Z)
 * ===================================================================== */

#define MT6816_PPR                    1024
#define MT6816_CPR_4X                 (MT6816_PPR * 4)       /* 4096 */
#define MT6816_COUNTS_PER_DEG         ((float)MT6816_CPR_4X / 360.0f)  /* ~11.38 */
#define MT6816_DEG_PER_COUNT          (360.0f / (float)MT6816_CPR_4X)  /* ~0.0879° */

/* ===================================================================== *
 *  [F] 双环球平衡控制
 *
 *  控制架构:
 *    K230 球位置 →
 *    外环 PD: 位置+速度 → 目标曲柄角度 (度)
 *    内环 P:  角度误差 → PWM 占空比 (速度)
 *    前馈:    小车加速度 → 补偿占空比
 *
 *  调参顺序: 外环 Kp/Kd → 内环 Kp → 前馈 Kff
 * ===================================================================== */

/* ── 外环: 球位置 PD → 目标曲柄角度 (°) ── */
#define SCR_OUTER_KP                  0.06f   /* 位置误差 1mm → 0.06° 倾角 */
#define SCR_OUTER_KD                  0.12f   /* 速度 1mm/s → 0.12° 阻尼 */
#define SCR_OUTER_ANGLE_MAX           10.0f   /* 目标角度限幅 (±10°) */

/* ── 内外环分频控制 ──
 *
 *   外环 (K230 + PD):   100Hz (10ms) — 视觉传感器更新慢, 无需高频
 *   内环 (编码器 + P):  300Hz (~3.33ms) — 编码器高频反馈, 3x 外环
 *   分频比: 内环 : 外环 = SCR_INNER_OUTER_RATIO : 1
 *
 *   实现:
 *     - TIMG1 产生 300Hz 中断 → 内环 P + PWM 更新
 *     - SysTick 10ms → 外环 PD + 目标角度更新
 *     - 若 TIMG1 未使能, StepperCrank_Control() 自动降级为 3x 软件子循环
 *
 *   调参顺序: 外环 Kp/Kd → 内环 Kp → 前馈 Kff
 * ===================================================================== */
#define SCR_OUTER_PERIOD_MS           10U     /* 外环周期 (ms) */
#define SCR_INNER_OUTER_RATIO         3U      /* 内环:外环 频率比 */
#define SCR_INNER_PERIOD_US           3333U   /* 内环周期 (us) ≈ 10ms/3 */
#define SCR_USE_TIMG1_INNER_LOOP      0       /* 1=使用TIMG1硬件定时, 0=软件3x子循环 */

/* ── 内环: 角度误差 P → PWM 速度指令 ── */
#define SCR_INNER_KP                  3.0f    /* 角度误差 1° → 速度指令 +3 */

/* ── 前馈: 小车加速度 → 补偿速度指令 ── */
#define SCR_ACC_FF                    0.02f   /* 1mm/s² → +0.02 速度指令 */

/* ── 速度限幅 ── */
#define SCR_PWM_SPEED_MAX             20.0f   /* max speed cmd (PWM duty reference) */

/* ── EMA 滤波系数 ── */
#define SCR_ANGLE_EMA_ALPHA           0.3f    /* 编码器角度 EMA 平滑 */

/* ===================================================================== *
 *  [G] Task 3: 静止球往复 ±5cm
 *
 *  复用双环参数, 独立状态机.
 *  静止场景下 car_accel=0, 不使用前馈.
 * ===================================================================== */

#define SCR_T3_TOLERANCE              10.0f   /* 到达判定容差 (mm) */
#define SCR_T3_HOLD_PLUS5_MS          1000U   /* +5cm 保持时间 (ms) */
#define SCR_T3_HOLD_MINUS5_MS         1000U   /* -5cm 保持时间 (ms) */

#endif /* CAR_CONFIG_H */
