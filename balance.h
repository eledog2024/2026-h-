/**
 * @file    balance.h
 * @brief   步进电机 + 曲柄摆杆球平衡系统 (唯一方案)
 *
 * 传感器:
 *   K230 视觉 (UART1) → 球位置/速度/加速度
 *   MT6816 编码器 (PA4-7) → 步进电机轴绝对角度
 *
 * 执行器:
 *   步进电机 (PWM+DIR 接口, PA0=TIMA0_CCP0, PA1=DIR) + 曲柄连杆 → 摆杆倾角
 *
 * 控制架构 (双环, 分频 3:1):
 *   [外环 PD] 球位置+速度 → 目标曲柄角度 (°)    @ 100Hz (10ms)
 *   [内环 P]  角度误差 → PWM 占空比 (速度)       @ 300Hz (~3.33ms)
 *   [前馈]    小车加速度 → 补偿速度指令
 *
 * 分频实现:
 *   - 默认 (软件模式):
 *         每 10ms 调用 StepperCrank_Control() 一次.
 *         内部自动执行: 外环 PD ×1 + 内环 P ×3 (逐次刷新编码器).
 *
 *   - 可选 (硬件定时器模式, SCR_USE_TIMG1_INNER_LOOP=1):
 *         每 10ms 调用 StepperCrank_ControlOuter() (或 StepperCrank_Control()).
 *         TIMG1 @ 300Hz ISR → g_inner_tick 标志.
 *         主循环轮询: if(g_inner_tick){g_inner_tick=false; StepperCrank_ControlInner();}
 *
 * 编码器直接输出角度 = 曲柄角, 无需运动学换算.
 * MT6816 PWM 提供绝对角度, 上电即知位置, 无需回零.
 */

#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "Board/car_config.h"

/* ===================================================================== *
 *  API — 初始化 + 主控制
 * ===================================================================== */

/**
 * @brief 初始化系统 (步进电机引脚 + MT6816 + 可选 TIMG1 内环定时器)
 *
 * 内部: 配置 GPIO → 初始化 MT6816 → 脱机等待.
 * 若 SCR_USE_TIMG1_INNER_LOOP=1, 同时初始化 TIMG1 @ 300Hz.
 * 上电后 PWM 立即给出绝对角度, 无需回零.
 */
void StepperCrank_Init(void);

/**
 * @brief 外环 PD (100Hz): 球位置+速度 → 目标曲柄角度 (°)
 *
 * 每 10ms 调用一次 (SysTick).
 *
 * 流程:
 *   1. K230_ParseStream() 解析球位置
 *   2. EMA 滤波球位置
 *   3. PD 计算 → g_target_angle_deg
 *   4. 前馈分量 → g_feedforward_cmd (供内环)
 *
 * @param car_accel_mms2  小车加速度 (mm/s²), 静止场景传 0.
 */
void StepperCrank_ControlOuter(float car_accel_mms2);

/**
 * @brief 内环 P (300Hz): 角度误差 → PWM 速度指令
 *
 * 每 ~3.33ms 调用一次 (TIMG1 / 软件 3x 子循环).
 *
 * 流程:
 *   1. MT6816_Update() 更新编码器状态
 *   2. P 控制: step_cmd = Kp × (target_angle - actual_angle)
 *   3. 叠加前馈: step_cmd += g_feedforward_cmd
 *   4. 限幅 → PWM duty 输出
 *   5. Z 相校准检测
 *
 * @note 读取 g_target_angle_deg 和 g_feedforward_cmd (外环写入).
 */
void StepperCrank_ControlInner(void);

/**
 * @brief 双环球平衡控制 (兼容接口, 每 10ms 调用)
 *
 * 内部自动分频:
 *   外环 PD ×1 (100Hz)
 *   内环 P  ×SCR_INNER_OUTER_RATIO=3 (300Hz 等效)
 *
 * 硬件定时器模式 (SCR_USE_TIMG1_INNER_LOOP=1):
 *   仅执行外环; 内环由 TIMG1 ISR 驱动.
 *   调用者须在主循环中轮询 g_inner_tick 并调用 StepperCrank_ControlInner().
 *
 * @param car_accel_mms2  小车加速度 (mm/s²), 前馈用. 静止场景传 0.
 */
void StepperCrank_Control(float car_accel_mms2);

/** @brief 设置目标球位置 (mm, 相对中心) */
void StepperCrank_SetTarget(int16_t target_mm);

/** @brief 获取当前目标 */
int16_t StepperCrank_GetTarget(void);

/** @brief 停止并脱机电机, 清除前馈和使能标志 */
void StepperCrank_Stop(void);

/** @brief 球是否到达目标 (±容差) */
bool StepperCrank_AtTarget(uint16_t tolerance_mm);

/* ===================================================================== *
 *  Task 3: 静止球往复 ±5cm
 * ===================================================================== */

void StepperCrank_Task3Reset(void);
bool StepperCrank_Task3Control(void);

/* ===================================================================== *
 *  内环定时器标志 (仅 SCR_USE_TIMG1_INNER_LOOP=1 时有效)
 *
 *  TIMG1 ISR (@ 300Hz) 置位 g_inner_tick.
 *  调用者在主循环中轮询并消费:
 *
 *    while (1) {
 *        while (!g_tick && !g_inner_tick) { __WFI(); }
 *        if (g_tick)  { g_tick = false;  StepperCrank_ControlOuter(accel); }
 *        if (g_inner_tick) { g_inner_tick = false; StepperCrank_ControlInner(); }
 *    }
 * ===================================================================== */

#if SCR_USE_TIMG1_INNER_LOOP
extern volatile bool g_inner_tick;
#endif

/* ===================================================================== *
 *  调试接口
 * ===================================================================== */

float   SCR_GetBallPosMm(void);            /* 滤波后球位置 (mm) */
float   SCR_GetBallVelMms(void);           /* 球速度 (mm/s) */
float   SCR_GetEncoderAngleDeg(void);      /* 编码器测量角度 (°) */
float   SCR_GetEncoderVelocityDegS(void);  /* 编码器角速度 (°/s) */

#endif /* BALANCE_H */
