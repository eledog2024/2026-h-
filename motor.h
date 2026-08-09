/**
 * @file    motor.h
 * @brief   电机控制: 循迹 + 精准转向
 *
 * 硬件:
 *   - 8路灰度传感器 (白底红线)
 *   - 左右霍尔编码器电机 (差速驱动)
 *
 * 两个核心功能:
 *   Motor_LineFollow()   — 灰度循迹 (外环位置PID)
 *   Motor_PreciseTurn()  — 精准转向 (编码器里程计 + 灰度对齐)
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================== *
 *  PID 控制结构体
 * ===================================================================== */
typedef struct {
    float kp;       /* 比例系数 */
    float ki;       /* 积分系数 */
    float kd;       /* 微分系数 */
    float lim;      /* 输出限幅 */
    float sum;      /* 积分累加 */
    float last;     /* 上次误差 */
} PID_t;

/* ---- 基础控制 ---- */
void Motor_Init(void);
void Motor_On(void);
void Motor_Off(void);
void Motor_Stop(void);

/** @brief 底层: 直接写 PWM, 不经 PID/滤波 (供内部转弯/停止使用) */
void Motor_SetRaw(int8_t left, int8_t right);

/**
 * @brief 编码器速度闭环: 以目标 RPM 驱动左右轮
 *
 *        根据编码器 RPM 反馈实时调整 PWM, 自动补偿坡度/负载变化.
 *        内部计算前馈 (RPM→PWM 系数) + PID 修正 + EMA 输出滤波.
 *
 * @param target_l  左轮目标转速 (RPM), 正=前进, 负=后退
 * @param target_r  右轮目标转速 (RPM)
 *
 * 典型用法:
 *   // 在 SysTick ISR (20ms) 中:
 *   Encoder_UpdateSpeed(20);
 *   Motor_Set(30.0f, 30.0f);  // 左右轮都跑 30 RPM
 *
 *   // 循迹主循环 (10ms) 中:
 *   Motor_LineFollow(pos, CAR_BASE_SPEED);  // 外环位置PID → 调用 Motor_Set
 */
void Motor_Set(float target_l, float target_r);

void Motor_Reset(void);
void Motor_ResetLinePID(void);
void Motor_StopAll(void);
void PID_Reset(PID_t *p);

/* ---- 循迹 ---- */

/**
 * @brief 灰度位置循迹 (外环 PID, 直接 PWM 输出)
 *
 * @param pos  灰度位置 0~160 (来自 Gray_GetPosition())
 * @param pwm  基础速度 0~100
 *
 * 典型用法:
 *   while (1) {
 *       int16_t pos = Gray_GetPosition();
 *       Motor_LineFollow(pos, CAR_BASE_SPEED);
 *       delay_ms(20);
 *   }
 */
void Motor_LineFollow(int16_t pos, int16_t pwm);

/**
 * @brief 灰度位置+速率双环循迹 (外环 PD, 直接 PWM 输出)
 *
 * @param pos   灰度位置 0~160
 * @param rate  位置变化速率 (来自 Gray_GetPositionRate())
 * @param pwm   基础速度 0~100
 */
void Motor_LineFollowPD(int16_t pos, float rate, int16_t pwm);

/**
 * @brief 直行 (直接 PWM 输出, 无内环)
 *
 * @param target_rpm  目标转速 (RPM, 当前未使用, 保留以备后用)
 */
void Motor_DriveStraightRPM(void);

float Motor_GetLastOutL(void);
float Motor_GetLastOutR(void);

/**
 * @brief 纯编码器里程计转弯 (粗控制: 开环差速 + 脉冲计数即停)
 *
 * @param speed      基础速度 0~100
 * @param direction  TURN_LEFT_DIR(0) / TURN_RIGHT_DIR(1)
 * @param angle_deg  目标角度 (度)
 * @return true=到位
 */
bool Motor_TurnByEncoder(int8_t speed, uint8_t direction, float angle_deg);

#endif
