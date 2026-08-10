/**
 * @file    mt6816.h
 * @brief   MT6816 高精度磁编码器驱动
 *
 * MT6816 是一款 14-bit 高精度磁编码器, 提供:
 *   - AB 相: 1024 线正交编码器 (4倍频后 4096 脉冲/圈)
 *   - PWM:   绝对角度输出 (占空比 0~100% 对应 0~360°)
 *   - Z 相:  每圈一次索引脉冲 (固定参考点, 断电保持)
 *
 * 硬件接线 (MSPM0G3507):
 *   PA4 → ENC_A   (编码器 A 相, 双边沿中断)
 *   PA5 → ENC_B   (编码器 B 相, 双边沿中断)
 *   PA6 → ENC_PWM (编码器 PWM 输出, 双边沿中断)
 *   PA7 → ENC_Z   (编码器 Z 相索引, 上升沿中断)
 *
 * 架构:
 *   - AB 相通过 GPIO 双边沿中断实现 4X 正交解码 (4096 CPR)
 *   - PWM 占空比通过 GPIO 双边沿中断 + 硬件定时器时间戳测量
 *   - Z 相触发时校准绝对零点
 *
 * 使用方式:
 *   1. MT6816_Init() 初始化硬件
 *   2. 每 10ms 调用 MT6816_Update() 更新状态
 *   3. 通过 API 获取角度、速度等
 */

#ifndef MT6816_H
#define MT6816_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================== *
 *  编码器参数
 * ===================================================================== */

#define MT6816_PPR                    1024     /* AB 相 脉冲数/圈 */
#define MT6816_CPR_4X                 (MT6816_PPR * 4)  /* 4X 计数/圈 = 4096 */
#define MT6816_COUNTS_PER_DEG         ((float)MT6816_CPR_4X / 360.0f)  /* ~11.38 */
#define MT6816_DEG_PER_COUNT          (360.0f / (float)MT6816_CPR_4X)  /* ~0.0879° */

/* ===================================================================== *
 *  API
 * ===================================================================== */

/**
 * @brief 初始化 MT6816 编码器
 *
 * 配置:
 *   - GPIO PA4/PA5/PA6/PA7 引脚方向、Polarity、中断
 *   - TIMG2 自由运行定时器 (用于 PWM 时间戳)
 *   - 使能 NVIC 中断 (GPIOA)
 *
 * @note 在调用前需确保 SYSCFG_DL_init() 已完成
 */
void MT6816_Init(void);

/**
 * @brief 更新编码器状态 (每 10ms 调用一次)
 *
 * 内部计算:
 *   - 角度 EMA 滤波 (PWM 绝对角 + 编码器增量融合)
 *   - 角速度 (差分 + EMA)
 *   - Z 相校准检测
 */
void MT6816_Update(void);

/**
 * @brief 获取编码器原始计数值 (4X 正交, 累计跨圈)
 * @return 计数值, 正转递增, 反转递减
 */
int32_t MT6816_GetRawCount(void);

/**
 * @brief 获取当前绝对角度 (0.0 ~ 360.0 度)
 *
 * 融合 PWM 绝对角度与编码器增量, EMA 滤波.
 *
 * @return 角度 (度)
 */
float MT6816_GetAngleDeg(void);

/**
 * @brief 获取角速度
 *
 * 编码器位置差分 + EMA 滤波.
 *
 * @return 角速度 (度/秒), CW为正
 */
float MT6816_GetVelocityDegS(void);

/**
 * @brief 获取 PWM 通道测量的原始角度
 *
 * 直接由 PWM 占空比计算, 未经融合/滤波.
 *
 * @return 角度 (度), 0.0 ~ 360.0
 */
float MT6816_GetPWMAngleDeg(void);

/**
 * @brief 是否检测到 Z 相索引脉冲
 *
 * 每次 Z 中断置位, Update() 中可清除.
 *
 * @return true=检测到Z相, false=未检测到
 */
bool MT6816_IsZDetected(void);

/**
 * @brief 手动复位编码器位置为零
 */
void MT6816_ResetPosition(void);

/**
 * @brief 获取完整圈数 (从初始化开始)
 * @return 圈数 (正转+, 反转-)
 */
int32_t MT6816_GetRevolutions(void);

/**
 * @brief GPIOA 中断处理函数
 *
 * 由平台中断向量调用, 处理所有 MT6816 引脚中断:
 *  - PA4/PA5: AB 相正交解码
 *  - PA6:     PWM 占空比测量
 *  - PA7:     Z 相索引检测
 */
void MT6816_IRQHandler(void);

#endif /* MT6816_H */
