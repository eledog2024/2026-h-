/**
 * @file    imu.h
 * @brief   HWT906 9轴 IMU 传感器驱动头文件 (UART 接口)
 *
 * 传感器: WitMotion HWT906 (MPU-9250 + 片载卡尔曼滤波)
 * 通信:   UART3, 115200 baud, 8N1, 连续输出模式
 *
 * 数据输出:
 *   - 加速度: Ax, Ay, Az (m/s²), 量程 ±16g
 *
 * 接线 (见 hw_config.h / empty.syscfg):
 *   PB2 (UART3_TX) → HWT906 RX
 *   PB3 (UART3_RX) → HWT906 TX
 *   3.3V / GND       → HWT906 VCC / GND
 *
 * WitMotion UART 协议 (连续输出模式):
 *   - 帧头 0x55, 标签字节, 数据段, 校验和
 *   - 0x51: 加速度帧 (AxL AxH AyL AyH AzL AzH TL TH SUM)
 *           转换: accel = raw / 32768 * 16g (g=9.8m/s²)
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/* ===================================================================== *
 *  API 函数
 * ===================================================================== */

/**
 * @brief 初始化 UART 外设, 使能 RX 中断, 等待 HWT906 上电稳定
 *
 *        配置 UART3 为主模式 115200-8N1,
 *        PB2=TX, PB3=RX.
 *
 * @note  需在 main() 初始化阶段调用.
 *        上电后 HWT906 约需 500~1000ms 稳定, 此函数包含等待.
 */
void IMU_Init(void);

/**
 * @brief 读取三轴加速度
 * @param ax 输出: X 轴加速度 (m/s²)
 * @param ay 输出: Y 轴加速度 (m/s²) — 小车前进/后退方向
 * @param az 输出: Z 轴加速度 (m/s²)
 *
 * 转换公式: accel = raw / 32768 * 16 * 9.8 (m/s²)
 *           raw 来自 0x51 帧 AxL/H, AyL/H, AzL/H (int16 LE)
 */
void IMU_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief 读取 Y 轴加速度 (小车前进方向)
 * @return Y 轴加速度 (m/s²), >0 为前进加速
 */
float IMU_ReadAccelY(void);

/**
 * @brief 读取最近一次解析的原始 int16 加速度值 (调试用)
 */
void IMU_ReadRawAccel(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az);

/**
 * @brief 调试: 获取帧统计信息
 * @param cnt_51  输出: 收到的 0x51 (加速度) 帧数
 * @param cnt_53  输出: 收到的 0x53 (角度) 帧数
 * @param cnt_other 输出: 收到其他标签的帧数
 * @param last_tag 输出: 最近一次收到的标签字节 (0=未收到)
 */
void IMU_GetFrameStats(uint32_t *cnt_51, uint32_t *cnt_53,
                       uint32_t *cnt_other, uint8_t *last_tag);

#endif /* IMU_H */
