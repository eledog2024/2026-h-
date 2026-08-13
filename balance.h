#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "Board/car_config.h"

/* 球平衡系统：K230 外环 PD 100Hz + MT6816 内环 P 300Hz */
void StepperCrank_Init(void);
void StepperCrank_ControlOuter(float car_accel_mms2);
void StepperCrank_ControlInner(void);
void StepperCrank_Control(float car_accel_mms2);
void StepperCrank_SetTarget(int16_t target_mm);
int16_t StepperCrank_GetTarget(void);
void StepperCrank_Stop(void);
bool StepperCrank_AtTarget(uint16_t tolerance_mm);

/* Task3：钢球在 ±5cm 间往复并保持 */
void StepperCrank_Task3Reset(void);
bool StepperCrank_Task3Control(void);

/* TIMG1 300Hz 内环触发标志 */
#if SCR_USE_TIMG1_INNER_LOOP
extern volatile bool g_inner_tick;
#endif

/* 调试数据接口 */
float SCR_GetBallPosMm(void);
float SCR_GetBallVelMms(void);
float SCR_GetEncoderAngleDeg(void);
float SCR_GetEncoderVelocityDegS(void);

#endif
