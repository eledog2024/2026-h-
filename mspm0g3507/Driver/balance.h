/* 钢球平衡控制接口。 */
#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "../Board/car_config.h"

void StepperCrank_Init(void);
void StepperCrank_ControlOuter(float car_accel_mms2);
void StepperCrank_ControlInner(void);
void StepperCrank_Control(float car_accel_mms2);
void StepperCrank_SetTarget(int16_t target_mm);
int16_t StepperCrank_GetTarget(void);
void StepperCrank_Stop(void);
bool StepperCrank_AtTarget(uint16_t tolerance_mm);

void StepperCrank_Task3Reset(void);
bool StepperCrank_Task3Control(void);

#if SCR_USE_TIMG1_INNER_LOOP
extern volatile bool g_inner_tick;
#endif

float SCR_GetBallPosMm(void);
float SCR_GetBallVelMms(void);
float SCR_GetEncoderAngleDeg(void);
float SCR_GetEncoderVelocityDegS(void);

#endif
