

/* 电机与 PID 控制接口。 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;       
    float ki;       
    float kd;       
    float lim;      
    float sum;      
    float last;     
} PID_t;

void Motor_Init(void);
void Motor_On(void);
void Motor_Off(void);
void Motor_Stop(void);

void Motor_SetRaw(int8_t left, int8_t right);

void Motor_Set(float target_l, float target_r);

void Motor_Reset(void);
void Motor_ResetLinePID(void);
void Motor_StopAll(void);
void PID_Reset(PID_t *p);

void Motor_LineFollow(int16_t pos, int16_t pwm);

void Motor_LineFollowPD(int16_t pos, float rate, int16_t pwm);

void Motor_DriveStraightRPM(void);

float Motor_GetLastOutL(void);
float Motor_GetLastOutR(void);

bool Motor_TurnByEncoder(int8_t speed, uint8_t direction, float angle_deg);

#endif
