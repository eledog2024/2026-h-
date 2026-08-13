/* 双轮电机与循迹控制。 */
#include "motor.h"
#include "encoder.h"
#include "gray.h"
#include "Board/car_config.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define PWM_TOP        1000U   
#define MID_POS        80.0f
#define PIVOT_SPEED    10     
#define PIVOT_STOP_MS             30U
#define PIVOT_EXIT_STOP_FRAMES 3       

static PID_t pos_pid;       
static PID_t spd_pid_l;     
static PID_t spd_pid_r;     
static float last_out_l;    
static float last_out_r;    

/* 初始化 PID 参数。 */
static void pid_init(PID_t *p, float kp, float ki, float kd, float lim)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->lim = lim;
    p->sum = 0.0f;
    p->last = 0.0f;
}

/* 计算 PID 输出。 */
static float pid_run(PID_t *p, float err)
{
    float out;

    p->sum += err;
    out = p->kp * err + p->ki * p->sum
        + p->kd * (err - p->last);
    p->last = err;
    return out;
}

/* 设置单侧电机输出。 */
static void set_one(uint8_t side, int8_t pwm)
{
    uint32_t cmp;
    int16_t val = -(int16_t)pwm; 

    if (val > 100) val = 100;
    if (val < -100) val = -100;
    cmp = PWM_TOP - (PWM_TOP * (uint32_t)(val < 0 ? -val : val) / 100U);

    if (side == 0U) {
        
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, cmp, PWM_L_CC_INDEX);
        if (val > 0) {
            DL_GPIO_setPins( GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_ML1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_ML2_PIN);
        } else if (val < 0) {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_ML1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_ML2_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_ML1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_ML2_PIN);
        }
    } else {
        
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, cmp, PWM_R_CC_INDEX);
        if (val > 0) {
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_MR1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_MR2_PIN);
        } else if (val < 0) {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_MR1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_MR2_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_MR1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_MR2_PIN);
        }
    }
}

/* 提供电机控制延时。 */
static void motor_delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms * 8000U; i++) {
        __NOP();
    }
}

/* 初始化双轮电机。 */
void Motor_Init(void)
{
    Motor_Off();
    pid_init(&pos_pid, CAR_PID_OUTER_KP, CAR_PID_OUTER_KI,
             CAR_PID_OUTER_KD, CAR_TURN_SPEED_DIFF_RPM);
    pid_init(&spd_pid_l, CAR_SPEED_PID_KP, CAR_SPEED_PID_KI,
             CAR_SPEED_PID_KD, CAR_SPEED_PID_OUTPUT_LIMIT);
    pid_init(&spd_pid_r, CAR_SPEED_PID_KP, CAR_SPEED_PID_KI,
             CAR_SPEED_PID_KD, CAR_SPEED_PID_OUTPUT_LIMIT);
}

/* 使能电机驱动。 */
void Motor_On(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_PIN_STBY_PIN);
    DL_TimerA_startCounter(PWM_MOTOR_INST);
}

/* 关闭电机驱动。 */
void Motor_Off(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_STBY_PIN);
    Motor_Stop();
}

/* 直接设置双轮 PWM。 */
void Motor_SetRaw(int8_t left, int8_t right)
{
    set_one(0U, left);
    set_one(1U, right);
}

/* 闭环设置双轮轮速。 */
void Motor_Set(float target_l, float target_r)
{
    float rpm_l = Encoder_GetRPM(ENCODER_L);
    float rpm_r = Encoder_GetRPM(ENCODER_R);

    
    float ff_l = (target_l >= 0.0f ? target_l : -target_l) * CAR_RPM_TO_PWM_L;
    float ff_r = (target_r >= 0.0f ? target_r : -target_r) * CAR_RPM_TO_PWM_R;

    
    float err_l = target_l - rpm_l;
    float err_r = target_r - rpm_r;
    float pid_l = pid_run(&spd_pid_l, err_l);
    float pid_r = pid_run(&spd_pid_r, err_r);

    
    float raw_l = (target_l >= 0.0f ? ff_l : -ff_l) + pid_l;
    float raw_r = (target_r >= 0.0f ? ff_r : -ff_r) + pid_r;

    
    last_out_l += CAR_SPEED_OUT_FILTER_ALPHA * (raw_l - last_out_l);
    last_out_r += CAR_SPEED_OUT_FILTER_ALPHA * (raw_r - last_out_r);

    Motor_SetRaw((int8_t)last_out_l, (int8_t)last_out_r);
}

/* 停止双轮电机。 */
void Motor_Stop(void)
{
    Motor_SetRaw(0, 0);
}

/* 复位循迹 PID。 */
void Motor_Reset(void)
{
    pid_init(&pos_pid, CAR_PID_OUTER_KP, CAR_PID_OUTER_KI,
             CAR_PID_OUTER_KD, CAR_TURN_SPEED_DIFF_RPM);
}

/* 清除 PID 积分状态。 */
void PID_Reset(PID_t *p)
{
    p->sum = 0.0f;
    p->last = 0.0f;
}

/* 复位全部电机 PID。 */
void Motor_ResetLinePID(void)
{
    Motor_Reset();
    PID_Reset(&spd_pid_l);
    PID_Reset(&spd_pid_r);
}

/* 停止并复位电机。 */
void Motor_StopAll(void)
{
    Motor_Stop();
    Motor_Reset();
}

/* 按灰度位置循迹。 */
void Motor_LineFollow(int16_t pos, int16_t pwm)
{
    float err;
    float dif;
    float tar_l;
    float tar_r;

    
    static bool   pivot_mode      = false;  
    static bool   pivot_wait      = false;  
    static int8_t pivot_dir       = 0;      
    static bool   line_seen       = false;  
    static uint8_t exit_stop_cnt  = 0;      

    uint8_t raw        = Gray_GetRawByte();
    uint8_t line_count = Gray_GetLineCount();
    bool    line_lost  = (line_count == 0);

    
    if (exit_stop_cnt > 0) {
        Motor_Stop();
        exit_stop_cnt--;
        if (exit_stop_cnt == 0) {
            pivot_mode = false;
            pivot_dir  = 0;
            Motor_ResetLinePID();
        }
        return;
    }

    
    if (pivot_mode && line_count >= 2) {
        exit_stop_cnt = PIVOT_EXIT_STOP_FRAMES;
        Motor_Stop();
        return;
    }

    
    if (pivot_mode) {
        
        if (line_count >= 2) {
            line_seen = true;               
        } else if (line_seen) {
            
            pivot_dir  = -pivot_dir;
            pivot_wait = true;
            line_seen  = false;
            Motor_Stop();
            return;
        }

        if (pivot_wait) {
            
            pivot_wait = false;
            Motor_Stop();
            return;
        }

        if (pivot_dir < 0) {
            Motor_SetRaw(-PIVOT_SPEED, PIVOT_SPEED);  
        } else {
            Motor_SetRaw(PIVOT_SPEED, -PIVOT_SPEED);  
        }
        return;
    }

    
    if (line_lost) {
        pivot_mode = true;
        pivot_wait = true;
        line_seen  = false;
        
        pivot_dir = (pos < (int16_t)MID_POS) ? -1 : 1;
        Motor_Stop();
        return;
    }

    
    err = (float)pos - MID_POS;

    
    dif = pid_run(&pos_pid, err);
    tar_l = pwm + dif;
    tar_r = pwm - dif;

    Motor_SetRaw((int8_t)tar_l, (int8_t)tar_r);
}

/* 按基准速度直行。 */
void Motor_DriveStraightRPM(void)
{
    Motor_SetRaw((int8_t)CAR_BASE_SPEED, (int8_t)CAR_BASE_SPEED);
}

/* 读取左轮输出。 */
float Motor_GetLastOutL(void) { return last_out_l; }
/* 读取右轮输出。 */
float Motor_GetLastOutR(void) { return last_out_r; }

/* 按编码器角度转向。 */
bool Motor_TurnByEncoder(int8_t speed, uint8_t direction, float angle_deg)
{
    float target_total_f = (CAR_WHEEL_BASE_CM * angle_deg
                          * (float)ENCODER_PPR_OUTPUT)
                       / (180.0f * CAR_WHEEL_DIAMETER_CM);                       
    int32_t target_total = (int32_t)target_total_f;
    uint32_t timeout_ms = 5000U;
    uint32_t elapsed = 0U;

    
    int32_t pulse_start_L = (int32_t)Encoder_GetPulse(ENCODER_L);
    int32_t pulse_start_R = (int32_t)Encoder_GetPulse(ENCODER_R);
    int32_t total_pulses = 0;

    
    if (direction == TURN_LEFT_DIR) {
        Motor_SetRaw(-speed, speed);
    } else {
        Motor_SetRaw(speed, -speed);
    }

    
    while (total_pulses < target_total && elapsed < timeout_ms) {
        motor_delay_ms(1);
        elapsed++;
        int32_t cur_L = (int32_t)Encoder_GetPulse(ENCODER_L);
        int32_t cur_R = (int32_t)Encoder_GetPulse(ENCODER_R);
        int32_t diff_L = cur_L - pulse_start_L;
        int32_t diff_R = cur_R - pulse_start_R;
        total_pulses = (diff_L >= 0 ? diff_L : -diff_L)
                     + (diff_R >= 0 ? diff_R : -diff_R);
    }

    Motor_Stop();
    return (total_pulses >= target_total);
}

