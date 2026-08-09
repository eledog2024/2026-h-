#include "motor.h"
#include "encoder.h"
#include "gray.h"
#include "Board/car_config.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define PWM_TOP        1000U   /* 必须与 SysConfig 中 gPWM_MOTORConfig.period 一致 */
#define MID_POS        80.0f
#define PIVOT_SPEED    10     /* pivot 原地旋转固定低速 */
#define PIVOT_STOP_MS      *    30U     /* 进入 pivot 后先停车时长 (ms) */
#define PIVOT_EXIT_STOP_FRAMES 3       /* pivot 退出前停车帧数 */

/* ===================================================================== *
 *  PID 实例
 * ===================================================================== */

static PID_t pos_pid;       /* 外环：位置 -> 转速差 */
static PID_t spd_pid_l;     /* 内环：左轮转速 */
static PID_t spd_pid_r;     /* 内环：右轮转速 */
static float last_out_l;    /* 上一次左轮 PWM 输出 (供 VOFA 读取) */
static float last_out_r;    /* 上一次右轮 PWM 输出 (供 VOFA 读取) */

/* ===================================================================== *
 *  辅助函数
 * ====================== =============================================== */

static void pid_init(PID_t *p, float kp, float ki, float kd, float lim)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->lim = lim;
    p->sum = 0.0f;
    p->last = 0.0f;
}

static float pid_run(PID_t *p, float err)
{
    float out;

    p->sum += err;
    out = p->kp * err + p->ki * p->sum
        + p->kd * (err - p->last);
    p->last = err;
    return out;
}

static void set_one(uint8_t side, int8_t pwm)
{
    uint32_t cmp;
    int16_t val = -(int16_t)pwm; /* 保留原电机安装方向 */

    if (val > 100) val = 100;
    if (val < -100) val = -100;
    cmp = PWM_TOP - (PWM_TOP * (uint32_t)(val < 0 ? -val : val) / 100U);

    if (side == 0U) {
        /* 左轮：PWM_L (PA8, TIMA0_CC0), 方向 ML1/ML2 */
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
        /* 右轮：PWM_R (PA9, TIMA0_CC1), 方向 MR1/MR2 */
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

static void motor_delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms * 8000U; i++) {
        __NOP();
    }
}

/* ===================================================================== *
 *  初始化 / 开关
 * ===================================================================== */

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

void Motor_On(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_PIN_STBY_PIN);
    DL_TimerA_startCounter(PWM_MOTOR_INST);
}

void Motor_Off(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_STBY_PIN);
    Motor_Stop();
}

/* 底层: 直接写 PWM 硬件, 不经过 PID/滤波 (供内部转弯/停止使用) */
void Motor_SetRaw(int8_t left, int8_t right)
{
    set_one(0U, left);
    set_one(1U, right);
}

/**
 * @brief 编码器速度闭环控制 (内环 PI + 前馈)
 *
 *        用编码器 RPM 反馈实时修正 PWM, 使实际转速跟踪目标值.
 *        上坡自动加大 PWM, 下坡自动减小 PWM, 负载变化时自动补偿.
 *
 *        架构:
 *          PWM = 前馈(base_pwm) + PID(target_rpm − actual_rpm)
 *
 *        前馈 (feedforward):
 *          base_pwm = |target_rpm| × CAR_RPM_TO_PWM
 *          让 PID 只需修正偏差, 不需要从零开始积分.
 *
 *        PID 修正:
 *          误差 = target_rpm − Encoder_GetRPM()
 *          PID_out = Kp×err + Ki×∫err + Kd×Δerr
 *
 *        输出滤波 (EMA):
 *          最终 PWM = 上次 PWM + α × (本次 PWM − 上次 PWM)
 *          平滑 PID 高频抖动, 减少电机冲击.
 *
 * @param target_l  左轮目标转速 (RPM), 正=前进, 负=后退
 * @param target_r  右轮目标转速 (RPM)
 *
 * @note  必须在 SysTick 或定时器中周期性调用 Encoder_UpdateSpeed()
 *        来刷新 RPM, 否则 Encoder_GetRPM() 返回的是旧值.
 *        推荐调用周期: 10~20ms.
 *
 * @note  上坡/下坡行为:
 *        平地: 前馈 PWM ≈ 40 就能维持目标 RPM, PID 输出 ≈ 0
 *        上坡: RPM 掉 → PID 积分累积 → 自动加 PWM (最大 +CAR_SPEED_PID_OUTPUT_LIMIT)
 *        下坡: RPM 超 → PID 输出负值 → 自动减 PWM
 */
void Motor_Set(float target_l, float target_r)
{
    float rpm_l = Encoder_GetRPM(ENCODER_L);
    float rpm_r = Encoder_GetRPM(ENCODER_R);

    /* ---- 前馈: RPM → PWM 预估值 ---- */
    float ff_l = (target_l >= 0.0f ? target_l : -target_l) * CAR_RPM_TO_PWM_L;
    float ff_r = (target_r >= 0.0f ? target_r : -target_r) * CAR_RPM_TO_PWM_R;

    /* ---- PID 修正 ---- */
    float err_l = target_l - rpm_l;
    float err_r = target_r - rpm_r;
    float pid_l = pid_run(&spd_pid_l, err_l);
    float pid_r = pid_run(&spd_pid_r, err_r);

    /* ---- 合并: 前馈 + PID ---- */
    float raw_l = (target_l >= 0.0f ? ff_l : -ff_l) + pid_l;
    float raw_r = (target_r >= 0.0f ? ff_r : -ff_r) + pid_r;

    /* ---- EMA 输出滤波 (防抖) ---- */
    last_out_l += CAR_SPEED_OUT_FILTER_ALPHA * (raw_l - last_out_l);
    last_out_r += CAR_SPEED_OUT_FILTER_ALPHA * (raw_r - last_out_r);

    Motor_SetRaw((int8_t)last_out_l, (int8_t)last_out_r);
}

void Motor_Stop(void)
{
    Motor_SetRaw(0, 0);
}

/* ===================================================================== *
 *  PID 状态管理
 * ===================================================================== */

void Motor_Reset(void)
{
    pid_init(&pos_pid, CAR_PID_OUTER_KP, CAR_PID_OUTER_KI,
             CAR_PID_OUTER_KD, CAR_TURN_SPEED_DIFF_RPM);
}

void PID_Reset(PID_t *p)
{
    p->sum = 0.0f;
    p->last = 0.0f;
}

void Motor_ResetLinePID(void)
{
    Motor_Reset();
    PID_Reset(&spd_pid_l);
    PID_Reset(&spd_pid_r);
}

void Motor_StopAll(void)
{
    Motor_Stop();
    Motor_Reset();
}

/* ===================================================================== *
 *  循迹控制
 * ===================================================================== */

/**
 * @brief 灰度位置循迹 (外环 PID, 直接 PWM 输出)
 *
 * @note  新 gray 模块已包含 EMA 低通滤波,
 *        motor 层直接使用位置值, 不重复滤波.
 *        如需更平滑的响应, 可在 car_config.h 中增大 CAR_GRAY_FILTER_ALPHA.
 */
void Motor_LineFollow(int16_t pos, int16_t pwm)
{
    float err;
    float dif;
    float tar_l;
    float tar_r;

    /*
     * 丢线恢复状态机 (静态变量, 跨调用保持)
     *
     * 场景: 线跑出传感器视野 → 彻底丢线.
     *       丢线时刻停车, 下一时刻原地旋转找线.
     *
     * 策略:
     *   1. 丢线第1帧: 立即停车; 根据丢线前 pos 记录方向
     *   2. 丢线第2帧起: 低速原地旋转找线
     *   3. 旋转中短暂看到线又丢 → 反方向旋转 (转过了)
     *   4. 旋转中看到 ≥2 盏 → 停车, 复位 PID, 恢复循迹
     */
    static bool   pivot_mode      = false;  /* true=正在原地旋转找线 */
    static bool   pivot_wait      = false;  /* true=丢线/换向第1帧, 只停车 */
    static int8_t pivot_dir       = 0;      /* -1=左转, +1=右转 */
    static bool   line_seen       = false;  /* 旋转中是否短暂看到过线 */
    static uint8_t exit_stop_cnt  = 0;      /* pivot 退出前停车计数 */

    uint8_t raw        = Gray_GetRawByte();
    uint8_t line_count = Gray_GetLineCount();
    bool    line_lost  = (line_count == 0);

    /* ---- pivot 退出停车阶段: 停稳后再恢复循迹 ---- */
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

    /* ---- 退出 pivot: 旋转中看到 ≥2 盏 → 停车, 恢复循迹 ---- */
    if (pivot_mode && line_count >= 2) {
        exit_stop_cnt = PIVOT_EXIT_STOP_FRAMES;
        Motor_Stop();
        return;
    }

    /* ---- pivot 模式: 旋转 + 换向检测 ---- */
    if (pivot_mode) {
        /* 检测: 旋转中短暂看到 ≥2 盏又丢 → 反方向 */
        if (line_count >= 2) {
            line_seen = true;               /* 记录: 看到过线 */
        } else if (line_seen) {
            /* 之前看到线, 现在又丢了 → 转过站, 反向 */
            pivot_dir  = -pivot_dir;
            pivot_wait = true;
            line_seen  = false;
            Motor_Stop();
            return;
        }

        if (pivot_wait) {
            /* 换向后第1帧: 只停车 */
            pivot_wait = false;
            Motor_Stop();
            return;
        }

        if (pivot_dir < 0) {
            Motor_SetRaw(-PIVOT_SPEED, PIVOT_SPEED);  /* 左转 */
        } else {
            Motor_SetRaw(PIVOT_SPEED, -PIVOT_SPEED);  /* 右转 */
        }
        return;
    }

    /* ---- 检测: 丢线 → 触发 pivot ---- */
    if (line_lost) {
        pivot_mode = true;
        pivot_wait = true;
        line_seen  = false;
        /*
         * 方向由丢线前最后一帧的 pos 决定:
         *   pos < 80 → 线偏左, 左转找回
         *   pos > 80 → 线偏右, 右转找回
         * pos 来自 Gray_GetPosition(), 丢线时返回 g_last_position
         */
        pivot_dir = (pos < (int16_t)MID_POS) ? -1 : 1;
        Motor_Stop();
        return;
    }

    /*
     * 正常循迹: 外环 PID
     *   gray 位置 0~160, 中心 80
     *   err > 0 → 线偏右 → 右轮加速/左轮减速
     *   err < 0 → 线偏左 → 左轮加速/右轮减速
     */
    err = (float)pos - MID_POS;

    /* 外环给出左右轮 PWM 差速 */
    dif = pid_run(&pos_pid, err);
    tar_l = pwm + dif;
    tar_r = pwm - dif;

    Motor_SetRaw((int8_t)tar_l, (int8_t)tar_r);
}

/**
 * @brief 灰度位置+速率双环循迹 (外环 PD, 直接 PWM 输出)
 *
 * @note  利用 Gray_GetPositionRate() 提供的位置变化速率作为超前预测:
 *        - rate > 0: 线向右移动 (车在向左偏) → D 项提前修正
 *        - rate < 0: 线向左移动 (车在向右偏) → D 项提前修正
 *        相比纯 P 控制, PD 可以更快响应弯道, 减小过冲.
 *
 *        调用方式:
 *          float rate;
 *          int16_t pos = Gray_GetPositionAndRate(&rate);
 *          Motor_LineFollowPD(pos, rate, CAR_BASE_SPEED);
 */

/* ===================================================================== *
 *  直行
 * ===================================================================== */

void Motor_DriveStraightRPM(void)
{
    Motor_SetRaw((int8_t)CAR_BASE_SPEED, (int8_t)CAR_BASE_SPEED);
}

/**
 * @brief 获取上一次 PID 输出的 PWM 值 (供 VOFA+ 调试)
 */
float Motor_GetLastOutL(void) { return last_out_l; }
float Motor_GetLastOutR(void) { return last_out_r; }


/* ===================================================================== *
 *  纯霍尔编码器里程计转弯  Motor_TurnByEncoder
 * ===================================================================== */

/**
 * @brief 纯编码器里程计转弯 (粗控制: 开环差速 + 脉冲计数)
 *
 * 左右轮反向旋转, 实时累加编码器脉冲绝对值,
 * 达到目标角度对应的脉冲数后立即停止.
 *
 * @param speed      基础速度 0~100
 * @param direction  TURN_LEFT_DIR(0) / TURN_RIGHT_DIR(1)
 * @param angle_deg  目标角度 (度)
 * @return true=到位, false=超时
 */


bool Motor_TurnByEncoder(int8_t speed, uint8_t direction, float angle_deg)
{
    // /* 目标脉冲数 (方向标定系数修正左右不对称) */
    // float calib = (direction == TURN_LEFT_DIR) ? CAR_TURN_LEFT_CALIBRATION
    //                                             : CAR_TURN_RIGHT_CALIBRATION;
    // float target_total_f = (CAR_WHEEL_BASE_CM * angle_deg
    //                       * (float)ENCODER_PPR_OUTPUT)
    //                    / (180.0f * CAR_WHEEL_DIAMETER_CM) * calib;
    float target_total_f = (CAR_WHEEL_BASE_CM * angle_deg
                          * (float)ENCODER_PPR_OUTPUT)
                       / (180.0f * CAR_WHEEL_DIAMETER_CM);                       
    int32_t target_total = (int32_t)target_total_f;
    uint32_t timeout_ms = 5000U;
    uint32_t elapsed = 0U;

    /* 记录起始脉冲 (用 int32_t 避免 int16_t 溢位) */
    int32_t pulse_start_L = (int32_t)Encoder_GetPulse(ENCODER_L);
    int32_t pulse_start_R = (int32_t)Encoder_GetPulse(ENCODER_R);
    int32_t total_pulses = 0;

    /* 设置旋转方向 */
    if (direction == TURN_LEFT_DIR) {
        Motor_SetRaw(-speed, speed);
    } else {
        Motor_SetRaw(speed, -speed);
    }

    /* 粗旋转: 跑足目标脉冲或超时 */
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


