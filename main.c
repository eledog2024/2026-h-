/**
 * @file    main.c
 * @brief   智能小车竞赛 — 循迹 + 步进电机曲柄球平衡
 *
 * 赛道: 操场形跑道 (黑线白底), 起点=横向黑线
 *
 * 题目:
 *   T2: 循迹一圈 + 计时
 *   T3: 静止球往复 ±5cm
 *   T4: A→B 直道 + 球平衡@中心 (时间控制 ≤8s)
 *   T5: 循迹一圈 + 球平衡@中心 (≤30s)
 *   T6: 循迹一圈 + 球平衡@任意位置 (≤30s)
 *
 * 硬件: MSPM0G3507, K230(UART1), MT6816(PA4-7), 步进电机 PWM+DIR(PA0-1),
 *       灰度8路, 编码器电机×2, TM1637数码管, SG90舵机
 */

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "Board/car_config.h"
#include "Board/hw_config.h"
#include "Driver/motor.h"
#include "Driver/encoder.h"
#include "Driver/gray.h"
#include "Driver/imu.h"
#include "Driver/k230_uart.h"
#include "Driver/seven_seg.h"
#include "balance.h"

/* ===================================================================== *
 *  系统定时 — SysTick 10ms
 * ===================================================================== */

#define SYSTICK_FREQ_HZ        32000000U
#define SYSTICK_PERIOD_10MS    (SYSTICK_FREQ_HZ / 100U)

static volatile bool    g_tick_10ms;
static volatile uint32_t g_sys_time_ms;

void SysTick_Handler(void)
{
    g_tick_10ms = true;
    g_sys_time_ms += 10;
}

/* ===================================================================== *
 *  任务 / 状态
 * ===================================================================== */

typedef enum {
    TASK_2 = 2, TASK_3 = 3, TASK_4 = 4, TASK_5 = 5, TASK_6 = 6,
} TaskId_t;

typedef enum {
    ST_MODE_SELECT,
    ST_MODE_SET_POS,       /* T6: 设置球目标位置 */
    ST_TASK_INIT,
    ST_TASK_RUNNING,
    ST_TASK_DONE,
} AppState_t;

typedef enum {
    LP_WAIT_LEAVE, LP_FOLLOWING, LP_DETECTED_FINISH, LP_STOPPED,
} LapPhase_t;

/* ===================================================================== *
 *  全局变量
 * ===================================================================== */

static AppState_t   g_app_state      = ST_MODE_SELECT;
static TaskId_t     g_task_id        = TASK_2;
static uint32_t     g_task_start_ms;
static uint32_t     g_task_elapsed_ms;
static int16_t      g_ball_target_mm = 100;   /* 中心 O=125mm, 偏移=100 表示中心 */

/* 循迹 */
static int16_t      g_gray_pos        = 80;
static LapPhase_t   g_lap_phase       = LP_WAIT_LEAVE;
static int32_t      g_pulse_at_leave;
static uint8_t      g_finish_debounce;

/* 检测阈值 */
#define FINISH_MIN_PULSES      300
#define FINISH_DEBOUNCE_CNT      3
#define T4_RUN_TIME_MS         5000U
#define T4_TIMEOUT_MS          8000U
#define LAP_TIMEOUT_MS         30000U

/* ===================================================================== *
 *  按键 S2 (PB21)
 * ===================================================================== */

static bool S2_IsPressed(void)
{
    return (DL_GPIO_readPins(GPIO_BUTTON_PORT,
                             GPIO_BUTTON_PIN_S2_PIN) == 0U);
}

static bool S2_WaitLongPress(void)
{
    uint32_t start = g_sys_time_ms;
    while (S2_IsPressed()) {
        if ((g_sys_time_ms - start) >= 800) {
            while (S2_IsPressed()) { /* spin */ }
            return true;
        }
    }
    return false;
}

/* ===================================================================== *
 *  一圈检测
 * ===================================================================== */

static void LapDetect_Update(void)
{
    uint8_t  line_count = Gray_GetLineCount();
    int32_t  pulses = (int32_t)(Encoder_GetPulse(ENCODER_L) >= 0
                   ? Encoder_GetPulse(ENCODER_L) : -Encoder_GetPulse(ENCODER_L))
                   + (int32_t)(Encoder_GetPulse(ENCODER_R) >= 0
                   ? Encoder_GetPulse(ENCODER_R) : -Encoder_GetPulse(ENCODER_R));

    switch (g_lap_phase) {
    case LP_WAIT_LEAVE:
        if (line_count <= CAR_LEAVE_LINE_COUNT) {
            g_pulse_at_leave  = pulses;
            g_finish_debounce = 0;
            g_lap_phase       = LP_FOLLOWING;
        }
        break;
    case LP_FOLLOWING:
        if (line_count >= CAR_FINISH_LINE_COUNT
            && (pulses - g_pulse_at_leave) > (int32_t)FINISH_MIN_PULSES) {
            g_finish_debounce++;
            if (g_finish_debounce >= FINISH_DEBOUNCE_CNT) {
                g_lap_phase = LP_DETECTED_FINISH;
            }
        } else {
            g_finish_debounce = 0;
        }
        break;
    default: break;
    }
}

static void LapDetect_Reset(void)
{
    g_lap_phase       = LP_WAIT_LEAVE;
    g_pulse_at_leave  = 0;
    g_finish_debounce = 0;
}

/* ===================================================================== *
 *  小车加速度前馈 (IMU Y 轴, m/s² → mm/s²)
 * ===================================================================== */

static float GetCarAccel(void)
{
    return IMU_ReadAccelY() * 1000.0f;
}

/* ===================================================================== *
 *  数码管显示
 * ===================================================================== */

static void SEG_Update(void)
{
    if (g_app_state == ST_TASK_RUNNING || g_app_state == ST_TASK_DONE) {
        uint16_t cs = (uint16_t)(g_task_elapsed_ms / 10);
        if (cs > 9999) cs = 9999;
        SEG_DisplayTime(cs);
    }
}

/* ===================================================================== *
 *  任务初始化
 * ===================================================================== */

static void TaskInit(void)
{
    Motor_Stop();
    StepperCrank_Stop();
    Encoder_UpdateSpeed(10);
    StepperCrank_Init();
    Motor_ResetLinePID();
    LapDetect_Reset();

    g_task_start_ms   = g_sys_time_ms;
    g_task_elapsed_ms = 0;

    switch (g_task_id) {
    case TASK_2:
        Motor_On();
        break;

    case TASK_3:
        StepperCrank_Task3Reset();
        break;

    case TASK_4:
    case TASK_5:
        g_ball_target_mm = 100;           /* 中心 */
        StepperCrank_SetTarget(g_ball_target_mm);
        Motor_On();
        break;

    case TASK_6:
        /* 目标位置已在 ST_MODE_SET_POS 中设置 */
        StepperCrank_SetTarget(g_ball_target_mm);
        Motor_On();
        break;
    }
}

/* ===================================================================== *
 *  任务执行 (每 10ms)
 * ===================================================================== */

static void TaskRun(void)
{
    g_task_elapsed_ms = g_sys_time_ms - g_task_start_ms;

    /* ── 球平衡 (T3-T6) ── */
    if (g_task_id >= TASK_3) {
        if (g_task_id == TASK_3) {
            StepperCrank_Task3Control();
        } else {
            StepperCrank_Control(GetCarAccel());
        }
    }

    /* ── TASK 2: 循迹一圈 ── */
    if (g_task_id == TASK_2) {
        if (g_lap_phase == LP_DETECTED_FINISH || g_lap_phase == LP_STOPPED) {
            Motor_Stop();
        } else {
            LapDetect_Update();
            Motor_LineFollow(g_gray_pos, (int16_t)CAR_BASE_SPEED);
        }
    }

    /* ── TASK 4: A→B 直道 + 球平衡@中心 ── */
    else if (g_task_id == TASK_4) {
        if (g_task_elapsed_ms >= T4_TIMEOUT_MS
            || g_task_elapsed_ms >= T4_RUN_TIME_MS) {
            Motor_Stop();
            g_lap_phase = LP_STOPPED;
        } else {
            Motor_LineFollow(g_gray_pos, (int16_t)CAR_BASE_SPEED);
        }
    }

    /* ── TASK 5/6: 一圈 + 球平衡 ── */
    else if (g_task_id == TASK_5 || g_task_id == TASK_6) {
        if (g_task_elapsed_ms >= LAP_TIMEOUT_MS) {
            Motor_Stop();
            g_lap_phase = LP_STOPPED;
        } else if (g_lap_phase == LP_DETECTED_FINISH) {
            Motor_Stop();
            g_lap_phase = LP_STOPPED;
        } else if (g_lap_phase != LP_STOPPED) {
            LapDetect_Update();
            Motor_LineFollow(g_gray_pos, (int16_t)CAR_BASE_SPEED);
        }
    }
}

/* ===================================================================== *
 *  任务完成 / 清理
 * ===================================================================== */

static bool IsTaskDone(void)
{
    if (g_task_id == TASK_3) {
        /* Task3Control 内部状态机判定 */
        return false;  /* 完成判定在 Task3Control 内, 外部不干预 */
    }
    return (g_lap_phase == LP_STOPPED);
}

static void TaskCleanup(void)
{
    Motor_Stop();
    Motor_Off();
    StepperCrank_Stop();
}

/* ===================================================================== *
 *  主函数
 * ===================================================================== */

int main(void)
{
    /* ── 硬件初始化 ── */
    SYSCFG_DL_init();
    SysTick_Config(SYSTICK_PERIOD_10MS);

    Motor_Init();
    Encoder_Init();
    Gray_Init();
    K230_UART_Init();
    IMU_Init();
    SEG_Init();
    SEG_DisplayNum((uint16_t)g_task_id);

    /* ── 主循环 (10ms 节拍) ── */
    while (1) {
        if (!g_tick_10ms) continue;
        g_tick_10ms = false;

        /* 传感器刷新 */
        Encoder_UpdateSpeed(10);
        g_gray_pos = Gray_GetPosition();

        switch (g_app_state) {

        /* ── 模式选择 (S2 短按切换, 长按确认) ── */
        case ST_MODE_SELECT: {
            static bool prev = false;
            bool now = S2_IsPressed();

            if (now && !prev) {
                if (S2_WaitLongPress()) {
                    if (g_task_id == TASK_6) {
                        g_app_state = ST_MODE_SET_POS;
                        g_ball_target_mm = 100;
                        SEG_DisplaySigned(0);
                    } else {
                        g_app_state = ST_TASK_INIT;
                    }
                    prev = false;
                } else {
                    g_task_id = (TaskId_t)((uint8_t)g_task_id + 1);
                    if ((uint8_t)g_task_id > (uint8_t)TASK_6)
                        g_task_id = TASK_2;
                    SEG_DisplayNum((uint16_t)g_task_id);
                }
            }
            prev = now;
            break;
        }

        /* ── T6 位置设置 ── */
        case ST_MODE_SET_POS: {
            static bool prev = false;
            bool now = S2_IsPressed();

            if (now && !prev) {
                if (S2_WaitLongPress()) {
                    g_app_state = ST_TASK_INIT;
                    prev = false;
                } else {
                    int16_t t = g_ball_target_mm + 5;
                    if (t > 200) t = 0;
                    g_ball_target_mm = t;
                    SEG_DisplaySigned(g_ball_target_mm - 100);
                }
            }
            prev = now;
            break;
        }

        /* ── 任务初始化 ── */
        case ST_TASK_INIT:
            TaskInit();
            SEG_DisplayTime(0);
            g_app_state = ST_TASK_RUNNING;
            break;

        /* ── 任务运行 ── */
        case ST_TASK_RUNNING:
            TaskRun();

            if (IsTaskDone()) {
                TaskCleanup();
                g_app_state = ST_TASK_DONE;
            }

            {
                static uint8_t c = 0;
                if (++c >= 20) { c = 0; SEG_Update(); }
            }
            break;

        /* ── 任务完成 → S2 返回 ── */
        case ST_TASK_DONE: {
            static bool prev = false;
            bool now = S2_IsPressed();
            if (now && !prev) {
                g_app_state = ST_MODE_SELECT;
                SEG_DisplayNum((uint16_t)g_task_id);
            }
            prev = now;
            break;
        }
        } /* switch */
    } /* while(1) */
}
