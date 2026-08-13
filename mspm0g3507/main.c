/* 任务调度与循迹、平衡控制入口。 */
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
#include "Driver/balance.h"

#define SYSTICK_FREQ_HZ        32000000U
#define SYSTICK_PERIOD_10MS    (SYSTICK_FREQ_HZ / 100U)
#define FINISH_MIN_PULSES      300
#define FINISH_DEBOUNCE_CNT    3
#define T4_RUN_TIME_MS         5000U
#define T2_TIMEOUT_MS          20000U
#define LAP_TIMEOUT_MS         30000U

static volatile bool g_tick_10ms;
static volatile uint32_t g_sys_time_ms;

typedef enum { TASK_2 = 2, TASK_3 = 3, TASK_4 = 4, TASK_5 = 5, TASK_6 = 6 } TaskId_t;
typedef enum { ST_MODE_SELECT, ST_MODE_SET_POS, ST_TASK_INIT, ST_TASK_RUNNING, ST_TASK_DONE } AppState_t;
typedef enum { LP_WAIT_LEAVE, LP_FOLLOWING, LP_DETECTED_FINISH, LP_STOPPED } LapPhase_t;

static AppState_t g_app_state = ST_MODE_SELECT;
static TaskId_t g_task_id = TASK_2;
static uint32_t g_task_start_ms;
static uint32_t g_task_elapsed_ms;
static int16_t g_ball_target_mm;
static int16_t g_gray_pos = 80;
static LapPhase_t g_lap_phase = LP_WAIT_LEAVE;
static int32_t g_pulse_at_leave;
static uint8_t g_finish_debounce;

/* 产生 10 ms 调度节拍。 */
void SysTick_Handler(void)
{
    g_tick_10ms = true;
    g_sys_time_ms += 10U;
}

/* 读取 S2 按键状态。 */
static bool S2_IsPressed(void)
{
    return DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_S2_PIN) == 0U;
}

/* 等待并识别长按。 */
static bool S2_WaitLongPress(void)
{
    uint32_t start = g_sys_time_ms;
    while (S2_IsPressed()) {
        if (g_sys_time_ms - start >= 800U) {
            while (S2_IsPressed()) { }
            return true;
        }
    }
    return false;
}

/* 更新循迹圈数检测。 */
static void LapDetect_Update(void)
{
    uint8_t line_count = Gray_GetLineCount();
    int32_t pulses = (Encoder_GetPulse(ENCODER_L) >= 0 ? Encoder_GetPulse(ENCODER_L) : -Encoder_GetPulse(ENCODER_L))
                   + (Encoder_GetPulse(ENCODER_R) >= 0 ? Encoder_GetPulse(ENCODER_R) : -Encoder_GetPulse(ENCODER_R));

    if (g_lap_phase == LP_WAIT_LEAVE && line_count <= CAR_LEAVE_LINE_COUNT) {
        g_pulse_at_leave = pulses;
        g_finish_debounce = 0;
        g_lap_phase = LP_FOLLOWING;
    } else if (g_lap_phase == LP_FOLLOWING) {
        if (line_count >= CAR_FINISH_LINE_COUNT && pulses - g_pulse_at_leave > FINISH_MIN_PULSES) {
            if (++g_finish_debounce >= FINISH_DEBOUNCE_CNT) g_lap_phase = LP_DETECTED_FINISH;
        } else {
            g_finish_debounce = 0;
        }
    }
}

/* 清除圈数检测状态。 */
static void LapDetect_Reset(void)
{
    g_lap_phase = LP_WAIT_LEAVE;
    g_pulse_at_leave = 0;
    g_finish_debounce = 0;
}

/* 获取小车纵向加速度。 */
static float GetCarAccel(void)
{
    return IMU_ReadAccelY() * 1000.0f;
}

/* 刷新任务计时显示。 */
static void SEG_Update(void)
{
    uint16_t cs = (uint16_t)(g_task_elapsed_ms / 10U);
    SEG_DisplayTime(cs > 9999U ? 9999U : cs);
}

/* 初始化当前竞赛任务。 */
static void TaskInit(void)
{
    Motor_Stop();
    StepperCrank_Stop();
    Encoder_UpdateSpeed(10U);
    StepperCrank_Init();
    Motor_ResetLinePID();
    LapDetect_Reset();
    g_task_start_ms = g_sys_time_ms;
    g_task_elapsed_ms = 0U;

    switch (g_task_id) {
    case TASK_2:
        Motor_On();
        break;
    case TASK_3:
        StepperCrank_Task3Reset();
        break;
    case TASK_4:
    case TASK_5:
        g_ball_target_mm = 0;
        StepperCrank_SetTarget(0);
        Motor_On();
        break;
    case TASK_6:
        StepperCrank_SetTarget(g_ball_target_mm);
        Motor_On();
        break;
    default:
        break;
    }
}

/* 执行当前任务周期。 */
static void TaskRun(void)
{
    g_task_elapsed_ms = g_sys_time_ms - g_task_start_ms;

    if (g_task_id == TASK_3) {
        if (StepperCrank_Task3Control()) g_lap_phase = LP_STOPPED;
        return;
    }
    if (g_task_id >= TASK_4) StepperCrank_Control(GetCarAccel());

    if (g_task_id == TASK_2 || g_task_id == TASK_5 || g_task_id == TASK_6) {
        if (g_lap_phase != LP_STOPPED) {
            LapDetect_Update();
            if (g_lap_phase == LP_DETECTED_FINISH
                || g_task_elapsed_ms >= (g_task_id == TASK_2 ? T2_TIMEOUT_MS : LAP_TIMEOUT_MS)) {
                Motor_Stop();
                g_lap_phase = LP_STOPPED;
            } else {
                Motor_LineFollow(g_gray_pos, (int16_t)CAR_BASE_SPEED);
            }
        }
    } else if (g_task_id == TASK_4) {
        if (g_task_elapsed_ms >= T4_RUN_TIME_MS) {
            Motor_Stop();
            g_lap_phase = LP_STOPPED;
        } else {
            Motor_LineFollow(g_gray_pos, (int16_t)CAR_BASE_SPEED);
        }
    }
}

/* 判断当前任务是否结束。 */
static bool IsTaskDone(void)
{
    return g_lap_phase == LP_STOPPED;
}

/* 停止任务相关执行器。 */
static void TaskCleanup(void)
{
    Motor_Stop();
    Motor_Off();
    StepperCrank_Stop();
}

/* 初始化硬件并运行任务状态机。 */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SYSTICK_PERIOD_10MS);
    Motor_Init();
    Encoder_Init();
    Gray_Init();
    K230_UART_Init();
    IMU_Init();
    SEG_Init();
    SEG_DisplayNum((uint16_t)g_task_id);

    while (1) {
        if (!g_tick_10ms) continue;
        g_tick_10ms = false;
        Encoder_UpdateSpeed(10U);
        g_gray_pos = Gray_GetPosition();

        switch (g_app_state) {
        /* 任务选择：短按切换，长按确认。 */
        case ST_MODE_SELECT: {
            static bool prev;
            bool now = S2_IsPressed();
            if (now && !prev) {
                if (S2_WaitLongPress()) {
                    if (g_task_id == TASK_6) {
                        g_ball_target_mm = 0;
                        SEG_DisplaySigned(g_ball_target_mm);
                        g_app_state = ST_MODE_SET_POS;
                    } else {
                        g_app_state = ST_TASK_INIT;
                    }
                    prev = false;
                } else {
                    g_task_id = (TaskId_t)((uint8_t)g_task_id + 1U);
                    if (g_task_id > TASK_6) g_task_id = TASK_2;
                    SEG_DisplayNum((uint16_t)g_task_id);
                }
            }
            prev = now;
            break;
        }
        /* 位置设定：任务 6 设置钢球目标。 */
        case ST_MODE_SET_POS: {
            static bool prev;
            bool now = S2_IsPressed();
            if (now && !prev) {
                if (S2_WaitLongPress()) {
                    g_app_state = ST_TASK_INIT;
                    prev = false;
                } else {
                    g_ball_target_mm += 5;
                    if (g_ball_target_mm > 125) g_ball_target_mm = -125;
                    SEG_DisplaySigned(g_ball_target_mm);
                }
            }
            prev = now;
            break;
        }
        /* 任务初始化：复位并启动选定功能。 */
        case ST_TASK_INIT:
            TaskInit();
            SEG_DisplayTime(0);
            g_app_state = ST_TASK_RUNNING;
            break;
        /* 任务运行：执行控制并刷新计时。 */
        case ST_TASK_RUNNING:
            TaskRun();
            if (IsTaskDone()) {
                TaskCleanup();
                g_app_state = ST_TASK_DONE;
            }
            if ((g_task_elapsed_ms % 200U) == 0U) SEG_Update();
            break;
        /* 任务完成：等待按键返回选择。 */
        case ST_TASK_DONE: {
            static bool prev;
            bool now = S2_IsPressed();
            if (now && !prev) {
                g_app_state = ST_MODE_SELECT;
                SEG_DisplayNum((uint16_t)g_task_id);
            }
            prev = now;
            break;
        }
        default:
            g_app_state = ST_MODE_SELECT;
            break;
        }
    }
}