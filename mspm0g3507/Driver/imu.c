

/* IMU 串口接收与加速度解析。 */
#include "imu.h"
#include "Board/hw_config.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define IMU_RAW_TO_MS2             0.00478515625f

#define IMU_RING_SIZE              1024U
#define IMU_RING_MASK              (IMU_RING_SIZE - 1U)

#define IMU_FRAME_DATA_LEN         8U

typedef enum {
    IMU_PARSE_SYNC,       
    IMU_PARSE_TAG,        
    IMU_PARSE_DATA,       
} imu_parse_state_t;

static volatile uint8_t g_rx_ring[IMU_RING_SIZE];
static volatile uint8_t g_rx_head;       
static volatile uint8_t g_rx_tail;       

static imu_parse_state_t g_state     = IMU_PARSE_SYNC;
static uint8_t           g_tag;
static uint8_t           g_data[IMU_FRAME_DATA_LEN];
static uint8_t           g_data_idx;
static uint8_t           g_checksum;

static volatile float  g_accel_x;
static volatile float  g_accel_y;
static volatile float  g_accel_z;
static volatile int16_t g_raw_accel_x;
static volatile int16_t g_raw_accel_y;
static volatile int16_t g_raw_accel_z;

static bool  g_ready;
static volatile bool g_frame_valid;   

static volatile uint32_t g_cnt_51;
static volatile uint32_t g_cnt_53;
static volatile uint32_t g_cnt_other;
static volatile uint8_t  g_last_tag;

/* 接收 IMU 串口数据。 */
void UART3_IRQHandler(void)
{
    
    while (DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST) == false) {
        uint8_t byte = DL_UART_Main_receiveData(UART_IMU_INST);
        uint8_t next  = (g_rx_head + 1U) & IMU_RING_MASK;

        
        if (next == g_rx_tail) {
            g_rx_tail = (g_rx_tail + 1U) & IMU_RING_MASK;
        }
        g_rx_ring[g_rx_head] = byte;
        g_rx_head = next;
    }
}

/* 读取一字节缓存数据。 */
static bool ring_get(uint8_t *byte)
{
    if (g_rx_head == g_rx_tail) {
        return false;  
    }
    *byte = g_rx_ring[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1U) & IMU_RING_MASK;
    return true;
}

/* 解析一帧 IMU 数据。 */
static void imu_parse_frame(void)
{
    int16_t raw_ax, raw_ay, raw_az;

    
    raw_ax = (int16_t)(((uint16_t)g_data[1] << 8) | (uint16_t)g_data[0]);
    raw_ay = (int16_t)(((uint16_t)g_data[3] << 8) | (uint16_t)g_data[2]);
    raw_az = (int16_t)(((uint16_t)g_data[5] << 8) | (uint16_t)g_data[4]);

    
    g_raw_accel_x = raw_ax;
    g_raw_accel_y = raw_ay;
    g_raw_accel_z = raw_az;

    
    g_accel_x = IMU_RAW_TO_MS2 * (float)raw_ax;
    g_accel_y = IMU_RAW_TO_MS2 * (float)raw_ay;
    g_accel_z = IMU_RAW_TO_MS2 * (float)raw_az;
    g_frame_valid = true;
    g_cnt_51++;
}

/* 解析缓存中的 IMU 数据。 */
static void imu_parse_stream(void)
{
    uint8_t byte;

    while (ring_get(&byte)) {
        switch (g_state) {

        case IMU_PARSE_SYNC:
            if (byte == 0x55U) {
                g_state    = IMU_PARSE_TAG;
                g_checksum = 0x55U;          
            }
            break;

        case IMU_PARSE_TAG:
            g_tag       = byte;
            g_checksum += byte;               
            g_data_idx  = 0U;

            g_last_tag = byte;
            if (byte == 0x51U) {
                
                g_state = IMU_PARSE_DATA;
            } else {
                
                if (byte == 0x53U) {
                    g_cnt_53++;
                } else {
                    g_cnt_other++;
                }
                g_state = IMU_PARSE_SYNC;
                continue;
            }
            break;

        case IMU_PARSE_DATA:
            if (g_data_idx < IMU_FRAME_DATA_LEN) {
                
                g_data[g_data_idx] = byte;
                g_checksum += byte;
                g_data_idx++;
            } else {
                
                if ((g_checksum & 0xFFU) == byte) {
                    
                    imu_parse_frame();
                }
                
                g_state = IMU_PARSE_SYNC;
            }
            break;
        }
    }
}

/* 初始化 IMU 通信。 */
void IMU_Init(void)
{
    
    DL_UART_Main_enableInterrupt(UART_IMU_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);

    
    g_accel_x     = 0.0f;
    g_accel_y     = 0.0f;
    g_accel_z     = 0.0f;
    g_raw_accel_x = 0;
    g_raw_accel_y = 0;
    g_raw_accel_z = 0;
    g_frame_valid = false;

    
    {
        uint32_t timeout = 4000000U;  
        while (!g_frame_valid && timeout > 0U) {
            imu_parse_stream();
            timeout--;
        }
    }

    g_ready = true;
}

/* 读取三轴加速度。 */
void IMU_ReadAccel(float *ax, float *ay, float *az)
{
    
    imu_parse_stream();

    if (ax != NULL) {
        *ax = g_accel_x;
    }
    if (ay != NULL) {
        *ay = g_accel_y;
    }
    if (az != NULL) {
        *az = g_accel_z;
    }
}

/* 读取 Y 轴加速度。 */
float IMU_ReadAccelY(void)
{
    imu_parse_stream();
    return g_accel_y;
}

/* 读取原始三轴加速度。 */
void IMU_ReadRawAccel(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az)
{
    
    imu_parse_stream();

    if (raw_ax != NULL) { *raw_ax = g_raw_accel_x; }
    if (raw_ay != NULL) { *raw_ay = g_raw_accel_y; }
    if (raw_az != NULL) { *raw_az = g_raw_accel_z; }
}

void IMU_GetFrameStats(uint32_t *cnt_51, uint32_t *cnt_53,
                       uint32_t *cnt_other, uint8_t *last_tag)
{
    if (cnt_51   != NULL) { *cnt_51   = g_cnt_51; }
    if (cnt_53   != NULL) { *cnt_53   = g_cnt_53; }
    if (cnt_other != NULL) { *cnt_other = g_cnt_other; }
    if (last_tag != NULL) { *last_tag = g_last_tag; }
}
