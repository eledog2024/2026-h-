

/* IMU 加速度接口。 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void IMU_Init(void);

void IMU_ReadAccel(float *ax, float *ay, float *az);

float IMU_ReadAccelY(void);

void IMU_ReadRawAccel(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az);

void IMU_GetFrameStats(uint32_t *cnt_51, uint32_t *cnt_53,
                       uint32_t *cnt_other, uint8_t *last_tag);

#endif
