

/* MT6816 编码器接口。 */
#ifndef MT6816_H
#define MT6816_H

#include <stdint.h>
#include <stdbool.h>

#define MT6816_PPR                    1024     
#define MT6816_CPR_4X                 (MT6816_PPR * 4)  
#define MT6816_COUNTS_PER_DEG         ((float)MT6816_CPR_4X / 360.0f)  
#define MT6816_DEG_PER_COUNT          (360.0f / (float)MT6816_CPR_4X)  

void MT6816_Init(void);

void MT6816_Update(void);

int32_t MT6816_GetRawCount(void);

float MT6816_GetAngleDeg(void);

float MT6816_GetVelocityDegS(void);

float MT6816_GetPWMAngleDeg(void);

bool MT6816_IsZDetected(void);

void MT6816_ResetPosition(void);

int32_t MT6816_GetRevolutions(void);

void MT6816_IRQHandler(void);

#endif
