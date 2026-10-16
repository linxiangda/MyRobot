#ifndef _LOWPASS_H_
#define _LOWPASS_H_
#include <stdint.h>


typedef struct {
    float cutoff_hz;     /* 截止频率，Hz */
    float output;        /* 上次滤波结果 */
    uint8_t initialized;
} LowPassFilter_t;

void LowPass_Init(LowPassFilter_t *filter, float cutoff_hz);
float LowPass_Update(LowPassFilter_t *filter, float input, float dt);

#endif
