#ifndef _LOWPASS_H_
#define _LOWPASS_H_
#include <stdint.h>


typedef struct {
    float output;   
    uint8_t initialized;
} LowPassFilter_t;

float LowPass_Update(LowPassFilter_t *filter, float input, float cutoff_hz,float dt);

#endif
