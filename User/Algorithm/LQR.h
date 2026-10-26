#ifndef LQR_H
#define LQR_H

#include "main.h"

typedef struct {
    float state[6];
    float K[2][6];
    float left_torque;
    float right_torque;
}LQR_t;

void LQR_Compute(LQR_t *LQR);

#endif
