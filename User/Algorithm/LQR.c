#include "LQR.h"

static float Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

void LQR_Compute(LQR_t *LQR)
{
    float output[2] = {0.0f, 0.0f};

    /* u = -Kx */
    for (int motor = 0; motor < 2; motor++) {
        for (int i = 0; i < 6; i++) {
            output[motor] -= LQR->K[motor][i] * LQR->state[i];
        }
    }

    float balance = (output[0] + output[1]) * 0.5f;
    float turn = (output[0] - output[1]) * 0.5f;

    balance = Clamp(balance, -0.3f, 0.3f);

    float abs_balance = balance;
    if (abs_balance < 0.0f) {
        abs_balance = -abs_balance;
    }

    float turn_limit = 0.3f - abs_balance;
    if (turn_limit > 0.06f) {
        turn_limit = 0.06f;
    }

    turn = Clamp(turn, -turn_limit, turn_limit);

    LQR->left_torque = balance + turn;
    LQR->right_torque = balance - turn;
}
