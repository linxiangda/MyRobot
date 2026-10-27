#include "wheel_control.h"
#include "LQR.h"

LQR_t Wheel_LQR = {
    .balance_out_limit = 0.2f,
    .turn_out_limit = 0.04f,
    .K = {
    {-2.23606798f, -1.92577537f, -4.69436806f,
     -0.53061877f, -0.70710678f, -0.15553182f},

    {-2.23606798f, -1.92577537f, -4.69436806f,
     -0.53061877f,  0.70710678f,  0.15553182f}
    },
};
void Wheel_Control(float state[6], float *left_torque, float *right_torque){
    for(uint8_t i=0;i<6;i++){
        Wheel_LQR.state[i] = state[i];
    }
    
    LQR_Compute(&Wheel_LQR);
    *left_torque = Wheel_LQR.left_torque;
    *right_torque = Wheel_LQR.right_torque;
}
