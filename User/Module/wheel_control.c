#include "wheel_control.h"
#include "LQR.h"

static const float K[2][6] = {
    {-2.23606798f, -1.92577537f, -4.69436806f,
     -0.53061877f, -0.70710678f, -0.15553182f},

    {-2.23606798f, -1.92577537f, -4.69436806f,
     -0.53061877f,  0.70710678f,  0.15553182f}
};

LQR_t Wheel_LQR;
void Wheel_Control(float state[6], float *left_torque, float *right_torque){
    for(uint8_t i=0;i<6;i++){
        Wheel_LQR.state[i] = state[i];
    }
    for(uint8_t i=0;i<2;i++){
        for(uint8_t j=0;j<6;j++){
            Wheel_LQR.K[i][j] = K[i][j];
        }
    }
    
    LQR_Compute(&Wheel_LQR);
    *left_torque = Wheel_LQR.left_torque;
    *right_torque = Wheel_LQR.right_torque;
}
