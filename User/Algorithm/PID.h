#ifndef _PID_H
#define _PID_H

#include <stdint.h>

typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float output;       // 输出值
    
    float integral;     // 积分项
    float prev_input;   // 上一次误差
    
    float out_limit;      // 输出上限
    
    float integral_limit; // 积分限幅
	
	uint8_t firstPID;
}PID_t;

void PID_Init(PID_t *pid,float Kp,float Ki,float Kd,float out_limit);
void PID_Set_integral_limit(PID_t *pid); 
void PID_Set(PID_t *pid,float Kp,float Ki,float Kd);
float PID_Compute(PID_t *pid,float target,float input,float ts);
void PID_Reset(PID_t *pid);

#endif
