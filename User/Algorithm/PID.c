#include "PID.h"

/**
 * @brief PID初始化
 * @param PID *pid,float Kp,float Ki,float Kd,float out_limit
 * @return none
 * @attention 积分限幅与输出限幅KI挂钩。积分项最大能单独到输出限幅
 */
void PID_Init(PID_t *pid,float Kp,float Ki,float Kd,float out_limit) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->output = 0;
    
    pid->integral = 0;
    pid->prev_input = 0;
    
    pid->out_limit = out_limit;
    
    pid->integral_limit = pid->out_limit / (Ki+1.0f);//只在初始化与Ki绑定，之后修改
	pid->firstPID = 1;
}
/**
 * @brief PID积分限幅设置
 * @param PID *pid
 * @return none
 * @attention 改变Ki后使用
 */
void PID_Set_integral_limit(PID_t *pid) {
    pid->integral_limit = pid->out_limit / (pid->Ki+1.0f);
}
/**
 * @brief PID设置
 * @param PID *pid,float Kp,float Ki,float Kd
 * @return none
 */
void PID_Set(PID_t *pid,float Kp,float Ki,float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
/**
 * @brief PID计算
 * @param PID *pid, float target, float input, float ts
 * @return output 输出值
 */
float PID_Compute(PID_t *pid, float target, float input, float ts) {
    float error = target - input;

    float p_term = pid->Kp * error;

    // 对测量值微分，首次调用不计算
    float d_term = 0.0f;
    if (pid->firstPID) {
        pid->firstPID = 0;
        pid->prev_input = input;
    } else {
        d_term = -pid->Kd * (input - pid->prev_input) / ts;
        pid->prev_input = input;
    }

    float i_term = pid->Ki * pid->integral;
    float output = p_term + i_term + d_term;

    // 抗积分饱和
    if ((output > pid->out_limit && error > 0) || (output < -pid->out_limit && error < 0)) {
        //输出到上限，积分项不累加
    } else {
        pid->integral += error * ts;

        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }

    // 输出限幅
    if (output > pid->out_limit) output = pid->out_limit;
    if (output < -pid->out_limit) output = -pid->out_limit;

    pid->output = output;
    return output;
}
/**
 * @brief PID复位
 * @param PID *pid
 * @return none
 * @attention none
 */
void PID_Reset(PID_t *pid) {
    pid->integral = 0;
    pid->prev_input = 0;
    pid->output = 0;
	pid->firstPID = 1;
}
