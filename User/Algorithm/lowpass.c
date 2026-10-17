#include "lowpass.h"

#define PI 3.14159265358979323846f

/* 截止频率必须小于fs/2，最好留出明显余量fs/5 */
float LowPass_Update(LowPassFilter_t *filter, float input, float cutoff_hz,float dt){
    float rc;
    float alpha;

    /* 首次采样直接赋值，避免输出从 0 缓慢爬升 */
    if (!filter->initialized){
        filter->output = input;
        filter->initialized = 1;
        return input;
    }

    rc = 1.0f / (2.0f * PI * cutoff_hz); // rc = T = 1/wc = 1/(2*pi*fc)
    alpha = dt / (rc + dt); // alpha = dt / (rc + dt) 即每次采样时，输出对当前误差修正多少的比例
    
    filter->output = filter->output + alpha * (input - filter->output);// 离散差分方程

    return filter->output;
}
