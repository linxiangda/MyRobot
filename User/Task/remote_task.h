#ifndef REMOTE_TASK_H
#define REMOTE_TASK_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#define REMOTE_MAX_SPEED_MPS       0.3f
#define REMOTE_MAX_YAW_RATE_RAD_S  1.0f
#define REMOTE_MAX_LEG_ANGLE_RAD   0.56199602f
#define REMOTE_DATA_TIMEOUT_MS    300U

/* 对外发布的队列数据，目标值与使能开关相互独立。
 * 断线或输入无效时，速度与偏航角速度归零，腿角保持上一次有效值。
 * 首次有效输入前腿角为零；控制层负责故障处理，并检查队列数据是否超时。 */
typedef struct {
    float speed_set;          /* 右摇杆前后：-0.3～0.3 m/s */
    float yaw_rate_set;       /* 右摇杆左右：-1～1 rad/s */
    float leg_angle_set;      /* 左侧不回中轴：0～0.56199602 rad */
    uint8_t enable;           /* 事件组第 0 位的实际使能状态，仅用于状态展示。 */
    uint8_t online;           /* 无线连接状态，与机器人使能状态不同。 */
    uint32_t update_ms;       /* 最后一次有效无线报文时间，使用 HAL_GetTick()。 */
} RemoteData_t;

/* 主函数创建长度为 1、元素类型为 RemoteData_t 的队列，控制任务使用 xQueuePeek 读取。
 * 遥控任务是唯一发布者，不再提供额外的读取或转换接口。 */
extern QueueHandle_t xRemoteQueue;
void vRemoteTask(void *pvParameters);
#endif
