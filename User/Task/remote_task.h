#ifndef REMOTE_TASK_H
#define REMOTE_TASK_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#define REMOTE_MAX_SPEED_MPS       0.3f
#define REMOTE_MAX_YAW_RATE_RAD_S  1.0f
#define REMOTE_MAX_LEG_ANGLE_RAD   0.56199602f
#define REMOTE_DATA_TIMEOUT_MS    300U

/* Public queue item. Targets are independent of the enable switch.
 * Offline/invalid inputs: zero speed/yaw rate, hold last valid leg angle.
 * Before the first valid input use REMOTE_DEFAULT_LEG_ANGLE_RAD.
 * Controller owns fault handling and must check queue freshness as well. */
typedef struct {
    float speed_set;          /* right stick forward/back: -0.3..0.3 m/s */
    float yaw_rate_set;       /* right stick left/right: -1..1 rad/s */
    float leg_angle_set;      /* left non-return axis: 0..0.56199602 rad */
    uint8_t enable;           /* Actual event-group bit0, informational only. */
    uint8_t online;           /* Radio link, not the robot enable state. */
    uint32_t update_ms;       /* last valid radio packet, HAL_GetTick() */
} RemoteData_t;

/* main: xQueueCreate(1, sizeof(RemoteData_t)); control: xQueuePeek().
 * remote task is the only publisher. No separate read/convert API needed. */
extern QueueHandle_t xRemoteQueue;
void vRemoteTask(void *pvParameters);
#endif
