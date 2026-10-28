#ifndef REMOTE_TASK_H
#define REMOTE_TASK_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#define REMOTE_MAX_SPEED_MPS       0.3f
#define REMOTE_MAX_YAW_RATE_RAD_S  1.0f
#define REMOTE_MAX_LEG_ANGLE_RAD   0.56199602f
#define REMOTE_DATA_TIMEOUT_MS    300U

/* Public queue item. Physical units; enable is a RUN REQUEST, not motor state.
 * Ignore all targets if disabled/offline/stale. Controller owns fault handling. */
typedef struct {
    float speed_mps;          /* right stick forward/back: -0.3..0.3 */
    float yaw_rate_rad_s;     /* right stick left/right: -1..1 */
    float leg_angle_rad;      /* left non-return axis: 0..0.56199602 */
    uint8_t enable;
    uint8_t online;
    uint32_t update_ms;       /* last valid radio packet, HAL_GetTick() */
} RemoteData_t;

/* main: xQueueCreate(1, sizeof(RemoteData_t)); control: xQueuePeek().
 * remote task is the only publisher. No separate read/convert API needed. */
extern QueueHandle_t xRemoteQueue;
void vRemoteTask(void *pvParameters);
#endif
