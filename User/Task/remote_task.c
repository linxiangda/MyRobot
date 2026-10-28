#include "remote_task.h"
#include "remote.h"
#include "imu.h"
#include "main.h"
#include "task.h"
#include <string.h>

#define RAD_TO_DEG 57.2957795131f
extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xVbusQueue;

/* Protocol/channel details stay here; the controller sees physical targets. */
static void MakeQueueData(const RemoteFrame_t *frame, RemoteData_t *data)
{
    memset(data, 0, sizeof(*data));
    data->online = frame->online;
    data->update_ms = frame->update_ms;
    if (!frame->online || !frame->enable ||
        (uint32_t)(HAL_GetTick() - frame->update_ms) >= REMOTE_DATA_TIMEOUT_MS) return;
    data->enable = 1;
    data->speed_mps = frame->channel[1] * 0.001f * REMOTE_MAX_SPEED_MPS;
    data->yaw_rate_rad_s = frame->channel[0] * 0.001f * REMOTE_MAX_YAW_RATE_RAD_S;
    data->leg_angle_rad = frame->channel[3] * 0.001f * REMOTE_MAX_LEG_ANGLE_RAD;
}

static void ReadTelemetry(RemoteTelemetry_t *telemetry)
{
    static IMUData_TypeDef imu;
    memset(telemetry, 0, sizeof(*telemetry));
    if (xVbusQueue != NULL && xQueuePeek(xVbusQueue, &telemetry->voltage_v, 0) == pdTRUE)
        telemetry->valid |= 1U;
    if (xIMUQueue != NULL && xQueuePeek(xIMUQueue, &imu, 0) == pdTRUE &&
        imu.error == 0 && (uint32_t)(HAL_GetTick() - imu.update_ms) <= 100U) {
        telemetry->roll_deg = imu.roll * RAD_TO_DEG;
        telemetry->pitch_deg = imu.pitch * RAD_TO_DEG;
        telemetry->yaw_deg = imu.yaw * RAD_TO_DEG;
        telemetry->temperature_c = imu.temperature;
        telemetry->valid |= 2U;
    }
}

void vRemoteTask(void *pvParameters)
{
    static RemoteFrame_t frame;
    static RemoteData_t data;
    static RemoteTelemetry_t telemetry;
    uint8_t initialized = 0;
    (void)pvParameters;
    configASSERT(xRemoteQueue != NULL);
    if (xRemoteQueue == NULL) { for (;;) vTaskDelay(pdMS_TO_TICKS(1000)); }
    xQueueOverwrite(xRemoteQueue, &data);

    for (;;) {
        if (!initialized) initialized = Remote_Init() == 0;
        if (initialized) {
            ReadTelemetry(&telemetry);
            if (Remote_Update(&frame, &telemetry) != 0) initialized = 0;
        } else memset(&frame, 0, sizeof(frame));
        MakeQueueData(&frame, &data);
        xQueueOverwrite(xRemoteQueue, &data);
        vTaskDelay(pdMS_TO_TICKS(initialized ? 1 : 100));
    }
}
