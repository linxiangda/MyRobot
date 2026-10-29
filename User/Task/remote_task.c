#include "remote_task.h"
#include "remote.h"
#include "imu.h"
#include "can_task.h"
#include "main.h"
#include "task.h"
#include "event_groups.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.2957795131f
extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xVbusQueue;
extern QueueHandle_t xMotorQueue;
extern EventGroupHandle_t xEventGroup;

/* Only this task owns the radio-command acknowledgement. */
static uint32_t command_session;
static uint16_t command_ack;
static uint8_t command_ack_valid;

static uint8_t ReadEnabled(void)
{
    return (xEventGroupGetBits(xEventGroup) & 1U) != 0;
}

static uint8_t CanEnableRobot(void)
{
    static IMUData_TypeDef imu;
    static MotorData_TypeDef motor;
    uint32_t now = HAL_GetTick();
    const EventBits_t ready = (1U << 1) | (1U << 2);
    if ((xEventGroupGetBits(xEventGroup) & ready) != ready ||
        xIMUQueue == NULL || xMotorQueue == NULL ||
        xQueuePeek(xIMUQueue, &imu, 0) != pdTRUE ||
        xQueuePeek(xMotorQueue, &motor, 0) != pdTRUE) return 0;
    /* Match the existing controller's 20ms IMU, 500ms motor, 20deg limits.
     * The <= form also rejects NaN/Inf pitch instead of enabling on them. */
    if (imu.error || (uint32_t)(now - imu.update_ms) > 20U ||
        !(fabsf(imu.pitch - 0.002f) <= 20.0f * 3.1415926f / 180.0f)) return 0;
    for (uint8_t i = 0; i < 4; i++) {
        if ((uint32_t)(now - motor.update_ms[i]) > 500U ||
            !motor.motor_online[i] || !motor.motor_calibrated[i] ||
            motor.motor_calibrating[i] || motor.motor_error[i]) return 0;
    }
    return 1;
}

static void HandleSwitchCommand(const RemoteFrame_t *frame)
{
    if (frame->session != command_session) {
        command_session = frame->session;
        command_ack = 0;
        command_ack_valid = 0;
    }
    if (!frame->online || !frame->command_valid ||
        (uint32_t)(HAL_GetTick() - frame->update_ms) >= REMOTE_DATA_TIMEOUT_MS) return;
    if (command_ack_valid) {
        uint16_t delta = (uint16_t)(frame->command_sequence - command_ack);
        if (delta == 0 || delta >= 32768U) return;
    }
    if (!frame->enable) {
        xEventGroupClearBits(xEventGroup, 1U);
    } else if (frame->input_valid && CanEnableRobot()) {
        xEventGroupSetBits(xEventGroup, 1U);
    }
    /* Rejected ON requests are processed too: retransmitting that command
     * must not automatically enable the robot after the fault clears. */
    command_ack = frame->command_sequence;
    command_ack_valid = 1;
}

/* Protocol/channel details stay here; the controller sees physical targets. */
static void MakeQueueData(const RemoteFrame_t *frame, RemoteData_t *data)
{
    data->speed_set = 0.0f;
    data->yaw_rate_set = 0.0f;
    data->enable = ReadEnabled();
    data->online = frame->online &&
        (uint32_t)(HAL_GetTick() - frame->update_ms) < REMOTE_DATA_TIMEOUT_MS;
    data->update_ms = frame->update_ms;
    /* Preserve data->leg_angle_set on disconnect/invalid input. The run
     * switch is independent: valid targets are published even when disabled. */
    if (!data->online || !frame->input_valid) return;
    data->speed_set = frame->channel[1] * 0.001f * REMOTE_MAX_SPEED_MPS;
    data->yaw_rate_set = frame->channel[0] * 0.001f * REMOTE_MAX_YAW_RATE_RAD_S;
    data->leg_angle_set = frame->channel[3] * 0.001f * REMOTE_MAX_LEG_ANGLE_RAD;
}

static void ReadTelemetry(RemoteTelemetry_t *telemetry)
{
    static IMUData_TypeDef imu;
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->enabled = ReadEnabled();
    telemetry->command_session = command_session;
    telemetry->command_ack = command_ack;
    telemetry->command_ack_valid = command_ack_valid;
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
    configASSERT(xEventGroup != NULL);
    if (xRemoteQueue == NULL || xEventGroup == NULL) {
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    xQueueOverwrite(xRemoteQueue, &data);

    for (;;) {
        if (!initialized) initialized = Remote_Init() == 0;
        if (initialized) {
            ReadTelemetry(&telemetry);
            if (Remote_Update(&frame, &telemetry) != 0) initialized = 0;
        } else memset(&frame, 0, sizeof(frame));
        HandleSwitchCommand(&frame);
        MakeQueueData(&frame, &data);
        xQueueOverwrite(xRemoteQueue, &data);
        vTaskDelay(pdMS_TO_TICKS(initialized ? 1 : 100));
    }
}
