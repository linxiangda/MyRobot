#ifndef REMOTE_H
#define REMOTE_H
#include <stdint.h>
#define REMOTE_TIMEOUT_MS 300U
#define REMOTE_PROTOCOL_VERSION 2U
#define REMOTE_TYPE_CONTROL 1U
#define REMOTE_TYPE_STATUS 2U
#define REMOTE_TYPE_REGISTER 3U
#define REMOTE_TYPE_REGISTER_ACK 4U

/* Internal V2 frame, used only by remote module/task. All wire integers LE.
 * Control: channels6..13, buttons14..15, enable16, command_valid17,
 * command_sequence18..19, input_valid20, session22..25.
 * Channels: rightX, rightY, leftX, leftY; first3 +/-1000, last 0..1000. */
typedef struct {
    int16_t channel[4];
    uint16_t buttons;
    uint16_t sequence;
    uint16_t command_sequence;
    uint32_t session;
    uint8_t enable;
    uint8_t command_valid;
    uint8_t input_valid;
    uint8_t online;
    uint32_t update_ms;
} RemoteFrame_t;

/* Status: mV6..7, roll/pitch/yaw/temp8..15 in 0.01deg/C, validity16,
 * RX online17, uptime18..21, session22..25, command_ack26..27,
 * actual event-group bit0 at28, command_ack_valid29. */
typedef struct {
    float voltage_v;
    float roll_deg, pitch_deg, yaw_deg, temperature_c;
    uint8_t valid;
    uint32_t command_session; /* Prevent acknowledging an old session's command. */
    uint16_t command_ack;
    uint8_t command_ack_valid;
    uint8_t enabled;
} RemoteTelemetry_t;

/* Called only by remote_task: 0 success, 1 radio error.
 * Receive -> return frame for task to handle -> wait2ms -> reply with the
 * next call's telemetry. No RTOS operations or event-group writes here.
 * Errors/timeout cancel registration and clear the internal control frame. */
uint8_t Remote_Init(void);
uint8_t Remote_Update(RemoteFrame_t *frame, const RemoteTelemetry_t *telemetry);
#endif
