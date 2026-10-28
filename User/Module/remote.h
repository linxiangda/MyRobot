#ifndef REMOTE_H
#define REMOTE_H
#include <stdint.h>
#define REMOTE_TIMEOUT_MS 300U

/* Internal radio frame, only for remote module/task.
 * Wire: A5 5A, version1, type1, sequence at4..5, channels at6..13,
 * buttons at14..15, enable at16. Total32 bytes, integers little-endian.
 * Channels: rightX, rightY, leftX, leftY; first3=-1000..1000, last=0..1000. */
typedef struct {
    int16_t channel[4];
    uint16_t buttons;
    uint16_t sequence;
    uint8_t enable;
    uint8_t online;
    uint32_t update_ms;
} RemoteFrame_t;

/* Type2 reply: sequence4..5, mV6..7, roll/pitch/yaw/temp8..15 in 0.01deg/C,
 * validity16, RX online17, uptime18..21. RX online is NOT motor enable status. */
typedef struct {
    float voltage_v;
    float roll_deg, pitch_deg, yaw_deg, temperature_c;
    uint8_t valid;
} RemoteTelemetry_t;

/* Called only by remote_task: 0 success, 1 radio error.
 * Update outputs stopped data on errors/timeout.
 * Receive -> wait2ms -> reply -> return to RX; no IRQ. */
uint8_t Remote_Init(void);
uint8_t Remote_Update(RemoteFrame_t *frame, const RemoteTelemetry_t *telemetry);
#endif
