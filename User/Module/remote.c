#include "remote.h"
#include "nrf_driver.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include <float.h>

/* session 非零表示已注册，online 非零表示已经收到有效控制序号。 */
static RemoteFrame_t state;
static uint8_t initialized;
static uint8_t reply_type;
static uint8_t sending;
static uint32_t reply_ms;
static uint32_t register_ms;
static uint16_t reply_sequence;

static uint16_t ReadU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t ReadI16(const uint8_t *p)
{
    uint16_t value = ReadU16(p);
    return (int16_t)(value < 32768U ? (int32_t)value : (int32_t)value - 65536);
}

static void PutU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static uint32_t ReadU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void PutU32(uint8_t *p, uint32_t value)
{
    for (uint8_t i = 0; i < 4; i++) p[i] = (uint8_t)(value >> (8 * i));
}

static uint8_t Finite(float value)
{
    return value == value && fabsf(value) <= FLT_MAX;
}

static int16_t ScaledAngle(float value)
{
    if (!Finite(value)) return 0;
    value *= 100.0f;
    if (value > 32767.0f) value = 32767.0f;
    if (value < -32768.0f) value = -32768.0f;
    return (int16_t)value;
}

static void DropLink(void)
{
    memset(&state, 0, sizeof(state));
    reply_type = 0;
}

static uint8_t Decode(const uint8_t *packet)
{
    int16_t channels[4];
    uint16_t seq;
    uint32_t session;
    uint32_t now = HAL_GetTick();
    if (packet[0] != 0xA5 || packet[1] != 0x5A ||
        packet[2] != REMOTE_PROTOCOL_VERSION) return 1;
    seq = ReadU16(packet + 4);
    session = ReadU32(packet + 22);
    if (session == 0) return 1;
    if (packet[3] == REMOTE_TYPE_REGISTER) {
        if (session != state.session) {
            DropLink();
            state.session = session;
            register_ms = now;
        }
        /* 同一会话的注册重试只重复回复确认，不重置控制指令。 */
        reply_type = REMOTE_TYPE_REGISTER_ACK;
        reply_sequence = seq;
        reply_ms = now;
        return 0;
    }
    if (packet[3] != REMOTE_TYPE_CONTROL ||
        session != state.session || packet[16] > 1 ||
        packet[17] > 1 || packet[20] > 1) return 1;
    for (uint8_t i = 0; i < 4; i++) {
        channels[i] = ReadI16(packet + 6 + 2 * i);
        if (channels[i] < -1000 || channels[i] > 1000) return 1;
    }
    if (channels[3] < 0) return 1;
    uint16_t delta = (uint16_t)(seq - state.sequence);
    if (state.online && delta >= 32768U) return 1;
    /* 重复报文仍然回复，但不刷新失联计时。 */
    if (!state.online || delta != 0) {
        memcpy(state.channel, channels, sizeof(channels));
        state.buttons = ReadU16(packet + 14);
        state.enable = packet[16];
        state.command_valid = packet[17];
        state.command_sequence = ReadU16(packet + 18);
        state.input_valid = packet[20];
        if (!state.input_valid) {
            memset(state.channel, 0, sizeof(state.channel));
            state.buttons = 0;
        }
        state.sequence = seq;
        state.update_ms = now;
        state.online = 1;
    }
    reply_type = REMOTE_TYPE_STATUS;
    reply_sequence = seq;
    reply_ms = now;
    return 0;
}

static void Encode(uint8_t *packet, const RemoteTelemetry_t *telemetry)
{
    float mv = telemetry->voltage_v * 1000.0f;
    uint32_t now = HAL_GetTick();
    memset(packet, 0, NRF_PACKET_SIZE);
    packet[0] = 0xA5; packet[1] = 0x5A;
    packet[2] = REMOTE_PROTOCOL_VERSION; packet[3] = reply_type;
    PutU16(packet + 4, reply_sequence);
    PutU32(packet + 22, state.session);
    if (reply_type == REMOTE_TYPE_REGISTER_ACK) return;
    packet[16] = telemetry->valid & 3U;
    if (!Finite(mv) || mv < 0.0f) {
        mv = 0;
        packet[16] &= (uint8_t)~1U;
    }
    if (mv > 65535.0f) mv = 65535.0f;
    PutU16(packet + 6, (uint16_t)mv);
    if (!Finite(telemetry->roll_deg) || !Finite(telemetry->pitch_deg) ||
        !Finite(telemetry->yaw_deg) || !Finite(telemetry->temperature_c))
        packet[16] &= (uint8_t)~2U;
    PutU16(packet + 8, (uint16_t)ScaledAngle(telemetry->roll_deg));
    PutU16(packet + 10, (uint16_t)ScaledAngle(telemetry->pitch_deg));
    /* 积分航向可能超过一圈，先回绕到 ±180°，避免按百分之一度编码时被整数范围截断。 */
    float yaw = 0.0f;
    if (Finite(telemetry->yaw_deg)) {
        yaw = fmodf(telemetry->yaw_deg, 360.0f);
        if (yaw > 180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;
    }
    PutU16(packet + 12, (uint16_t)ScaledAngle(yaw));
    PutU16(packet + 14, (uint16_t)ScaledAngle(telemetry->temperature_c));
    packet[17] = state.online;
    PutU32(packet + 18, now);
    packet[28] = telemetry->enabled ? 1 : 0;
    if (telemetry->command_ack_valid && telemetry->command_session == state.session) {
        PutU16(packet + 26, telemetry->command_ack);
        packet[29] = 1;
    }
}

uint8_t Remote_Init(void)
{
    DropLink();
    initialized = sending = 0;
    if (NRF_Init() != NRF_OK) return 1;
    initialized = 1;
    return 0;
}

uint8_t Remote_Update(RemoteFrame_t *frame, const RemoteTelemetry_t *telemetry)
{
    uint8_t packet[NRF_PACKET_SIZE];
    uint8_t result;
    if (!initialized) { memset(frame, 0, sizeof(*frame)); return 1; }
    if (state.session && (uint32_t)(HAL_GetTick() -
        (state.online ? state.update_ms : register_ms)) >= REMOTE_TIMEOUT_MS)
        DropLink();
    if (sending) {
        result = NRF_PollSend();
        if (result == NRF_BUSY) goto done;
        if (result != NRF_OK) goto failed;
        sending = 0;
    }
    /* 硬件接收 FIFO 最多保存三包，每次调用限制读取数量。 */
    for (uint8_t i = 0; i < 3; i++) {
        result = NRF_Receive(packet);
        if (result == NRF_EMPTY) break;
        if (result != NRF_OK) goto failed;
        /* 收到新指令后先返回任务处理，再发送回复，避免积压的开关指令合并丢失。 */
        if (Decode(packet) == 0) goto done;
    }
    if (reply_type && (uint32_t)(HAL_GetTick() - reply_ms) >= 2U) {
        Encode(packet, telemetry);
        if (NRF_Send(packet) != NRF_OK) goto failed;
        sending = 1;
        reply_type = 0;
    }
done:
    *frame = state;
    return 0;
failed:
    DropLink();
    initialized = sending = 0;
    NRF_Stop();
    *frame = state;
    return 1;
}

