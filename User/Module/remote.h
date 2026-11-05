#ifndef REMOTE_H
#define REMOTE_H
#include <stdint.h>
#define REMOTE_TIMEOUT_MS 300U
#define REMOTE_PROTOCOL_VERSION 2U
#define REMOTE_TYPE_CONTROL 1U
#define REMOTE_TYPE_STATUS 2U
#define REMOTE_TYPE_REGISTER 3U
#define REMOTE_TYPE_REGISTER_ACK 4U

/* V2 协议内部数据，仅供遥控模块和任务使用；线上整数采用小端字节序。
 * 控制包字节：通道 6～13，按键 14～15，目标使能 16，开关指令有效标志 17，
 * 开关指令序号 18～19，输入有效标志 20，会话编号 22～25。
 * 通道依次为右 X、右 Y、左 X、左 Y；前三路为 ±1000，最后一路为 0～1000。 */
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

/* 状态包字节：电压 6～7（mV），横滚、俯仰、偏航、温度 8～15（0.01 度或 ℃），
 * 数据有效标志 16，接收在线标志 17，运行时间 18～21，会话编号 22～25，
 * 开关指令确认序号 26～27，实际使能状态 28，确认序号有效标志 29。 */
typedef struct {
    float voltage_v;
    float roll_deg, pitch_deg, yaw_deg, temperature_c;
    uint8_t valid;
    uint32_t command_session; /* 防止把旧会话的开关指令确认为当前会话指令。 */
    uint16_t command_ack;
    uint8_t command_ack_valid;
    uint8_t enabled;
} RemoteTelemetry_t;

/* 仅由遥控任务调用，frame 和 telemetry 必须为有效指针。
 * 返回 0 表示成功，1 表示无线错误。
 * 收包后先返回任务处理，等待约 2ms，再用下次调用提供的状态回复。
 * 本层不操作 FreeRTOS 或事件组；错误或超时会取消注册并清空控制数据。 */
uint8_t Remote_Init(void);
uint8_t Remote_Update(RemoteFrame_t *frame, const RemoteTelemetry_t *telemetry);
#endif
