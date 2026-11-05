#ifndef NRF_DRIVER_H
#define NRF_DRIVER_H
#include <stdint.h>

#define NRF_PACKET_SIZE 32U
#define NRF_RF_CHANNEL 40U
#define NRF_OK 0U
#define NRF_ERROR 1U
#define NRF_EMPTY 2U
#define NRF_BUSY 3U

/* 固定 32 字节报文、1Mbps、2 字节 CRC，关闭自动应答和重发。
 * 主控接收地址：{0x52,0x4F,0x42,0x30,0x31}，按 SPI 写入顺序排列。
 * 主控发送地址：{0x52,0x45,0x4D,0x30,0x31}；遥控端交换收发地址。
 * 所有驱动接口由同一个任务调用。初始化等待上电，发送采用启动后轮询完成的方式。
 * 发送成功只表示本机发出报文，不表示遥控端已经收到。 */
uint8_t NRF_Init(void);
uint8_t NRF_Receive(uint8_t data[NRF_PACKET_SIZE]);
uint8_t NRF_Send(const uint8_t data[NRF_PACKET_SIZE]);
uint8_t NRF_PollSend(void);
void NRF_Stop(void);
#endif
