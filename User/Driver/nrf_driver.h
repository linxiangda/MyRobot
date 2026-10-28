#ifndef NRF_DRIVER_H
#define NRF_DRIVER_H
#include <stdint.h>

#define NRF_PACKET_SIZE 32U
#define NRF_RF_CHANNEL 40U
#define NRF_OK 0U
#define NRF_ERROR 1U
#define NRF_EMPTY 2U
#define NRF_BUSY 3U

/* Fixed 32-byte payload, 1Mbps, 2-byte CRC, no auto ACK/retry.
 * Robot RX address = {0x52,0x4F,0x42,0x30,0x31} (SPI byte order).
 * Robot TX address = {0x52,0x45,0x4D,0x30,0x31}.
 * Remote swaps these addresses. One task owns all driver calls.
 * Init waits for power-up; Send/PollSend form a nonblocking transmission.
 * TX success means emitted, not acknowledged by the remote.
 */
uint8_t NRF_Init(void);
uint8_t NRF_Receive(uint8_t data[NRF_PACKET_SIZE]);
uint8_t NRF_Send(const uint8_t data[NRF_PACKET_SIZE]);
uint8_t NRF_PollSend(void);
void NRF_Stop(void);
#endif
