#include "nrf_driver.h"
#include "nrf_bsp.h"
#include "main.h"
#include <string.h>

#define REG_CONFIG 0x00U
#define REG_STATUS 0x07U
#define CONFIG_RX 0x0FU
#define CONFIG_TX 0x0EU

static const uint8_t rx_address[5] = {0x52,0x4F,0x42,0x30,0x31};
static const uint8_t tx_address[5] = {0x52,0x45,0x4D,0x30,0x31};
static uint8_t ready;
static uint8_t transmitting;
static uint32_t tx_start_ms;

/* Always release CSN, including SPI failures. */
static uint8_t Transaction(uint8_t command, const uint8_t *tx,
                           uint8_t *rx, uint8_t length)
{
    uint8_t discarded;
    uint8_t result = NRF_OK;
    BSP_NRF_SetCSN(0);
    BSP_NRF_DelayUs(1);
    if (BSP_NRF_Transfer(command, &discarded) != 0) result = NRF_ERROR;
    for (uint8_t i = 0; i < length && result == NRF_OK; i++) {
        uint8_t value;
        if (BSP_NRF_Transfer(tx ? tx[i] : 0xFFU, &value) != 0)
            result = NRF_ERROR;
        else if (rx != NULL) rx[i] = value;
    }
    BSP_NRF_SetCSN(1);
    BSP_NRF_DelayUs(1);
    return result;
}

static uint8_t WriteReg(uint8_t reg, uint8_t value)
{
    return Transaction(0x20U | reg, &value, NULL, 1);
}

static uint8_t ReadReg(uint8_t reg, uint8_t *value)
{
    return Transaction(reg, NULL, value, 1);
}

static uint8_t EnterRx(void)
{
    BSP_NRF_SetCE(0);
    if (WriteReg(REG_CONFIG, CONFIG_RX) != NRF_OK) return NRF_ERROR;
    BSP_NRF_SetCE(1);
    BSP_NRF_DelayUs(150);
    return NRF_OK;
}

void NRF_Stop(void)
{
    BSP_NRF_SetCE(0);
    BSP_NRF_SetCSN(1);
    ready = 0;
    transmitting = 0;
}

uint8_t NRF_Init(void)
{
    uint8_t check[5];
    uint8_t value;
    ready = transmitting = 0;
    BSP_NRF_Init();
    /* Caller handles retry waits with FreeRTOS; these are startup waits only. */
    BSP_NRF_DelayMs(20);
    if (WriteReg(REG_CONFIG, 0x0CU) || /* power down, CRC16 */
        WriteReg(0x01, 0x00) ||       /* no automatic ACK */
        WriteReg(0x02, 0x01) ||       /* only RX pipe 0 */
        WriteReg(0x03, 0x03) ||       /* 5-byte addresses */
        WriteReg(0x04, 0x00) ||       /* no automatic retries */
        WriteReg(0x05, NRF_RF_CHANNEL) ||
        WriteReg(0x06, 0x06) ||       /* 1Mbps, 0dBm */
        WriteReg(0x11, NRF_PACKET_SIZE) ||
        WriteReg(0x1C, 0x00) ||       /* fixed payloads */
        WriteReg(0x1D, 0x00) ||
        Transaction(0x2A, rx_address, NULL, 5) ||
        Transaction(0x30, tx_address, NULL, 5) ||
        WriteReg(REG_STATUS, 0x70) ||
        Transaction(0xE1, NULL, NULL, 0) ||
        Transaction(0xE2, NULL, NULL, 0)) goto failed;

    if (Transaction(0x0A, NULL, check, 5) ||
        memcmp(check, rx_address, 5) != 0 ||
        ReadReg(0x05, &value) || value != NRF_RF_CHANNEL ||
        ReadReg(0x11, &value) || value != NRF_PACKET_SIZE ||
        ReadReg(0x01, &value) || value != 0 ||
        ReadReg(0x06, &value) || value != 0x06) goto failed;

    if (WriteReg(REG_CONFIG, CONFIG_RX)) goto failed;
    BSP_NRF_DelayMs(2);
    if (EnterRx()) goto failed;
    ready = 1;
    return NRF_OK;
failed:
    NRF_Stop();
    return NRF_ERROR;
}

uint8_t NRF_Receive(uint8_t data[NRF_PACKET_SIZE])
{
    uint8_t fifo;
    if (!ready || data == NULL) return NRF_ERROR;
    if (transmitting) return NRF_BUSY;
    /* Poll FIFO rather than RX_DR, so queued packets cannot be missed. */
    if (ReadReg(0x17, &fifo) || fifo == 0xFFU) goto failed;
    if (fifo & 0x01U) return NRF_EMPTY;
    if (Transaction(0x61, NULL, data, NRF_PACKET_SIZE) ||
        WriteReg(REG_STATUS, 0x40)) goto failed;
    return NRF_OK;
failed:
    NRF_Stop();
    return NRF_ERROR;
}

uint8_t NRF_Send(const uint8_t data[NRF_PACKET_SIZE])
{
    if (!ready || data == NULL) return NRF_ERROR;
    if (transmitting) return NRF_BUSY;
    BSP_NRF_SetCE(0);
    if (WriteReg(REG_CONFIG, CONFIG_TX) ||
        WriteReg(REG_STATUS, 0x30) ||
        Transaction(0xE1, NULL, NULL, 0) ||
        Transaction(0xA0, data, NULL, NRF_PACKET_SIZE)) goto failed;
    BSP_NRF_SetCE(1);
    BSP_NRF_DelayUs(15);
    BSP_NRF_SetCE(0);
    tx_start_ms = HAL_GetTick();
    transmitting = 1;
    return NRF_OK;
failed:
    NRF_Stop();
    return NRF_ERROR;
}

uint8_t NRF_PollSend(void)
{
    uint8_t status;
    uint8_t result;
    if (!ready || !transmitting) return NRF_ERROR;
    if (ReadReg(REG_STATUS, &status) || status == 0xFFU) goto failed;
    if (!(status & 0x30U) && (uint32_t)(HAL_GetTick() - tx_start_ms) < 10U)
        return NRF_BUSY;
    result = (status & 0x20U) ? NRF_OK : NRF_ERROR;
    if (WriteReg(REG_STATUS, 0x30) ||
        Transaction(0xE1, NULL, NULL, 0) || EnterRx()) goto failed;
    transmitting = 0;
    return result;
failed:
    NRF_Stop();
    return NRF_ERROR;
}
