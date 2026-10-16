#include "can_bsp.h"
#include "fdcan.h"
#include "string.h"
HAL_StatusTypeDef CAN1_Start(void){
    FDCAN_FilterTypeDef filter;

    filter.IdType = FDCAN_STANDARD_ID; //标准ID
    filter.FilterIndex = 0; //滤波器0
    filter.FilterType = FDCAN_FILTER_MASK; //掩码模式：ID1写ID,ID2写掩码
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //报文存入FIFO0
    filter.FilterID1 = 0x100; //ID
    filter.FilterID2 = 0x7FF; //掩码 

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)// 配置匹配ID 执行 FilterConfig(匹配ID的远程帧也接收，需要全局配置那里设)
        return HAL_ERROR;

    if (HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1, 
        FDCAN_REJECT, 
        FDCAN_REJECT, 
        FDCAN_REJECT_REMOTE, 
        FDCAN_REJECT_REMOTE) != HAL_OK)// 不匹配的数据帧去向，远程帧过滤接收还是拒收
        return HAL_ERROR;

    return HAL_FDCAN_Start(&hfdcan1);
}

HAL_StatusTypeDef CAN1_Send_Data(uint32_t id, const uint8_t *data,uint32_t len){
    FDCAN_TxHeaderTypeDef tx;
    uint8_t payload[8] = {0};

    if (id > 0x7FF || len > 8 || data == NULL)
        return HAL_ERROR;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
        return HAL_BUSY;

    memcpy(payload, data, len); //防止数据写入时，越界读取

    tx.Identifier = id; //ID
    tx.IdType = FDCAN_STANDARD_ID; //标准帧
    tx.TxFrameType = FDCAN_DATA_FRAME; //数据帧
    tx.DataLength = len; //发送数据长度
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;//CAN FD的ESI标志，表示发送节点处于“错误主动”状态；不表示当前发送出错。经典CAN不使用此位 
    tx.BitRateSwitch = FDCAN_BRS_OFF; //不切换位速率
    tx.FDFormat = FDCAN_CLASSIC_CAN; //使用经典CAN，也可选用FDCAN
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;//发送后不生成发送事件FIFO记录
    tx.MessageMarker = 0;//软件给报文设置的标记，用于在发送事件中识别报文；当前未启用发送事件，所以不用

    return HAL_FDCAN_AddMessageToTxFifoQ( &hfdcan1, &tx, payload);
}

