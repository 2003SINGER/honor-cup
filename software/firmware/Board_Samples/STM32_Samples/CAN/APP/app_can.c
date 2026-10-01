#include "app_can.h"
#include "app.h"
#include "fdcan.h"

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;


FDCAN_TxFrame_TypeDef TxFrame = {
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat = FDCAN_CLASSIC_CAN,
    .Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0,
};

void Can_Init(void)
{
    FDCAN_FilterTypeDef FDCAN1_FilterConfig;

    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterConfig.FilterIndex = 0;
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterConfig.FilterID1 = 0x00000000;
    FDCAN1_FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void FDCAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[8])
{
    printf("CAN RX:");
    for (int i = 0; i < 8; i++)
    {
        printf("0x%02x ", Data[i]);
    }
    printf("\n");
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data);

    FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier, FDCAN1_RxFrame.Data);
}



void Can_Test_Send(void)
{
    uint8_t send_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    static uint8_t count = 0;
    count++;
    send_data[0] = count;
    TxFrame.Header.Identifier = 0x000B & 0x7FF;
    for (int i = 0; i < 8; i++)
    {
        TxFrame.Data[i] = send_data[i];
    }
    printf("CAN TX:");
    for (int i = 0; i < 8; i++)
    {
        printf("0x%02x ", send_data[i]);
    }
    printf("\n");
    HAL_FDCAN_AddMessageToTxFifoQ(TxFrame.hcan,&TxFrame.Header,TxFrame.Data);
}
