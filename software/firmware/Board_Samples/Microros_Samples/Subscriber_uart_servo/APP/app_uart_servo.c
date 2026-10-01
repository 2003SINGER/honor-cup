#include "app_uart_servo.h"
#include "app.h"
#include "usart.h"
#include "cmsis_os2.h"


#define ENABLE_USERVO_DEBUG      0
#define ENABLE_RX_INT            1


static uint8_t g_rx_buff[MAX_RX_SIZE] = {0};
static uint8_t g_rx_flag = 0; 

// 请求数据
static int UartServo_Request_Data(void)
{
    #if ENABLE_RX_INT
    uint32_t last = HAL_GetTick();
    while (g_rx_flag == 0 && HAL_GetTick()-last<RX_TIMEOUT_MS)
    {
        osDelay(1);
    }
    if (g_rx_flag == 0) return HAL_TIMEOUT;
    return HAL_OK;
    #else
    return HAL_UART_Receive(&huart3, (uint8_t *)&g_rx_buff, MAX_RX_SIZE, RX_TIMEOUT_MS);
    #endif
}

static void UartServo_Clean_Rx_Buff(void)
{
    for (int i = 0; i < MAX_RX_SIZE; i++)
    {
        g_rx_buff[i] = 0;
    }
    g_rx_flag = 0;
}


// 发送数据
void UartServo_Send_Data(uint8_t* data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, data, len, 0xFF);
    #if ENABLE_USERVO_DEBUG
    printf("uservo send: ");
    for (int i = 0; i < len; i++)
    {
        printf("0x%02x ", data[i]);
    }
    printf("\n");
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


/* 控制舵机 */
void UartServo_Set_Position(uint8_t id, uint16_t value, uint16_t time)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = id & 0xff;
    uint8_t len = 0x07;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x2a;

    if (value > USERVO_MAX_VALUE)
        value = USERVO_MAX_VALUE;
    else if (value < USERVO_MIN_VALUE)
        value = USERVO_MIN_VALUE;

    uint8_t pos_H = (value >> 8) & 0xff;
    uint8_t pos_L = value & 0xff;

    uint8_t time_H = (time >> 8) & 0xff;
    uint8_t time_L = time & 0xff;

    uint8_t checknum = (~(s_id + len + cmd + addr +
                          pos_H + pos_L + time_H + time_L)) &
                       0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd,
                      addr, pos_H, pos_L, time_H, time_L, checknum};

    UartServo_Send_Data(data, sizeof(data));
}


/* 设置总线舵机的扭矩开关,0为关闭，1为开启*/
void UartServo_Set_Torque(uint8_t enable)
{
    uint8_t on_off = 0x00; /* 扭矩参数，0为关闭，1为开启 */
    if (enable)
    {
        on_off = 0x01;
    }
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = 0xfe; /* 发送广播的ID */
    uint8_t len = 0x04;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x28;

    uint8_t checknum = (~(s_id + len + cmd + addr + on_off)) & 0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd,
                      addr, on_off, checknum};
    UartServo_Send_Data(data, sizeof(data));
}

/* 写入目标ID(1~250) */
void UartServo_Set_ID(uint8_t id)
{
    if ((id >= 1) && (id <= 250))
    {
        uint8_t head1 = 0xff;
        uint8_t head2 = 0xff;
        uint8_t s_id = 0xfe; /* 发送广播的ID */
        uint8_t len = 0x04;
        uint8_t cmd = 0x03;
        uint8_t addr = 0x05;
        uint8_t set_id = id; /* 实际写入的ID */

        uint8_t checknum = (~(s_id + len + cmd + addr + set_id)) & 0xff;
        uint8_t data[] = {head1, head2, s_id, len, cmd,
                          addr, set_id, checknum};
        UartServo_Send_Data(data, sizeof(data));
    }
}

/*****************************读数据*********************************/

static int16_t UartServo_Rx_Parse(uint8_t request_id, uint8_t* rx_data)
{
    uint8_t head1 = rx_data[0];
    uint8_t head2 = rx_data[1];
    uint8_t s_id = rx_data[2];
    uint8_t checknum = 0;
    uint16_t read_value = 0;
    if (head1 != PTO_HEAD1 && head2 != PTO_HEAD2 && request_id != s_id) return -1;
    checknum = (~(rx_data[2] + rx_data[3] + rx_data[4] + rx_data[5] + rx_data[6])) & 0xFF;
    if (checknum == rx_data[7])
    {
        read_value = rx_data[5] << 8 | rx_data[6];
        #if ENABLE_USERVO_DEBUG
        printf("read arm value:%d, %d\n", s_id, read_value);
        #endif
        return read_value;
    }
    return -1;
}

/* 读取总线舵机当前位置 */
int16_t UartServo_Get_Position(uint8_t servo_id)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = servo_id & 0xff;
    uint8_t len = 0x04;
    uint8_t cmd = 0x02;
    uint8_t param_H = 0x38;
    uint8_t param_L = 0x02;
    int16_t servo_pulse = -1;
    uint8_t checknum = (~(s_id + len + cmd + param_H + param_L)) & 0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd, param_H, param_L, checknum};
    UartServo_Clean_Rx_Buff();
    #if ENABLE_RX_INT
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_rx_buff, MAX_RX_SIZE);
    #endif
    UartServo_Send_Data(data, sizeof(data));
    if (UartServo_Request_Data() == HAL_OK)
    {
        servo_pulse = UartServo_Rx_Parse(s_id, g_rx_buff);
    }
    #if ENABLE_USERVO_DEBUG
    printf("g_rx_buff: ");
    for (int i = 0; i < MAX_RX_SIZE; i++)
    {
        printf("0x%02x ", g_rx_buff[i]);
    }
    printf("\n");
    #endif
    return servo_pulse;
}

/* 设置舵机接收标识，数据接收完成时调用 */
void UartServo_Set_Rx_Flag(void)
{
    g_rx_flag = 1;
}
