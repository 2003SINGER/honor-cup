#include "app_lidar.h"
#include "string.h"
#include "stdio.h"

#include "app.h"
#include "cmsis_os2.h"
#include "usart.h"

#define RX_BUF_MAX     256


static uint8_t lidar_recv_buf[RX_BUF_MAX] = {'\0'}; // 一包数据的缓存  Cache of a packet of data

uint32_t Lidar_Ranges[LIDAR_DATA_LEN] = {'\0'};

static Lidar_Point_t lidar_points[LIDAR_DATA_LEN];
static Lidar_Data_t lidar_data;

static uint8_t lidar_new_data = 0; // 1：代表接收到数据了 1: The representative has received the data


static void Lidar_Parse_Data(void);
static uint8_t Lidar_Checkout(Lidar_Data_t *msg);
static int Lidar_Get_Distance(uint16_t dis_temp);
static double Lidar_Get_Mid_Angle(double abs_angle, uint8_t index, uint8_t data_len, double s_angle);
static double Lidar_Get_Start_Stop_Angle(uint16_t S_angle);
static double Lidar_Limit_Angle(double angle);
static void Lidar_Save_Angle_360(uint8_t mylength);

static void Lidar_Send(uint8_t* data, uint16_t len)
{
    HAL_UART_Transmit(&huart4, data, len, 0xFFFF);
}


// 雷达开始扫描 Radar begins scanning
void Lidar_Start(void)
{
    // Serial_Send(0xA5);
    // Serial_Send(0x60);
    uint8_t data[] = {0xA5, 0x60};
    Lidar_Send(data, 2);
}

// 雷达停止扫描 Radar stops scanning
void Lidar_Stop(void)
{
    // Serial_Send(0xA5);
    // Serial_Send(0x65);
    uint8_t data[] = {0xA5, 0x65};
    Lidar_Send(data, 2);
}

void Lidar_Increase_Frequency(uint8_t value)
{
    uint8_t data[] = {0xA5, 0x0B};
    if (value > 6) value = 6;
    for (int i = 0; i < value; i++)
    {
        Lidar_Send(data, 2);
        osDelay(1);
    }
}


// 接收雷达数据 Receive radar data
void Lidar_Recv_Data(uint8_t rxtemp)
{
    static uint8_t step = 0;
    static uint16_t si_len = 0;
    static uint8_t si_index = 0;
    switch (step)
    {

    case 0:
        if (rxtemp == Lidar_HeaderLSB)
        {
            step = 1;
            lidar_recv_buf[0] = Lidar_HeaderLSB;
        }
        break;
    case 1:
        if (rxtemp == Lidar_HeaderMSB)
        {
            step = 2;
            lidar_recv_buf[1] = Lidar_HeaderMSB;
        }
        break;
    case 2:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // CT信息
    case 3:
        lidar_recv_buf[step] = rxtemp;
        step++;
        si_len = rxtemp * 3; // S的数量
        // 在此判断数据长度时候是否会大于缓存 When determining whether the data length will be greater than the cache here
        if (si_len + 10 >= RX_BUF_MAX)
        {
            si_len = 0;
            step = 0;
            memset(lidar_recv_buf, 0, sizeof(lidar_recv_buf));
        }
        break;
    case 4:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // 起始角低8位 Starting angle is 8 digits lower
    case 5:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // 起始角高8位 Starting angle height of 8 digits
    case 6:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // 结束角低8位 end angle is 8 digits lower
    case 7:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // 结束角高8位 End angle height of 8 digits
    case 8:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // 校验码低8位 Low 8 digits of verification code
    case 9:
        lidar_recv_buf[step] = rxtemp;
        step++;
        break; // 校验码高8位 High 8 digits of verification code
    case 10:
    {
        lidar_recv_buf[step + si_index] = rxtemp;
        si_index++;

        if (si_index >= si_len)
        {
            Lidar_Parse_Data();
            si_index = 0;
            si_len = 0;
            step = 0;                                // 接收完一包数据
            memset(lidar_recv_buf, 0, sizeof(lidar_recv_buf)); // 清掉
        }
        break;
    }
    }
}

static void Lidar_Parse_Data(void)
{
    Lidar_Data_t *timiplus_msg_p = &lidar_data;
    uint16_t llen = lidar_recv_buf[3] * 3; // 长度 length
    uint8_t si_step = 0;

    // 小端转大端 Small end to large end
    timiplus_msg_p->PH = lidar_recv_buf[1] << 8 | lidar_recv_buf[0];
    timiplus_msg_p->CT = lidar_recv_buf[2];
    timiplus_msg_p->LSN = lidar_recv_buf[3];
    timiplus_msg_p->FSA = lidar_recv_buf[5] << 8 | lidar_recv_buf[4];
    timiplus_msg_p->LSA = lidar_recv_buf[7] << 8 | lidar_recv_buf[6];
    timiplus_msg_p->CS = lidar_recv_buf[9] << 8 | lidar_recv_buf[8];

    for (uint16_t i = 0; i < llen; i += 3)
    {
        timiplus_msg_p->SI[si_step].Intensity = lidar_recv_buf[10 + i] & 0x00FF;                        // 光照强度 Light intensity
        timiplus_msg_p->SI[si_step++].SI_dis = lidar_recv_buf[10 + i + 2] << 8 | lidar_recv_buf[10 + i + 1]; // 距离的有效值 Effective value of distance
    }

    // 异或处理 XOR processing
    if (Lidar_Checkout(timiplus_msg_p) != 0) // 校验码错误 Verification code error
    {
        memset(timiplus_msg_p, 0, sizeof(lidar_data));
        return;
    }
    lidar_new_data = 1; // 一包数据正确 One packet of data is correct
}

void Lidar_Get_Data(Lidar_Data_t *msg)
{

    double start_angle, end_angle, abs_angle;

    for (uint8_t len = 0; len < msg->LSN; len++)
    {                                                            // 距离解算 Distance calculation
        lidar_points[len].dis = Lidar_Get_Distance(msg->SI[len].SI_dis); // 获取距离数据 Obtain distance data
    }

    // 角度解算 Angle calculation
    start_angle = Lidar_Get_Start_Stop_Angle(msg->FSA); // 起始角 Starting angle
    end_angle = Lidar_Get_Start_Stop_Angle(msg->LSA);   // 结束角 End angle

    // 开始角数据存储 Start corner data storage
    lidar_points[0].angle = start_angle;

    // 结束角数据存储 End corner data storage
    lidar_points[msg->LSN - 1].angle = end_angle;

    // 角度限制 Angle limitation
    if (start_angle > end_angle) // 350->10这种情况
    {
        abs_angle = 360 - start_angle + end_angle;
    }
    else // 正常情况 Under normal circumstances
    {
        abs_angle = end_angle - start_angle;
    }

    // 中间角解算 Middle angle calculation
    if (abs_angle != 0)
    {
        for (uint8_t len = 1; len < msg->LSN - 1; len++) // 0是起始角;  msg->LSN-1:最终角
        {
            lidar_points[len].angle = Lidar_Get_Mid_Angle(abs_angle, len, msg->LSN - 1, start_angle);
        }
    }

    // 赋值到有效数据的数组 Assign an array to valid data
    Lidar_Save_Angle_360(msg->LSN);


    // static int aa_count = 0;
    // aa_count++;
    // if (aa_count >= 1)
    // {
    //     aa_count = 0;
    //     printf("angle:%f, %f, %d, %f\n", start_angle, abs_angle, msg->LSN, abs_angle/(msg->LSN-1));
    // }
}

static void Lidar_Update_Points(Lidar_Data_t *msg, Lidar_Point_t* output)
{
    static uint16_t last_index = 0;
    double start_angle, interval_angle;
    int start_distance;
    uint16_t start_index, index;
    start_angle = Lidar_Get_Start_Stop_Angle(msg->FSA); // 起始角 Starting angle
    start_distance = Lidar_Get_Distance(msg->SI[0].SI_dis);
    interval_angle = LIDAR_INTERVAL_POINTS;
    start_index = (uint16_t)(start_angle / interval_angle);
    // start_index = last_index + 1;
    start_index = start_index % LIDAR_DATA_LEN;
    if (msg->LSN <= 1)
    {
        last_index = start_index;
        output[0].dis = start_distance;
        output[0].angle = start_angle;
        for (int i = 0; i < start_index; i++)
        {
            output[i].dis = start_distance;
            output[i].angle = start_angle;
            Lidar_Ranges[i] = output[i].dis;
        }
        // printf("start angle:%d, %f\n", start_index, start_angle);
        return;
    }
    if (start_index > (last_index+1))
    {
        uint16_t offset_count = start_index-last_index-1;
        for (int i = 0; i < offset_count; i++)
        {
            output[last_index+1+i].dis = start_distance;
            output[last_index+1+i].angle = start_angle;
            Lidar_Ranges[last_index+1+i] = output[last_index+1+i].dis;
        }
        // printf("index error:%d, %d\n", last_index, start_index);
    }
    output[start_index].dis = start_distance;
    output[start_index].angle = start_angle;
    Lidar_Ranges[start_index] = output[start_index].dis;
    for (uint8_t i = 1; i < msg->LSN; i++)
    {
        index = (start_index + i) % LIDAR_DATA_LEN;
        output[index].dis = Lidar_Get_Distance(msg->SI[i].SI_dis);
        output[index].angle = start_angle + interval_angle * i;
        Lidar_Ranges[index] = output[index].dis;
    }
    last_index = index;
}

static uint8_t Lidar_Checkout(Lidar_Data_t *msg)
{
    uint16_t result = 0;
    uint16_t second = (msg->LSN << 8) | msg->CT; // CT和LSN的合体，LSN在前 Combination of CT and LSN, with LSN before
    uint8_t len_temp = msg->LSN;
    result = msg->PH ^ second ^ msg->FSA ^ msg->LSA;

    do
    {
        result ^= msg->SI[len_temp - 1].Intensity; // 光照强度 Light intensity
        result ^= msg->SI[len_temp - 1].SI_dis;    // 距离值 Distance value
        len_temp--;
    } while (len_temp);

    if (result != msg->CS)
    {
        return 1; // 校验码错误 Verification code error
    }

    return 0;
}

// 获取距离 Get distance
static int Lidar_Get_Distance(uint16_t dis_temp)
{
    return ((dis_temp >> 8) << 6) | ((dis_temp & 0x00FF) >> 2); // 高8位*64 + 低8位去掉最后两位即为距离 High 8 bits * 64+Low 8 bits. Removing the last two bits is the distance
}

// 中间角度解算 Intermediate angle calculation
static double Lidar_Get_Mid_Angle(double abs_angle, uint8_t index, uint8_t data_len, double s_angle)
{
    double angle_temp = abs_angle / data_len * index;
    angle_temp += s_angle;
    return Lidar_Limit_Angle(angle_temp);
}

// 起始角、结束角解算 Starting angle and ending angle calculation
static double Lidar_Get_Start_Stop_Angle(uint16_t S_angle)
{
    // 1级解算 Level 1 solution
    double angle_temp = (double)((S_angle >> 1) / 64.0);
    return angle_temp;
}

// 判断角是否符合360°的范围 Determine whether the angle falls within the range of 360 °
static double Lidar_Limit_Angle(double angle)
{
    if (angle > 360)
    {
        return angle - 360;
    }

    if (angle < 0)
    {
        return 360 + angle;
    }

    return angle;
}

// static void Lidar_Save_Angles(double start_angle, double end_angle, uint8_t si_len)
// {
//     static uint16_t dis_max = 12000;
//     uint16_t Tminidis_index = 0;
//     for (uint8_t index = 0; index < si_len; index++)
//     {
//         if (lidar_points[index].angle > 360 || lidar_points[index].angle < 0) // 角度不合法 Illegal angle
//             continue;
//         Tminidis_index = lidar_points[index].angle; // 角度对应数组下标 Angle corresponding array index

//         Lidar_Ranges[Tminidis_index] = lidar_points[index].dis; // 距离的数据和角度一一对应 Corresponding distance data and angles one by one
//         if (Lidar_Ranges[Tminidis_index] > dis_max)
//         {
//             Lidar_Ranges[Tminidis_index] = dis_max; // 不超过协议距离最大值 Not exceeding the maximum protocol distance
//         }
//     }
// }


// 保存360度的有效数据 Save 360 degree valid data
static void Lidar_Save_Angle_360(uint8_t mylength)
{
    static uint16_t dis_max = 12000;
    uint16_t Tminidis_index = 0;
    for (uint8_t index = 0; index < mylength; index++)
    {
        if (lidar_points[index].angle > 360 || lidar_points[index].angle < 0) // 角度不合法 Illegal angle
            continue;
        Tminidis_index = lidar_points[index].angle; // 角度对应数组下标 Angle corresponding array index

        Lidar_Ranges[Tminidis_index] = lidar_points[index].dis; // 距离的数据和角度一一对应 Corresponding distance data and angles one by one
        if (Lidar_Ranges[Tminidis_index] > dis_max)
        {
            Lidar_Ranges[Tminidis_index] = dis_max; // 不超过协议距离最大值 Not exceeding the maximum protocol distance
        }
    }
}

// 给外面使用 解算雷达数据
// Provide radar data for external use
void Lidar_Update_Data(void)
{
    // Lidar_Get_Data(&lidar_data);
    Lidar_Update_Points(&lidar_data, lidar_points);
}


#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

void Lidar_Handle_Task(void)
{
    uint16_t rx_count = 0;
    HAL_UART_Receive_DMA(&huart4, dma_buffer, UART_DMA_BUFFER_SIZE);
    Lidar_Stop();
    osDelay(200);
    Lidar_Start();

    while (1)
    {
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER((&huart4)->hdmarx);
        rx_count = 0;
        while (dma_head != dma_tail)
        {
            Lidar_Recv_Data(dma_buffer[dma_head]);
            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
            if (lidar_new_data) break;
            rx_count++;
            if (rx_count > RX_BUF_MAX) break;
        }
        
        if (lidar_new_data)
        {
            Lidar_Update_Data();
            lidar_new_data = 0;
        }
        osDelay(1);
    }
}


