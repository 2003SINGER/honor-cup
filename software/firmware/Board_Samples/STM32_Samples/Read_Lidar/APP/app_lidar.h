#ifndef APP_LIDAR_H_
#define APP_LIDAR_H_


#include "stdint.h"

#define Lidar_HeaderLSB       0xAA
#define Lidar_HeaderMSB       0x55

#define LIDAR_SI_LEN          256    // 一包协议的SI最大长度, 大于40
#define LIDAR_DATA_LEN        666    // 有效数据储存
#define LIDAR_INTERVAL_POINTS (0.541f)// 雷达数据点间距角度



// 雷达的最终信息结构体
typedef struct Lidar_Point
{
    double angle; // 角度
    int dis;      // 距离
} Lidar_Point_t;

// 雷达SI采样有效数据结构体
typedef struct Lidar_Data_SI
{
    uint16_t Intensity; // 光强 协议是8位,因为后续要异或，直接补成16位
    uint16_t SI_dis;    // 没解算前的距离
} Lidar_Data_SI_t;

typedef struct Lidar_Data
{
    uint16_t PH;        // 包头
    uint8_t CT;         // CT信息
    uint8_t LSN;        // SI的长度
    uint16_t FSA;       // 起始角
    uint16_t LSA;       // 结束角
    uint16_t CS;        // 校验码
    Lidar_Data_SI_t SI[LIDAR_SI_LEN]; // 数据有效数组
} Lidar_Data_t;


// 雷达有效数据
extern uint32_t Lidar_Ranges[LIDAR_DATA_LEN];


void Lidar_Start(void);
void Lidar_Stop(void);

void Lidar_Recv_Data(uint8_t rxtemp);
void Lidar_Update_Data(void);
void Lidar_Handle(void);
void Lidar_Init(void);

void Lidar_Increase_Frequency(uint8_t value);


void Lidar_Get_Data(Lidar_Data_t *msg);

#endif
