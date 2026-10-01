#include "app_arm6.h"
#include "app_uart_servo.h"
#include "app.h"
#include "cmsis_os2.h"


// 中间位置值
uint16_t g_arm_median_value[] = {MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};
uint16_t sync_servo[] = {MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};



static int Convert_Angle_To_Position(uint8_t id, int16_t angle)
{
    int value = -1;
    // if (angle < 0)
    // {
    //     return value;
    // }
    if (id == ARM_ID_1 || id == ARM_ID_2 || id == ARM_ID_3 || id == ARM_ID_4)
    {
        value = (3100 - 900) * (angle - 180) / (0 - 180) + 900;
    }
    if (id == ARM_ID_5)
    {
        value = (3700 - 380) * (angle - 0) / (270 - 0) + 380;
    }
    if (id == ARM_ID_6)
    {
        value = (3100 - 900) * (angle - 0) / (180 - 0) + 900;
    }
    return value;
}

static int Convert_Position_To_Angle(uint8_t id, int16_t pos)
{
    int value = -1;
    if (pos < 0)
    {
        return value;
    }
    if (id == ARM_ID_1 || id == ARM_ID_2 || id == ARM_ID_3 || id == ARM_ID_4)
    {
        // value = (pos - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5;
        value = (pos - 900) / (2200.0) * (-180) + 180.5;
    }
    if (id == ARM_ID_5)
    {
        // value = (270 - 0) * (pos - 380) / (3700 - 380) + 0 + 0.5;
        value = (pos - 380) / 3320.0 * 270 + 0.5;
    }
    if (id == ARM_ID_6)
    {
        // value = (180 - 0) * (pos - 900) / (3100 - 900)  + 0 + 0.5;
        value = (pos - 900) / 2200.0 * 180  + 0.5;
    }
    return value;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

/* 设置同步写的缓存值 */
void Arm_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6)
{


    for (int i = 0; i < 6; i++)
    {
        if (sync_servo[i] > ARM_MAX_VALUE)
        {
            sync_servo[i] = MEDIAN_VALUE;
        }
        else if (sync_servo[i] < ARM_MIN_VALUE)
        {
            sync_servo[i] = MEDIAN_VALUE;
        }
    }
}

/* 同时向多个舵机写入不同的参数 */
void Arm_Sync_Write(uint16_t sync_time)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = 0xfe;
    uint8_t len = 0x22; /* 数据长度，需要根据实际ID个数修改：len= sizeof(data)-4*/
    uint8_t cmd = 0x83;
    uint8_t addr = 0x2a;
    uint8_t data_len = 0x04; /* 实际单个写入舵机的字节数，不需要修改 */

    uint8_t ctrl_id[MAX_SERVO_NUM] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    uint8_t pos_n_H[MAX_SERVO_NUM] = {0};
    uint8_t pos_n_L[MAX_SERVO_NUM] = {0};

    uint8_t pos_sum = 0;
    uint8_t time_H = (sync_time >> 8) & 0xff;
    uint8_t time_L = sync_time & 0xff;

    for (uint8_t i = 0; i < MAX_SERVO_NUM; i++)
    {
        pos_n_H[i] = (sync_servo[i] >> 8) & 0xff;
        pos_n_L[i] = sync_servo[i] & 0xff;
        pos_sum = (pos_sum + ctrl_id[i] + pos_n_H[i] + pos_n_L[i] + time_H + time_L) & 0xff;
    }

    uint8_t checknum = (~(s_id + len + cmd + addr + data_len + pos_sum)) & 0xff;

    uint8_t data[] = {head1, head2, s_id, len, cmd, addr, data_len,
                      ctrl_id[0], pos_n_H[0], pos_n_L[0], time_H, time_L,
                      ctrl_id[1], pos_n_H[1], pos_n_L[1], time_H, time_L,
                      ctrl_id[2], pos_n_H[2], pos_n_L[2], time_H, time_L,
                      ctrl_id[3], pos_n_H[3], pos_n_L[3], time_H, time_L,
                      ctrl_id[4], pos_n_H[4], pos_n_L[4], time_H, time_L,
                      ctrl_id[5], pos_n_H[5], pos_n_L[5], time_H, time_L,
                      checknum};
    UartServo_Send_Data(data, sizeof(data));
}

/* 控制机械臂关节角度 */
void Arm_Set_Angle(uint8_t id, int16_t angle, int16_t runtime)
{
    int16_t value = Convert_Angle_To_Position(id, angle);
    // printf("pos:%d\n", value);
    if (value < 0) return;

    UartServo_Set_Position(id, value, runtime);
}


/* 同时控制机械臂六个关节角度 */
void Arm_Set_Angle6(int16_t a1, int16_t a2, int16_t a3, int16_t a4, int16_t a5, int16_t a6, int16_t runtime)
{
    int16_t value1 = Convert_Angle_To_Position(ARM_ID_1, a1);
    int16_t value2 = Convert_Angle_To_Position(ARM_ID_2, a2);
    int16_t value3 = Convert_Angle_To_Position(ARM_ID_3, a3);
    int16_t value4 = Convert_Angle_To_Position(ARM_ID_4, a4);
    int16_t value5 = Convert_Angle_To_Position(ARM_ID_5, a5);
    int16_t value6 = Convert_Angle_To_Position(ARM_ID_6, a6);
    Arm_Set_Snyc_Buffer(value1, value2, value3, value4, value5, value6);
    Arm_Sync_Write(runtime);
}



/* 读取舵机当前角度 */
int16_t Arm_Get_Angle(uint8_t id)
{
    int16_t angle = 0;
    int16_t value = UartServo_Get_Position(id);

    angle = Convert_Position_To_Angle(id, value);
    return angle;
}


