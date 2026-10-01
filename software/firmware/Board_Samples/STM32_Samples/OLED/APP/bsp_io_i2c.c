#include <bsp_io_i2c.h>


// Microsecond delay  微秒级延迟
static void Delay_For_Pin(uint8_t nCount)
{
    uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < 10; i++);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// Microsecond delay  微秒级延迟
void IIC_Delay(uint8_t nCount)
{
    Delay_For_Pin(nCount);
}

// Initialize the IIC  初始化IIC
void IIC_Init(void)
{
	IIC_Delay(1);
}

// Generates the IIC initiation signal  产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();
	IIC_SDA(1);
	IIC_SCL(1);
	IIC_Delay(4);
 	IIC_SDA(0);
	IIC_Delay(4);
	IIC_SCL(0);
}

// Generates an IIC stop signal  产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);
 	IIC_Delay(4);
	IIC_SCL(1);
	IIC_SDA(1);
	IIC_Delay(4);
}

// 等待应答信号到来
// 返回值：1，接收应答失败. 0，接收应答成功
// Wait for the answer signal to arrive.
// Return value: 1, receive and reply failed 0, receive and reply succeeded
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();
	IIC_SDA(1);IIC_Delay(1);
	IIC_SCL(1);IIC_Delay(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);
	return 0;
}

// Generate AN ACK reply  产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	IIC_Delay(2);
	IIC_SCL(1);
	IIC_Delay(2);
	IIC_SCL(0);
}
// No ACK response is generated  不产生ACK应答
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	IIC_Delay(2);
	IIC_SCL(1);
	IIC_Delay(2);
	IIC_SCL(0);
}

// IIC发送一个字节，返回从机有无应答，1，有应答，0，无应答
// The IIC sends a byte that returns whether the slave machine answered, 1, yes, 0, no
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	SDA_OUT();
    IIC_SCL(0);
    for(t=0;t<8;t++)
    {
        IIC_SDA((txd&0x80)>>7);
        txd<<=1;
		IIC_Delay(2);
		IIC_SCL(1);
		IIC_Delay(2);
		IIC_SCL(0);
		IIC_Delay(2);
    }
}
// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
// Read 1 byte, ack=1, send ACK, ack=0, send nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0);
        IIC_Delay(2);
		IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;
		IIC_Delay(1);
    }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// IIC写一个字节, 返回值:0,正常, 其他,错误代码
// IIC writes a byte, return value :0, normal, otherwise, error code
uint8_t IIC_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}

// IIC连续写，buf为要写的数据地址。返回值:0,正常，其他,错误代码
// IIC continuous write, buF is the address of the data to be written.  Return value :0, normal, otherwise, error code
uint8_t IIC_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	for (i = 0; i < len; i++)
	{
		IIC_Send_Byte(buf[i]);
		if (IIC_Wait_Ack())
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}

