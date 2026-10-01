#ifndef BSP_IO_I2C_H_
#define BSP_IO_I2C_H_

#include "app.h"

//#define SCL_GPIO_Port   GPIOB
//#define SCL_Pin         GPIO_PIN_10
//#define SDA_GPIO_Port   GPIOB
//#define SDA_Pin         GPIO_PIN_11

// SCL PB10, SDA PB11
#define SDA_IN()    {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}
#define SDA_OUT()   {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;}


#define IIC_SCL(a)  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, a)
#define IIC_SDA(a)  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, a)
#define READ_SDA    HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)


void IIC_Delay(uint8_t nCount);
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);



uint8_t IIC_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t IIC_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif /* BSP_IO_I2C_H_ */
