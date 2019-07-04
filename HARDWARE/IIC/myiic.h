#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++IIC GPIOCONFIG+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define IIC_GPIO_CLK		RCC_APB2Periph_GPIOB
#define IIC_SCL_PIN         GPIO_Pin_4
#define IIC_SDA_PIN         GPIO_Pin_5
#define IIC_GPIO_PORT       GPIOB
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//IO方向设置
/*模拟IIC端口输出输入定义*/
#define SCL_H         IIC_GPIO_PORT->BSRR = IIC_SCL_PIN
#define SCL_L         IIC_GPIO_PORT->BRR  = IIC_SCL_PIN 
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/  
#define SDA_H         IIC_GPIO_PORT->BSRR = IIC_SDA_PIN
#define SDA_L         IIC_GPIO_PORT->BRR  = IIC_SDA_PIN
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define SCL_read      IIC_GPIO_PORT->IDR  & IIC_SCL_PIN
#define SDA_read      IIC_GPIO_PORT->IDR  & IIC_SDA_PIN
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void I2C_GPIO_Config(void);
u8 IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
void ADXL345_RD_XYZ(s16 *x,s16 *y,s16 *z);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
















