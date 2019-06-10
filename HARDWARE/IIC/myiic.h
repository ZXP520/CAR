#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO方向设置
/*模拟IIC端口输出输入定义*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_4
#define SCL_L         GPIOB->BRR  = GPIO_Pin_4 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_5
#define SDA_L         GPIOB->BRR  = GPIO_Pin_5

#define SCL_read      GPIOB->IDR  & GPIO_Pin_4
#define SDA_read      GPIOB->IDR  & GPIO_Pin_5



void I2C_GPIO_Config(void);
u8 IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
void ADXL345_RD_XYZ(s16 *x,s16 *y,s16 *z);
#endif
















