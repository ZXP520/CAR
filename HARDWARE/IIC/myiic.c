/**
  ******************************************************************************
  * @file    myiic.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   模拟IIC的驱动程序
  ******************************************************************************
  * @attention  在os系统中IIC等通信延时需要用阻塞延时
  ******************************************************************************
  */
#include "myiic.h"
/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(IIC_GPIO_CLK, ENABLE);	 

	GPIO_InitStructure.GPIO_Pin =  IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStructure);
	//禁止JLink复用功能
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  IIC_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : IICdelay_us
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void IICdelay_us(u32 nTimer)
{
	u32 i=0;
	for(i=0;i<nTimer;i++)
	{
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
int I2C_Start(void)
{
	SDA_H;
	SCL_H;
	IICdelay_us(2);
	if(!SDA_read)return 0;	//SDA线为低电平则总线忙,退出
	SDA_L;
	IICdelay_us(2);
	if(SDA_read) return 0;	//SDA线为高电平则总线出错,退出
	SDA_L;
	IICdelay_us(2);
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
	SCL_L;
	IICdelay_us(2);
	SDA_L;
	IICdelay_us(2);
	SCL_H;
	IICdelay_us(2);
	SDA_H;
	IICdelay_us(2);
} 

/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	IICdelay_us(2);
	SDA_L;
	IICdelay_us(2);
	SCL_H;
	IICdelay_us(2);
	SCL_L;
	IICdelay_us(2);
}   

/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L;
	IICdelay_us(2);
	SDA_H;
	IICdelay_us(2);
	SCL_H;
	IICdelay_us(2);
	SCL_L;
	IICdelay_us(2);
} 

/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
int I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	IICdelay_us(2);
	SDA_H;			
	IICdelay_us(2);
	SCL_H;
	IICdelay_us(2);
	if(SDA_read)
	{
		SCL_L;
		IICdelay_us(2);
		return 0;
	}
	SCL_L;
	IICdelay_us(2);
	return 1;
}

/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
		SCL_L;
		IICdelay_us(2);
		if(SendByte&0x80)
		SDA_H;  
		else 
		SDA_L;   
		SendByte<<=1;
		IICdelay_us(2);
		SCL_H;
		IICdelay_us(2);
    }
    SCL_L;
}  

/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
		ReceiveByte<<=1;      
		SCL_L;
		IICdelay_us(2);
		SCL_H;
		IICdelay_us(2);	
		if(SDA_read)
		{
		ReceiveByte|=0x01;
		}
    }
    SCL_L;
    return ReceiveByte;
} 

//ZRX          
//单字节写入*******************************************
u8 IIC_Write_One_Byte(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    //delay5ms();
    return 1;
}

//单字节读取*****************************************
unsigned char IIC_Read_One_Byte(unsigned char SlaveAddress,unsigned char REG_Address)
{   
	unsigned char REG_data;     	
	if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

		REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}						      

//读取3个轴的数据
//x,y,z:读取到的数据
void ADXL345_RD_XYZ(s16 *x,s16 *y,s16 *z)
{
	u8 buf[6];
	u8 i;
	I2C_Start();  				 
	I2C_SendByte(0xA6);	//发送写器件指令	 
	I2C_WaitAck();	   
    I2C_SendByte(0x32);   		//发送寄存器地址(数据缓存的起始地址为0X32)
	I2C_WaitAck(); 	 										  		   
 
 	I2C_Start();  	 	   		//重新启动
	I2C_SendByte(0xA6+1);	//发送读器件指令
	I2C_WaitAck();
	for(i=0;i<6;i++)
	{
		if(i==5)buf[i]=I2C_RadeByte(),I2C_NoAck();//读取一个字节,不继续再读,发送NACK  
		else buf[i]=I2C_RadeByte(),I2C_Ack();	//读取一个字节,继续读,发送ACK 
 	}	        	   
    I2C_Stop();					//产生一个停止条件
	*x=(s16)(((u16)buf[1]<<8)+buf[0]); 	    
	*y=(s16)(((u16)buf[3]<<8)+buf[2]); 	    
	*z=(s16)(((u16)buf[5]<<8)+buf[4]); 	   
}












