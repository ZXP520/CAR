#include "gy85.h"
#include "myiic.h"

/*******************************************************************************
* Function Name  : ExtractData
* Description    : ???????
* Input          : None 
* Output         : None
* Return         : ?????? 0???? 1????
****************************************************************************** */
void InitGY85(void) //初始化Mpu6050
{
	 I2C_GPIO_Config();
	
	 //磁力计初始化
	 IIC_Write_One_Byte(HMC5883L_Addr,0x0B,0x01); 
   IIC_Write_One_Byte(HMC5883L_Addr,0x20,0x40);
   IIC_Write_One_Byte(HMC5883L_Addr,0x21,0x01);
   IIC_Write_One_Byte(HMC5883L_Addr,0x09,0x0d); //
	

	 //加速度计初始化
	 IIC_Write_One_Byte(ADXL345_Addr,0x31,0x0B);   //测量范围,正负16g，13位模式
	 IIC_Write_One_Byte(ADXL345_Addr,0x2C,0x1A);   //正常模式100HZ输出
   IIC_Write_One_Byte(ADXL345_Addr,0x2D,0x08);   //选择电源模式   参考pdf24页
   IIC_Write_One_Byte(ADXL345_Addr,0x2E,0x80);   //使能 DATA_READY 中断
	
	 //陀螺仪初始化
	 IIC_Write_One_Byte(ITG3205_Addr,PWR_M, 0x80);   //复位
   IIC_Write_One_Byte(ITG3205_Addr,SMPL,  0x07);    //
   IIC_Write_One_Byte(ITG3205_Addr,DLPF,  0x1E);    //±2000° 低通滤波5hz 采样率1khz
   IIC_Write_One_Byte(ITG3205_Addr,INT_C, 0x00 );  //中断
   IIC_Write_One_Byte(ITG3205_Addr,PWR_M, 0x00);   //默认
	 
}

unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address) //获得16位数据
{
	char H,L;
	H=IIC_Read_One_Byte(SlaveAddr,REG_Address);
	L=IIC_Read_One_Byte(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //合成数据
}

unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address) //获得16位数据
{
	char H,L;
	L=IIC_Read_One_Byte(SlaveAddr,REG_Address);
	H=IIC_Read_One_Byte(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //合成数据
}




