#ifndef __GY85_H
#define __GY85_H
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			  0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x32
#define	ACCEL_XOUT_L	0x33
#define	ACCEL_YOUT_H	0x34
#define	ACCEL_YOUT_L	0x35
#define	ACCEL_ZOUT_H	0x36
#define	ACCEL_ZOUT_L	0x37
#define	TEMP_OUT_H		0x1B
#define	TEMP_OUT_L		0x1C
#define	GYRO_XOUT_H		0x1D
#define	GYRO_XOUT_L		0x1E	
#define	GYRO_YOUT_H		0x1F
#define	GYRO_YOUT_L		0x20
#define	GYRO_ZOUT_H		0x21
#define	GYRO_ZOUT_L		0x22
#define	PWR_MGMT_1		0x3E	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			0x00	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define WHO	  			0x00
#define	SMPL			0x15
#define DLPF			0x16
#define INT_C			0x17
#define INT_S			0x1A
#define	TMP_H			0x1B
#define	TMP_L			0x1C
#define	GX_H			0x00
#define	GX_L			0x01
#define	GY_H			0x02
#define	GY_L			0x03
#define GZ_H			0x04
#define GZ_L			0x05
#define PWR_M			0x3E
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	HMC5883L_Addr   0x1a	//磁场传感器器件地址
#define	ADXL345_Addr    0xA6	//加速度传感器器件地址
#define	ITG3205_Addr    0xD0	//陀螺仪传感器器件地址
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void 	InitGY85(void);
unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address);
unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address); //获得16位数据
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
