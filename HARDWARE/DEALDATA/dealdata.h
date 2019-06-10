#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	DATAHEAD    0xDEED //数据头
#define RXDATALENTH 0x0A //接收数据长度
#define TXDATALENTH 0x0008 //发送数据长度
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//数据接收结构体
typedef struct
{
	_Bool Success_Flag;	//接收成功标志
	_Bool Hardware_Init;	//硬件初始化接收成功标志
	u8    FrameLength;		//帧长度
	u16   CMD;					//数据命令
	u8    DataNum;				//命令个数
	s16   CheckSum;			//校验码
	u8    Respond_Flag; //回应标志 1：回应 0：不回应
	
}DEALDATA_RX;
extern DEALDATA_RX DealData_Rx;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef enum 
{
	WhellDiameter=0,	//轮子直径
	WheelBase,				//轮子轴距
	WhellRollSpeed,   //轮子转速
	WhellSpeed,				//轮子速度
	ReductionRatio,   //轮子减速比
	WhellAcceleration,//轮子加速度
	EncoderLine, 			//编码器线数
	EncoderValue,     //编码器值
	IMUData,					//陀螺仪数据
	UltrasonicData,   //超声波数据
	EmergencyStop,    //急停状态
	VersionNumber,    //版本号
	RemainingBattery, //剩余电量
	HardwareParameter //硬件参数
	
}InquireCMD;//查询命令
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef enum 
{
	SWhellRollSpeed=0x8000,   //轮子转速
	SWhellSpeed,							//轮子速度
	STurningRadius,     			//轮子拐弯半径
	SWhellAcceleration,       //轮子加速度
	SChassisAttitude,  				//底盘姿态
	UploadData=0xF000					//上传数据命令
	
}SetCMD;//设置命令
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern union TEMPDATA{
		u16  UInTempData[15];
    s16  InTempData[15];
    u8 	 ChTempData[30];
}TempTxData,TempRxData;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DealRXData(void);
void SendEncoderAndIMU20Ms(u8 sendflag);
void GetIMU0ffset(void);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
