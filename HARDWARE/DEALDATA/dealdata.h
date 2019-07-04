#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	DATAHEAD    0xDEED //数据头
#define RXDATALENTH 0x0A //接收数据长度
#define TXDATALENTH 0x0008 //发送数据长度
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//手柄按键
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15


#define PSB_SQUARE      26

typedef struct
{
	u8  UP;
	u8	RIGHT;
	u8  DOWN;
	u8  LEFT;
	u8  L2;
	u8  R2;
	u8  L1;
	u8	R1;
	u8	GREEN;
	u8 	RED;
	u8	BLUE;
	u8  PINK;
	
	s16 PSS_LY;
	s16 PSS_LX;
	s16 PSS_RY;
	s16 PSS_RX;
	
}PSBKEY;
extern PSBKEY PSBKey;
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
	SPSRawData,								//PS原始数据
	UploadData=0xF000					//上传数据命令
	
}SetCMD;//设置命令
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern union TEMPDATA{
		u16  UInTempData[15];
    s16  InTempData[15];
    u8 	 ChTempData[30];
}TempTxData,TempRxData;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef struct
{
	u32 A0;
	u32 A1;
	u32 A2;
	u32 A3;
	u32 x1;
	u32 y1;
	u32 x2;
	u32 y2;
	u32 x3;
	u32 y3;
	double X;
	double Y;
	u8  StableFlag;//稳定标志
	double AverageX;
	double AverageY;
}UWBDATA;
extern UWBDATA UWBData;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DealRXData(void);
void SendEncoderAndIMU20Ms(u8 sendflag);
void GetIMU0ffset(void);
void DealPSData(void);
void DealUWBData(void);
void calcPhonePosition   (double x1, double y1, double d1,
													double x2, double y2, double d2,
													double x3, double y3, double d3);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
