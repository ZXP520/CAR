#ifndef __ERRORDETECT_H
#define __ERRORDETECT_H	 
#include "sys.h"



typedef struct
{
	//当前
	int16_t NGYData[3];			 //陀螺仪数据
	int16_t NAccelData[3];   //加速度计
	int16_t NMagnetData[3];  //磁力计
	
	//上次
	int16_t OGYData[3];			 //陀螺仪数据
	int16_t OAccelData[3];   //加速度计
	int16_t OMagnetData[3];  //磁力计
	
	//零偏值
	float OffsetGYData[3];
	float OffsetAccelData[3];
	float	OffsetMagnetData[3];  //磁力计
	
	_Bool ZeroFtlag;  //零飘除去标志
	
	float Yaw;//陀螺仪角度值
	
}IMUDATA;

extern IMUDATA  ImuData;
void Init_ErrorDetect(void);
void ErrorDetect(void);
#endif
