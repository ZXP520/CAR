/**
  ******************************************************************************
  * @file    errordetect.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   整机错误检测程序
  ******************************************************************************
  * @attention 
  * 
  ******************************************************************************
  */
#include "errordetect.h"
#include "control.h"	
#include "dealdata.h"
#include "bsp_usart.h"

IMUDATA  ImuData;

/*******************************************************************************
* Function Name  : Init_ErrorDetect
* Description    : 初始化错误标志
* Input          : None
* Output         : None
* Return         : None 
****************************************************************************** */
void Init_ErrorDetect(void)
{
	AllWheel.Erroe_flag.data=0;
}

/*******************************************************************************
* Function Name  : ReportError 如果有错误1S上报
* Description    : 上报错误
* Input          : None
* Output         : None
* Return         : None 
****************************************************************************** */
static void ReportError(void)
{
	s16 Cheksum=0;//校验和
	u8 i=0; 
	TXData.InRxData[0]=DATAHEAD;										//头
	TXData.ChRxData[2]=11;			//帧长度
	TXData.ChRxData[3]=EmergencyStop&0xFF;			//命令   小端模式先低后高
	TXData.ChRxData[4]=(EmergencyStop>>8)&0xFF;
	TXData.ChRxData[5]=0;
	TXData.ChRxData[6]=2;					//数据个数	
	//小端模式，先发高位
	for(i=0;i<2;i++)
	{
		TXData.ChRxData[7+i]=TempTxData.ChTempData[i];
	}
	
	//计算校验值
	for(i=0;i<TXData.ChRxData[2]-2;i++)
	{
		Cheksum+=TXData.ChRxData[i];
	}
	TXData.ChRxData[TXData.ChRxData[2]-2]=Cheksum&0xFF; //校验和
	TXData.ChRxData[TXData.ChRxData[2]-1]=(Cheksum>>8)&0xFF;
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,TXData.ChRxData[2]);
}

/*******************************************************************************
* Function Name  : ErrorDetect  100MS进入一次
* Description    : 错误检测
* Input          : None
* Output         : None
* Return         : None 
****************************************************************************** */
void ErrorDetect(void)
{
	static u8 i=0,Time_Cnt=0,error0=0,error1=0,error2=0,error3=0,error4=0,error5=0,error6=0,error7=0;
	Time_Cnt++;
	
	//通信检测200MS
	if(DealData_Rx.Success_Flag)
	{
		error0=0;
		DealData_Rx.Success_Flag=0;
	}
	else if(error0>2)
	{
		AllWheel.Erroe_flag.bits.bit0=1;
	}
	else
	{
		error0++;
	}
		 
		 
	//轮子故障检测5S
	//THREE
	if(ThreeWheel.AimSpeed-ThreeWheel.NowSpeed>20 || ThreeWheel.AimSpeed-ThreeWheel.NowSpeed<-20)
	{
		error1++;
	}
	else if(error1>50)
	{
		AllWheel.Erroe_flag.bits.bit1=1;
		error1=0;
	}
	else
	{
		error1=0;
	}

	//FOUR
	if(FourWheel.AimSpeed-FourWheel.NowSpeed>20 || FourWheel.AimSpeed-FourWheel.NowSpeed<-20)
	{
		error2++;
	}
	else if(error2>50)
	{
		AllWheel.Erroe_flag.bits.bit2=1;
		error2=0;
	}
	else
	{
		error2=0;
	}

	//LEFT
	if(LeftWheel.AimSpeed-LeftWheel.NowSpeed>20 || LeftWheel.AimSpeed-LeftWheel.NowSpeed<-20)
	{
		error3++;
	}
	else if(error3>50)
	{
		AllWheel.Erroe_flag.bits.bit3=1;
		error3=0;
	}
	else
	{
		error3=0;
	}

	if(RightWheel.AimSpeed-RightWheel.NowSpeed>20 || RightWheel.AimSpeed-RightWheel.NowSpeed<-20)
	{
		error4++;
	}
	else if(error4>50)
	{
		AllWheel.Erroe_flag.bits.bit4=1;
		error4=0;
	}
	else
	{
		error4=0;
	}

	//IMU检测 5S

	//陀螺仪
	if(ImuData.NGYData[0]==ImuData.OGYData[0]&&ImuData.NGYData[1]==ImuData.OGYData[1]&&ImuData.NGYData[2]==ImuData.OGYData[2])
	{
		error5++;
	}
	else if(error5>50)
	{
		AllWheel.Erroe_flag.bits.bit5=1;
		error5=0;
	}
	else
	{
		error5=0;
	}
	for(i=0;i<3;i++){ImuData.OGYData[i]=ImuData.NGYData[i];}

	//加速度计
	if(ImuData.NAccelData[0]==ImuData.OAccelData[0]&&ImuData.NAccelData[1]==ImuData.OAccelData[1]&&ImuData.NAccelData[2]==ImuData.OAccelData[2])
	{
		error6++;
	}
	else if(error6>50)
	{
		AllWheel.Erroe_flag.bits.bit6=1;
		error6=0;
	}
	else
	{
		error6=0;
	}
	for(i=0;i<3;i++){ImuData.OAccelData[i]=ImuData.NAccelData[i];}

	//磁力计
	if(ImuData.NMagnetData[0]==ImuData.OMagnetData[0]&&ImuData.NMagnetData[1]==ImuData.OMagnetData[1]&&ImuData.NMagnetData[2]==ImuData.OMagnetData[2])
	{
		error7++;
	}
	else if(error6>50)
	{
		error7=0;
		AllWheel.Erroe_flag.bits.bit7=1;
	}
	else
	{
		error7=0;
	}
	for(i=0;i<3;i++){ImuData.OMagnetData[i]=ImuData.NMagnetData[i];}

		
	//如果有错误1S
	if(AllWheel.Erroe_flag.data&&Time_Cnt>10)
	{
		Time_Cnt=0;
		TempTxData.InTempData[0]=AllWheel.Erroe_flag.data;
		ReportError();
		AllWheel.Erroe_flag.data=0;
	}
	
}
