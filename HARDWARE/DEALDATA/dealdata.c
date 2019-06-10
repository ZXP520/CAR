#include "dealdata.h"
#include "sys.h" 
#include "delay.h"
#include "control.h"
#include "timer.h"
#include "usart.h"	
#include "gy85.h"
#include "Encoder.h"
#include "bsp_usart.h"
#include "include.h"
#include "errordetect.h" 
#include "kalman.h"
#include "myiic.h"

//处理接收到的数据，中断调用
DEALDATA_RX DealData_Rx;
//定义缓存共用体
union TEMPDATA TempTxData ,TempRxData;//发送缓存，接收缓存

/*******************************************************************************
* Function Name  : Respond_To_Ros
* Description    : 返回数据给ROS
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
static void Respond_To_Ros(void)
{
	s16 Cheksum=0;//校验和
	u8 i=0; 
	TXData.InRxData[0]=DATAHEAD;										//头
	TXData.ChRxData[2]=DealData_Rx.FrameLength;			//帧长度
	TXData.ChRxData[3]=DealData_Rx.CMD&0xFF;			//命令   小端模式先低后高
	TXData.ChRxData[4]=(DealData_Rx.CMD>>8)&0xFF;
	TXData.ChRxData[5]=DealData_Rx.Respond_Flag;
	TXData.ChRxData[6]=DealData_Rx.DataNum;					//数据个数	
	//小端模式，先发高位
	for(i=0;i<DealData_Rx.DataNum;i++)
	{
		TXData.ChRxData[7+i]=TempTxData.ChTempData[i];
	}
	
	//计算校验值
	for(i=0;i<DealData_Rx.FrameLength-2;i++)
	{
		Cheksum+=TXData.ChRxData[i];
	}
	TXData.ChRxData[DealData_Rx.FrameLength-2]=Cheksum&0xFF; //校验和
	TXData.ChRxData[DealData_Rx.FrameLength-1]=(Cheksum>>8)&0xFF;
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,DealData_Rx.FrameLength);
}


void GetIMU0ffset(void)
{
	static u8 GetIMUData_cnt=0,GetIMUData_cnt1=0;
	while(GetIMUData_cnt<101)
	{
		if(GetIMUData_cnt==0)
		{
			GetData(ITG3205_Addr,GYRO_XOUT_H);
			GetData(ITG3205_Addr,GYRO_YOUT_H);
		  GetData(ITG3205_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
			GetQMC5883Data(ADXL345_Addr,ACCEL_XOUT_H);
			GetQMC5883Data(ADXL345_Addr,ACCEL_YOUT_H);
			GetQMC5883Data(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			
			GetQMC5883Data(HMC5883L_Addr,GX_H);
			GetQMC5883Data(HMC5883L_Addr,GY_H);
			GetQMC5883Data(HMC5883L_Addr,GZ_H);					//磁力计数据
			}
		else
		{
			ImuData.OffsetGYData[0]+=(float)((int16_t)GetData(ITG3205_Addr,GYRO_XOUT_H))*0.01;
			ImuData.OffsetGYData[1]+=(float)((int16_t)GetData(ITG3205_Addr,GYRO_YOUT_H))*0.01;
			ImuData.OffsetGYData[2]+=(float)((int16_t)GetData(ITG3205_Addr,GYRO_ZOUT_H))*0.01;						//陀螺仪数据
			
			ImuData.OffsetAccelData[0]+=(float)((int16_t)GetQMC5883Data(ADXL345_Addr,ACCEL_XOUT_H))*0.01;
			ImuData.OffsetAccelData[1]+=(float)((int16_t)GetQMC5883Data(ADXL345_Addr,ACCEL_YOUT_H))*0.01;
			ImuData.OffsetAccelData[2]+=(float)((int16_t)GetQMC5883Data(ADXL345_Addr,ACCEL_ZOUT_H))*0.01;					//加速度计数据
			
			ImuData.OffsetMagnetData[0]+=(float)((int16_t)GetQMC5883Data(HMC5883L_Addr,GX_H))*0.01;
			ImuData.OffsetMagnetData[1]+=(float)((int16_t)GetQMC5883Data(HMC5883L_Addr,GY_H))*0.01;
			ImuData.OffsetMagnetData[2]+=(float)((int16_t)GetQMC5883Data(HMC5883L_Addr,GZ_H))*0.01;					//磁力计数据

			delay_ms(5);
		}
		GetIMUData_cnt++;
	}

}

/*******************************************************************************
* Function Name  : DealRXData
* Description    : 每20MS上传编码器陀螺仪的数据
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
void SendEncoderAndIMU20Ms(u8 sendflag)
{
	s16 Cheksum=0;//校验和
	static u8 i=0,Time_Cnt=0; 
	Time_Cnt++;
	GetIMU0ffset();
	//因为IIC读取数据时间长，所以分时取数据
	switch(Time_Cnt)
	{
		case 1:  //0ms
		{
			TXData.InRxData[0]=DATAHEAD;										//头
			TXData.ChRxData[2]=35;													//帧长度
			TXData.ChRxData[3]= UploadData    &0xFF;			  //命令   小端模式先低后高
			TXData.ChRxData[4]=(UploadData>>8)&0xFF;
			TXData.ChRxData[5]=0 ;
			TXData.ChRxData[6]=26;					//数据个数	18+8=26
			TempTxData.InTempData[0]=GetEncoder.V2;
			TempTxData.InTempData[1]=GetEncoder.V3;
			TempTxData.InTempData[2]=GetEncoder.V4;
			TempTxData.InTempData[3]=GetEncoder.V5;
			break;
		}
		case 2:  //5ms
		{
			TempTxData.InTempData[4]=GetData(ITG3205_Addr,GYRO_XOUT_H);
			TempTxData.InTempData[5]=GetData(ITG3205_Addr,GYRO_YOUT_H);
			TempTxData.InTempData[6]=GetData(ITG3205_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			//用于错误检测
			ImuData.NGYData[0]=TempTxData.InTempData[4];
			ImuData.NGYData[1]=TempTxData.InTempData[5];
			ImuData.NGYData[2]=TempTxData.InTempData[6];
			break;
		}
		case 3:  //10ms
		{
			TempTxData.InTempData[7]=GetQMC5883Data(ADXL345_Addr,ACCEL_XOUT_H);
			TempTxData.InTempData[8]=GetQMC5883Data(ADXL345_Addr,ACCEL_YOUT_H);
			TempTxData.InTempData[9]=GetQMC5883Data(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			
			//用于错误检测
			ImuData.NAccelData[0]=TempTxData.InTempData[7];
			ImuData.NAccelData[1]=TempTxData.InTempData[8];
			ImuData.NAccelData[2]=TempTxData.InTempData[9];
			break;
		}
		case 4:  //15ms
		{
			TempTxData.InTempData[10]=GetQMC5883Data(HMC5883L_Addr,GX_H);
			TempTxData.InTempData[11]=GetQMC5883Data(HMC5883L_Addr,GY_H);
			TempTxData.InTempData[12]=GetQMC5883Data(HMC5883L_Addr,GZ_H);					//磁力计数据
			//用于错误检测
			ImuData.NMagnetData[0]=TempTxData.InTempData[10];
			ImuData.NMagnetData[1]=TempTxData.InTempData[11];
			ImuData.NMagnetData[2]=TempTxData.InTempData[12];
			
			break;
		}
		case 5:  //20ms
		{
			//小端模式，先发高位
			for(i=0;i<TXData.ChRxData[6];i++)
			{
				TXData.ChRxData[7+i]=TempTxData.ChTempData[i];
			}
			
			//计算校验值
			for(i=0;i<TXData.ChRxData[2]-2;i++)
			{
				Cheksum+=TXData.ChRxData[i];
			}
			TXData.ChRxData[TXData.ChRxData[2]-2]=Cheksum&0xFF; 				//校验和
			TXData.ChRxData[TXData.ChRxData[2]-1]=(Cheksum>>8)&0xFF;
			//DMA串口发送数据
			if(sendflag)
			{
				USART2_DMA_TX(TXData.ChRxData,TXData.ChRxData[2]);
			}
			//陀螺仪角度计算
			Angle_Calcu1();
			Time_Cnt=0;
			
			
			break;
		}
		default:break;
	}	
}

/*******************************************************************************
* Function Name  : ExtractData
* Description    : 提取接收的数据
* Input          : None 
* Output         : None
* Return         : 数据校验结果 0错误数据 1正确数据
****************************************************************************** */
static u8 ExtractData(void)
{
	u8 i;
	u16 ChekSum=0;
	if(RXData.InRxData[0]==(s16)DATAHEAD )//数据头
	{
		//取得数据特征
		DealData_Rx.FrameLength=RXData.ChRxData[2];   						 //包长度
		DealData_Rx.CMD=(RXData.ChRxData[3] + (RXData.ChRxData[4]<<8));//命令     //先低后高
		DealData_Rx.Respond_Flag=RXData.ChRxData[5];               //响应标志
		DealData_Rx.DataNum=RXData.ChRxData[6];										 //数据个数
		switch(DealData_Rx.DataNum)
		{
			case 1:TempRxData.InTempData[0]=RXData.ChRxData[7];break;
			case 2:for(i=0;i<2;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 4:for(i=0;i<4;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 6:for(i=0;i<6;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 8:for(i=0;i<8;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break; //速度设置最多为8
			default:break;
		}
		DealData_Rx.CheckSum=(RXData.ChRxData[DealData_Rx.FrameLength-1]<<8)+RXData.ChRxData[DealData_Rx.FrameLength-2];
		
		for(i=0;i<DealData_Rx.FrameLength-2;i++)
		{
			ChekSum+=RXData.ChRxData[i];
		}
		if(ChekSum == DealData_Rx.CheckSum)//数据校验正确
		{
			DealData_Rx.Success_Flag=1;
			return 1;
		}
		else
		{
			DealData_Rx.Success_Flag=0;
			return 0;
		}
	}
	return 0;
}

/*******************************************************************************
* Function Name  : DealRXData
* Description    : 处理接收的数据
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
void DealRXData(void)
{
	if(ExtractData()){;}
	else{return;}
	switch(DealData_Rx.CMD)//命令解析
	{
		//查询指令
		case WhellDiameter:  //轮子直径
		{
			TempTxData.InTempData[0]=Wheel_D;
			break;    
		}
		case WheelBase:      //轮子轴距
		{
			TempTxData.InTempData[0]=Wheel_SPACING;
			break;
		}
		case WhellRollSpeed: //轮子转速
		{
		  //数据放大100倍
			TempTxData.InTempData[0]=ThreeWheel.NowSpeed*100/(PI*Wheel_D);
			TempTxData.InTempData[1]=FourWheel.NowSpeed *100/(PI*Wheel_D);
			TempTxData.InTempData[2]=LeftWheel.NowSpeed *100/(PI*Wheel_D);
			TempTxData.InTempData[3]=RightWheel.NowSpeed*100/(PI*Wheel_D);
			break;
		}
		case WhellSpeed:     //轮子速度
		{		
			TempTxData.InTempData[0]=ThreeWheel.NowSpeed;
			TempTxData.InTempData[1]=FourWheel.NowSpeed;
			TempTxData.InTempData[2]=LeftWheel.NowSpeed;
			TempTxData.InTempData[3]=RightWheel.NowSpeed;
			break;
		}
		case ReductionRatio: //轮子减速比
		{
			TempTxData.InTempData[0]=Wheel_RATIO;
			break;
		}
		case WhellAcceleration://轮子加速度
		{	
			break;
		}
			
		case EncoderLine:      //编码器线数
		{	
			TempTxData.InTempData[0]=ENCODER_LINE;
			break;
		}
		case EncoderValue:     //编码器值
		{
		#if    VERSION==0
			TempTxData.InTempData[0]=GetEncoder.V5;
			TempTxData.InTempData[1]=GetEncoder.V3;
		#elif  VERSION==1
			TempTxData.InTempData[0]=GetEncoder.V3;
			TempTxData.InTempData[1]=GetEncoder.V4;
			TempTxData.InTempData[2]=GetEncoder.V5;
		#endif
			break;
		}
		case IMUData:     		 //陀螺仪数据
		{
			switch(DealData_Rx.DataNum)
			{
				case 6:  //3轴
				{
					TempTxData.InTempData[0]=GetData(ITG3205_Addr,GYRO_XOUT_H);
					TempTxData.InTempData[1]=GetData(ITG3205_Addr,GYRO_YOUT_H);
					TempTxData.InTempData[2]=GetData(ITG3205_Addr,GYRO_ZOUT_H);						//陀螺仪数据
					break;
				}
				case 12: //6轴
				{
					TempTxData.InTempData[0]=GetData(ITG3205_Addr,GYRO_XOUT_H);
					TempTxData.InTempData[1]=GetData(ITG3205_Addr,GYRO_YOUT_H);
					TempTxData.InTempData[2]=GetData(ITG3205_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
					TempTxData.InTempData[3]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
					TempTxData.InTempData[4]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
					TempTxData.InTempData[5]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
					break;
				}
				case 18:  //9轴
				{
					TempTxData.InTempData[0]=GetData(ITG3205_Addr,GYRO_XOUT_H);
					TempTxData.InTempData[1]=GetData(ITG3205_Addr,GYRO_YOUT_H);
					TempTxData.InTempData[2]=GetData(ITG3205_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
					TempTxData.InTempData[3]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
					TempTxData.InTempData[4]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
					TempTxData.InTempData[5]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			
					TempTxData.InTempData[6]=GetQMC5883Data(HMC5883L_Addr,GX_H);
					TempTxData.InTempData[7]=GetQMC5883Data(HMC5883L_Addr,GY_H);
					TempTxData.InTempData[8]=GetQMC5883Data(HMC5883L_Addr,GZ_H);					//磁力计数据
					break;
				}
				default:break;
			}
			break;
		}
		case UltrasonicData: 	 //超声波数据
		{
			break;
		}
		case EmergencyStop:    //急停状态
		{
			TempTxData.InTempData[0]=AllWheel.Erroe_flag.data;
			break;
		}
		case VersionNumber:    //版本号
		{
			TempTxData.InTempData[0]=VERSIONNUMBER;
			break;
		}
		case RemainingBattery: //剩余电量
		{
			TempTxData.ChTempData[0]=AllWheel.Electricity;
			break;  
		}
		case HardwareParameter://硬件参数
		{
			TempTxData.InTempData[0]=Wheel_D;      					//轮子直径
			TempTxData.InTempData[1]=Wheel_SPACING; 				//轮间距
			TempTxData.InTempData[2]=Wheel_RATIO;						//电机减速比
			TempTxData.InTempData[3]=ENCODER_LINE;  				//编码器线数
			TempTxData.InTempData[4]=VERSIONNUMBER; 				//版本号
			TempTxData.ChTempData[10]=AllWheel.Electricity; //电量
			DealData_Rx.Hardware_Init=1;
			//OSTaskStateSet(Task3,TASK_RUNNING);
			//OSTaskStateSet(Task5,TASK_RUNNING);
			break; 
		}
		
		//设置命令
		case SWhellRollSpeed:   //轮子转速
		{
			//数据缩小100倍
			ThreeWheelSpeedSet(TempRxData.InTempData[0]/100*(PI*Wheel_D));//前左
			FourWheelSpeedSet (TempRxData.InTempData[1]/100*(PI*Wheel_D));//前右
			LeftWheelSpeedSet	(TempRxData.InTempData[2]/100*(PI*Wheel_D));//左
		  RightWheelSpeedSet(TempRxData.InTempData[3]/100*(PI*Wheel_D));//右
			break;  
		}
		case SWhellSpeed: 			//轮子速度
		{
			ThreeWheelSpeedSet(TempRxData.InTempData[0]);//前左
			FourWheelSpeedSet (TempRxData.InTempData[1]);//前右
			LeftWheelSpeedSet	(TempRxData.InTempData[2]);//左
		  RightWheelSpeedSet(TempRxData.InTempData[3]);//右
			break;
		}
		case STurningRadius:    //拐弯半径
		{
			break;
		}
		case SWhellAcceleration://轮子加速度
		{
			break;
		}
		
		
		case SChassisAttitude:  //底盘姿态
		{
			OmniWheelscontrol(TempRxData.InTempData[0],TempRxData.InTempData[1],TempRxData.InTempData[2],0);
			break;
		}
		//正确的命令才响应
		default:return;
	}
	
	//响应标志为1才响应
	if(DealData_Rx.Respond_Flag)
	{
		Respond_To_Ros();
	}
}
