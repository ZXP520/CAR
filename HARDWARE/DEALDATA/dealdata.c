/**
  ******************************************************************************
  * @file    dealdata.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   用于串口接收的各类数据、uwb模块的定位数据处理等
  ******************************************************************************
  * @attention 
  * 
  ******************************************************************************
  */
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
#include <math.h>

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
	TXData.InRxData[0]=DATAHEAD;						//头
	TXData.ChRxData[2]=DealData_Rx.FrameLength;			//帧长度
	TXData.ChRxData[3]=DealData_Rx.Product_Type;		//产品类型
	TXData.ChRxData[4]=DealData_Rx.CMD&0xFF;			//命令   小端模式先低后高
	TXData.ChRxData[5]=(DealData_Rx.CMD>>8)&0xFF;
	TXData.ChRxData[6]=DealData_Rx.DataNum;				//数据个数	
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

/*******************************************************************************
* Function Name  : Respond_To_Ros
* Description    : 读取IMU的零飘值
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
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
			GetQMC5883Data(ADXL345_Addr,ACCEL_ZOUT_H);				//加速度计数据
			
			GetQMC5883Data(HMC5883L_Addr,GX_H);
			GetQMC5883Data(HMC5883L_Addr,GY_H);
			GetQMC5883Data(HMC5883L_Addr,GZ_H);						//磁力计数据
			}
		else
		{
			ImuData.OffsetGYData[0]+=(float)((int16_t)GetData(ITG3205_Addr,GYRO_XOUT_H))*0.01;
			ImuData.OffsetGYData[1]+=(float)((int16_t)GetData(ITG3205_Addr,GYRO_YOUT_H))*0.01;
			ImuData.OffsetGYData[2]+=(float)((int16_t)GetData(ITG3205_Addr,GYRO_ZOUT_H))*0.01;								//陀螺仪数据
			
			ImuData.OffsetAccelData[0]+=(float)((int16_t)GetQMC5883Data(ADXL345_Addr,ACCEL_XOUT_H))*0.01;
			ImuData.OffsetAccelData[1]+=(float)((int16_t)GetQMC5883Data(ADXL345_Addr,ACCEL_YOUT_H))*0.01;
			ImuData.OffsetAccelData[2]+=(float)((int16_t)GetQMC5883Data(ADXL345_Addr,ACCEL_ZOUT_H))*0.01;					//加速度计数据
			
			ImuData.OffsetMagnetData[0]+=(float)((int16_t)GetQMC5883Data(HMC5883L_Addr,GX_H))*0.01;
			ImuData.OffsetMagnetData[1]+=(float)((int16_t)GetQMC5883Data(HMC5883L_Addr,GY_H))*0.01;
			ImuData.OffsetMagnetData[2]+=(float)((int16_t)GetQMC5883Data(HMC5883L_Addr,GZ_H))*0.01;							//磁力计数据

			delay_ms(5);
		}
		GetIMUData_cnt++;
	}

}

/*******************************************************************************
* Function Name  : DealRXData
* Description    : 每20MS上传编码器陀螺仪的数据
* Input          : sendflag  发送标志 
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
			TXData.ChRxData[2]=46;												//帧长度  2+1+1+2+1+37+2=46
			TXData.ChRxData[3]=PRODUCTTYPE ;												//产品类型
			TXData.ChRxData[4]= ONSendData20ms    &0xFF;			  			//命令   小端模式先低后高
			TXData.ChRxData[5]=(ONSendData20ms>>8)&0xFF;
			TXData.ChRxData[6]=37;												//数据个数	8+8+18+2+1=37
			TempTxData.InTempData[0]=GetEncoder.V2;
			TempTxData.InTempData[1]=GetEncoder.V3;
			TempTxData.InTempData[2]=GetEncoder.V4;
			TempTxData.InTempData[3]=GetEncoder.V5;
			
			TempTxData.InTempData[4]=LeftWheel.NowSpeed;
			TempTxData.InTempData[5]=RightWheel.NowSpeed;
			TempTxData.InTempData[6]=ThreeWheel.NowSpeed;
			TempTxData.InTempData[7]=FourWheel.NowSpeed;
			break;
		}
		case 2:  //5ms
		{
			TempTxData.InTempData[8]=GetData(ITG3205_Addr,GYRO_XOUT_H);
			TempTxData.InTempData[9]=GetData(ITG3205_Addr,GYRO_YOUT_H);
			TempTxData.InTempData[10]=GetData(ITG3205_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			//用于错误检测
			ImuData.NGYData[0]=TempTxData.InTempData[8];
			ImuData.NGYData[1]=TempTxData.InTempData[9];
			ImuData.NGYData[2]=TempTxData.InTempData[10];
			break;
		}
		case 3:  //10ms
		{
			TempTxData.InTempData[11]=GetQMC5883Data(ADXL345_Addr,ACCEL_XOUT_H);
			TempTxData.InTempData[12]=GetQMC5883Data(ADXL345_Addr,ACCEL_YOUT_H);
			TempTxData.InTempData[13]=GetQMC5883Data(ADXL345_Addr,ACCEL_ZOUT_H);				//加速度计数据
			
			//用于错误检测
			ImuData.NAccelData[0]=TempTxData.InTempData[11];
			ImuData.NAccelData[1]=TempTxData.InTempData[12];
			ImuData.NAccelData[2]=TempTxData.InTempData[13];
			break;
		}
		case 4:  //15ms
		{
			TempTxData.InTempData[14]=GetQMC5883Data(HMC5883L_Addr,GX_H);
			TempTxData.InTempData[15]=GetQMC5883Data(HMC5883L_Addr,GY_H);
			TempTxData.InTempData[16]=GetQMC5883Data(HMC5883L_Addr,GZ_H);						//磁力计数据
			//用于错误检测
			ImuData.NMagnetData[0]=TempTxData.InTempData[14];
			ImuData.NMagnetData[1]=TempTxData.InTempData[15];
			ImuData.NMagnetData[2]=TempTxData.InTempData[16];
			
			break;
		}
		case 5:  //20ms
		{
			TempTxData.InTempData[17]=AllWheel.Erroe_flag.data;									//错误数据
			TempTxData.ChTempData[36]=AllWheel.Electricity;										//电量数据
			
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
			TXData.ChRxData[TXData.ChRxData[2]-2]=Cheksum&0xFF; 								//校验和
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
		DealData_Rx.FrameLength=RXData.ChRxData[2];   						 				//包长度
		DealData_Rx.Product_Type=RXData.ChRxData[3];               							//产品类型
		if(DealData_Rx.Product_Type != PRODUCTTYPE)											//如果产品类型不同则返回
		{
			return 0;
		}
		DealData_Rx.CMD=(RXData.ChRxData[4] + (RXData.ChRxData[5]<<8)); 					//命令     //先低后高
		
		DealData_Rx.DataNum=RXData.ChRxData[6];										 		//数据个数
		switch(DealData_Rx.DataNum)
		{
			case 0:break;
			case 1:TempRxData.InTempData[0]=RXData.ChRxData[7];break;
			case 2:for(i=0;i<2;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 4:for(i=0;i<4;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 5:for(i=0;i<5;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 6:for(i=0;i<6;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 8:for(i=0;i<8;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;  //速度设置最多为8
			case 9:for(i=0;i<9;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;  //摇杆数据为9
			default:break;
		}
		DealData_Rx.CheckSum=(RXData.ChRxData[DealData_Rx.FrameLength-1]<<8)+RXData.ChRxData[DealData_Rx.FrameLength-2];
		
		for(i=0;i<DealData_Rx.FrameLength-2;i++)
		{
			ChekSum+=RXData.ChRxData[i];
		}
		if(ChekSum == DealData_Rx.CheckSum)												  //数据校验正确
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
//手柄按键结构体
PSBKEY PSBKey;
void DealRXData(void)
{
	if(ExtractData()){;}
	else{return;}
	switch(DealData_Rx.CMD)  //命令解析
	{
		//查询指令
		case ProductType:	 //产品类型
		{
			TempTxData.ChTempData[0]=PRODUCTTYPE;
			break;
		}
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
		case OFFSendData20ms:		//关闭20ms数据上传
		{
			DealData_Rx.SendData20ms_Flag=0;
			break;
		}
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
		case SPSRawData:		//PS原始数据
		{
			PSBKey.PSS_LY=TempRxData.InTempData[0];
			PSBKey.PSS_LX=TempRxData.InTempData[1];
			PSBKey.PSS_RY=TempRxData.InTempData[2];
			PSBKey.PSS_RX=TempRxData.InTempData[3];
			//遥控器按键
			switch(TempRxData.ChTempData[8])
			{
				case PSB_PAD_UP: 		PSBKey.UP=1;	 	break;
				case PSB_PAD_RIGHT: 	PSBKey.RIGHT=1;		break;
				case PSB_PAD_DOWN: 		PSBKey.DOWN=1;		break;
				case PSB_PAD_LEFT: 		PSBKey.LEFT=1; 		break;
				case PSB_L2: 			PSBKey.L2=1;		break;
				case PSB_R2: 			PSBKey.R2=1;		break;
				case PSB_L1: 			PSBKey.L1=1;		break;
				case PSB_R1: 			PSBKey.R1=1;	 	break;
				case PSB_GREEN: 		PSBKey.GREEN=1;		break;
				case PSB_RED:	  		PSBKey.RED=1  ;		break;
				case PSB_BLUE: 			PSBKey.BLUE=1 ;		break;
				case PSB_PINK: 			PSBKey.PINK=1	;	break;
			}
			DealPSData();
			
			break;
		}
		case ONSendData20ms:		//开启20ms数据上传
		{
			DealData_Rx.SendData20ms_Flag=1;
			break;
		}
		//正确的命令才响应
		default:return;
	}
	
	//都需要响应
	Respond_To_Ros();
}

/*******************************************************************************
* Function Name  : DealRXData
* Description    : 处理PS2标签数据
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
void DealPSData(void)
{
	s16 speed_RX=0,speed_RY=0,speed_LX=0,speed_LY=0; 
	
	speed_LX=(PSBKey.PSS_LX-128)*2.5;
	speed_LY=(127-PSBKey.PSS_LY)*2.5;
	speed_RX=(PSBKey.PSS_RX-128);
	OmniWheelscontrol(speed_LX,speed_LY,speed_RX,0);
}

/*******************************************************************************
* Function Name  : DealRXData
* Description    : 处理UWB标签数据
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
//原始数据
//mr 07 00000aeb 00000da6 00000ef3 00000000 0237 bf 40224022 t4:0
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//定义数据结构体
UWBDATA UWBData;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DealUWBData(void)
{
	u8 i=0,j=0;
	u8 temp=0;
	for(i=0;i<200;i++)
	{
		if(USART3_Rx_Buff[i]=='m'&&USART3_Rx_Buff[i+1]=='r'&&USART3_Rx_Buff[i+2]==' '&&USART3_Rx_Buff[i+3]=='0'&&USART3_Rx_Buff[i+4]=='7')
		{
			//A0
			for(j=0;j<8;j++)
			{
				UWBData.A0=UWBData.A0<<4;
				temp=USART3_Rx_Buff[i+6+j];
				if(temp>='0'&&temp<='9')
				{
					temp=temp-'0';
				}
				else if(temp>='a'&&temp<='f')
				{
					temp=temp-0x57;
				}
				UWBData.A0+= temp;
			}
			
			//A1
			for(j=0;j<8;j++)
			{
				UWBData.A1=UWBData.A1<<4;
				temp=USART3_Rx_Buff[i+15+j];
				if(temp>='0'&&temp<='9')
				{
					temp=temp-'0';
				}
				else if(temp>='a'&&temp<='f')
				{
					temp=temp-0x57;
				}
				UWBData.A1+= temp;
			}
			
			
			//A2
			for(j=0;j<8;j++)
			{
				UWBData.A2=UWBData.A2<<4;
				temp=USART3_Rx_Buff[i+24+j];
				if(temp>='0'&&temp<='9')
				{
					temp=temp-'0';
				}
				else if(temp>='a'&&temp<='f')
				{
					temp=temp-0x57;
				}
				UWBData.A2+= temp;
			}
			
			
			//A3
			for(j=0;j<8;j++)
			{
				UWBData.A3=UWBData.A3<<4;
				temp=USART3_Rx_Buff[i+33+j];
				if(temp>='0'&&temp<='9')
				{
					temp=temp-'0';
				}
				else if(temp>='a'&&temp<='f')
				{
					temp=temp-0x57;
				}
				UWBData.A3+= temp;
			}	
			calcPhonePosition(UWBData.x1,UWBData.y1,UWBData.A0,
												UWBData.x2,UWBData.y2,UWBData.A1,
												UWBData.x3,UWBData.y3,UWBData.A2);
			break;
		}
	}
}

/*******************************************************************************
* Function Name  : UWBAbs
* Description    : 绝对值
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
double UWBAbs(double data)
{
	if(data<0)
	{
		return -data;
	}
	else
	{
		return data;
	}
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
UWB数据稳定性判断
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void JudgeUWBXYStable(void)
{
	static double LastX=0,LastY=0,TempX=0,TempY=0;
	static u8 StableFlag=0;
	if(UWBAbs(UWBData.Y-LastY)<10 && UWBAbs(UWBData.X-LastX)<10)//偏差小于10cm累计20次则算稳定
	{
		TempX+=UWBData.X*(1/20.0);
		TempY+=UWBData.Y*(1/20.0);
		StableFlag++;
	}
	else
	{
		TempX=0;
		TempY=0;
	
		UWBData.StableFlag=0;
		StableFlag=0;
	}
	if(StableFlag>20)
	{
		UWBData.AverageX=TempX;//获得平均值
		UWBData.AverageY=TempY;
		UWBData.StableFlag=1;
		StableFlag=0;
	}
	LastY=UWBData.Y;
	LastX=UWBData.X;
}

/*---------------------------------------------------------------------------------------------------------------------------------
三边定位法

设未知点位置为 (x, y)， 令其中的第一个球形 P0 的球心坐标为 (0, 0)，P2 处于相同纵坐标，球心坐标为 (d, 0)，P1 球心坐标为 (i, j)，
三个球形半径分别为 r1, r2, r3，z为三球形相交点与水平面高度。则有：

r12 = x2 + y2 + z2                P0(0,0)		P1(0,2.5)   P2(2.5,0)
r22 = (x - d)2 + y2 + z2
r32 = (x - i)2 + (y - j)2 + z2
当 z = 0 时， 即为三个圆在水平面上相交为一点，首先解出 x:

x = (r12 - r22 + d2) / 2d
将公式二变形，将公式一的 z2 代入公式二，再代入公式三得到 y 的计算公式：

y = (r12 - r32 - x2 + (x - i)2 + j2) / 2j

-------------------------------------------------------------------------------------------------------------------------------------*/

	// 三边测量法
    // 通过三点坐标和到三点的距离，返回第4点位置
void calcPhonePosition   (double x1, double y1, double d1,
													double x2, double y2, double d2,
													double x3, double y3, double d3) 
{
		
		double a11 = 2 * (x1 - x3);
		double a12 = 2 * (y1 - y3);
		double b1  = pow(x1, 2) - pow(x3, 2)
						   + pow(y1, 2) - pow(y3, 2)
						   + pow(d3, 2) - pow(d1, 2);
		double a21 = 2 * (x2 - x3);
		double a22 = 2 * (y2 - y3);
		double b2  = pow(x2, 2) - pow(x3, 2)
						   + pow(y2, 2) - pow(y3, 2)
						   + pow(d3, 2) - pow(d2, 2);

		UWBData.X = ((b1 * a22 - a12 * b2) / (a11 * a22 - a12 * a21))/10;
		UWBData.Y = ((a11 * b2 - b1 * a21) / (a11 * a22 - a12 * a21))/10;
		JudgeUWBXYStable();//数据稳定判断
}



