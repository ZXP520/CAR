/**
  ******************************************************************************
  * @file    controlturn.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   用于小车的各种运动控制，如：按某个方向旋转多少度，陀螺仪修正走直线等
  *          UWB定点行走的简单实现等
  ******************************************************************************
  * @attention 
  * 
  ******************************************************************************
  */
#include "controlturn.h"		
#include "sys.h"
#include "include.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include <math.h>
#include "Encoder.h"
#include "errordetect.h"
#include "fsc_stos.h"
#include "control.h"	
#include "bsp_usart.h"
#include "dealdata.h"

#define datatype int
#define MAX 35

//队列结构体
typedef struct 
{
	int front;//头
	int endline;//尾
	datatype data [MAX]; //队列数据   运动方向
	datatype data1[MAX]; //队列数据1  运动角度、时间
	datatype data2[MAX]; //队列数据3  运动速度
}queue;
queue Queue;

//初始化队列
void initQueue(void )
{
	Queue.front = Queue.endline = 0;
	memset(Queue.data, 0,MAX);
	memset(Queue.data1,0,MAX);
	memset(Queue.data2,0,MAX);
}

//入队
void enQueue(datatype data,datatype data1,datatype data2)
{
	//判断溢出
	if (Queue.endline<MAX)
	{
		//判断为空
		if (Queue.front == Queue.endline)
		{
			initQueue();
		}
		Queue.data[Queue.endline]  = data;
		Queue.data1[Queue.endline] = data1;
		Queue.data2[Queue.endline] = data2;
		Queue.endline += 1; 
	}
	else if(Queue.endline==MAX) //到了末尾循环覆盖
	{
		Queue.endline=0;
		Queue.data[Queue.endline]  = data;
		Queue.data1[Queue.endline] = data1;
		Queue.data2[Queue.endline] = data2;
		Queue.front += 1; 
		return;
	}
    
}

//出队
u8 deQueue(void)
{
	//判断为空
	if (Queue.front==MAX)
	{
		Queue.front=0;//到了末尾循环到头
	}
	if (Queue.front == Queue.endline+1)
	{
		return 0;
	}
	else
	{
	//Queue.front+= 1;
		return Queue.data[Queue.front++];
	}
}

//遍历队列
void printfQueue(void)
{
	u8 i=0;
    for (i = Queue.front; i < Queue.endline; i++)
    {
        printf("%d\n", Queue.data[i]);
    }
}

//判断队列是否为空
u8 EmptyQueue(void)
{
	if (Queue.front == Queue.endline)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**************************************************************************
函数功能：全向轮小车运动总控制
入口参数无 5ms进入一次
返回  值：无
**************************************************************************/
static u8 Task6_Step=0,QueueTask=100,Time_Cnt=0;;
//旋转任务总控制
void AllControlTrun(void)
{
	
	//没去零飘返回
	if(ImuData.ZeroFtlag==0)
	{
		return;
	}
	
	//任务切换延时500ms
	if(Task6_Step==0)
	{
		Time_Cnt++;
		if(Time_Cnt>100)
		{
			QueueTask=deQueue();
			Task6_Step=1;
			Time_Cnt=0;
		}
		return;//不返回则会出错
	}
	switch(QueueTask)
	{
		case 0:						//队列为空，暂停任务6
		{
			Task6_Step=0;
			OSTaskStateSet(Task6,TASK_PAUSING);
			break;
		}
		case Straight:    //直行
		{
			ControlStraight(Queue.data2[Queue.front-1],Queue.data1[Queue.front-1]);
			break;
		}
		case Back:				//后退
		{
			break;
		}
		case TurnLeft:    //左旋
		{
			ControlTrun(-abs(Queue.data2[Queue.front-1]),Queue.data1[Queue.front-1]);
			break;
		}
		case TurnRight:   //右旋
		{
			ControlTrun(abs(Queue.data2[Queue.front-1]),Queue.data1[Queue.front-1]);
			break;
		}
		case Stop:			  //终止所有任务，并清空任务队列
		{
			Task6_Step=0;
			initQueue();
			OmniWheelscontrol(0,0,0,0);
			OSTaskStateSet(Task6,TASK_PAUSING);
			break;
		}
		default:break;
	}
	
}

/**************************************************************************
函数功能：控制全向轮小车运动功能
入口参数dire运动状态  angle运动角度或者时间  speed运动速度
返回  值：无
**************************************************************************/
void SetTurn(int dire,int angle,int speed)
{
	if(OSTaskStateGet(Task6)==TASK_PAUSING)
	{
		//任务暂停初始化队列		说明任务指令已经完成，初始化队列
		initQueue();
		OSTaskStateSet(Task6,TASK_RUNNING);
	}
	enQueue(dire,angle,speed);
}

/**************************************************************************
函数功能：控制全向轮小车旋转功能
入口参数peed方向和速度  angel旋转角度
返回  值：无
**************************************************************************/
void ControlTrun(s8 speed,u16 angle)
{
	static u8 Trun_Flag=0;
	static float FirstAngle=0;
	if(angle>90)
	{
		angle=angle-10;
	}
	//没去零飘返回
	switch(Trun_Flag)
	{
		case 0: 
		{
			FirstAngle=ImuData.Yaw;
			OmniWheelscontrol(0,0,speed,0);
			Trun_Flag=1;
			break;
		}
		case 1:
		{
			if(speed>0)  //右旋
			{
				if(ImuData.Yaw>FirstAngle&&(360-ImuData.Yaw+FirstAngle)>angle)
				{
					OmniWheelscontrol(0,0,0,0);
					Trun_Flag=2;
				}
				else if(ImuData.Yaw<FirstAngle&&(FirstAngle-ImuData.Yaw)>angle)
				{
					OmniWheelscontrol(0,0,0,0);
					Trun_Flag=2;
				}
			}
			else 				//左旋
			{
				if(ImuData.Yaw>FirstAngle&&(ImuData.Yaw-FirstAngle)>angle)
				{
					OmniWheelscontrol(0,0,0,0);
					Trun_Flag=2;
				}
				else if(ImuData.Yaw<FirstAngle&&(ImuData.Yaw+360-FirstAngle)>angle)
				{
					OmniWheelscontrol(0,0,0,0);
					Trun_Flag=2;
				}
			}
			break;
		}
		case 2:
		{
			Task6_Step=0;//查询队列中的下一个任务
			Trun_Flag=0;
			break;
		}
		default:break;
	}
}

/**************************************************************************
函数功能：控制全向轮小车直行功能
入口参数peed速度  settime直行时间 ms
返回  值：无
**************************************************************************/
void ControlStraight(int speed,int settime)
{
	static u8 Trun_Flag=0;
	static float FirstAngle=0,AngleError=0;
	static int RAimEncoder=0,LAimEncoder=0,TAimEncoder=0;
	static u32 NowTime=0,FirstTime=0;
	//没去零飘返回
	switch(Trun_Flag)
	{
		case 0: 
		{
			OmniWheelscontrol(0,speed,0,0);
			FirstAngle=ImuData.Yaw;
			FirstTime=GetOSSliceTime();
			LAimEncoder=LeftWheel.AimsEncoder;
			RAimEncoder=RightWheel.AimsEncoder;
			TAimEncoder=ThreeWheel.AimsEncoder;
			Trun_Flag=1;
			break;
		}
		case 1:
		{
			if(speed>0)  //向前
			{
				if(ImuData.Yaw>FirstAngle)
				{
					if(ImuData.Yaw-FirstAngle>200)//越界
					{
						AngleError=360-ImuData.Yaw+FirstAngle;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder-AngleError*0.1;//角度系数0.1比较稳定
						if(RightWheel.AimsEncoder<RAimEncoder*0.8)//限制输出
						{
							RightWheel.AimsEncoder=RAimEncoder*0.8;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder+AngleError*0.1;
						if(ThreeWheel.AimsEncoder>TAimEncoder*1.2)//限制输出
						{
							ThreeWheel.AimsEncoder=TAimEncoder*1.2;
						}
					}
					else  //非越界
					{
						AngleError=ImuData.Yaw-FirstAngle;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder+AngleError*0.1;
						if(RightWheel.AimsEncoder>RAimEncoder*1.2)//限制输出
						{
							RightWheel.AimsEncoder=RAimEncoder*1.2;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder-AngleError*0.1;
						if(ThreeWheel.AimsEncoder<TAimEncoder*0.8)//限制输出
						{
							ThreeWheel.AimsEncoder=TAimEncoder*0.8;
						}
					}
				}
				else if(ImuData.Yaw<FirstAngle)
				{
					if(ImuData.Yaw-FirstAngle<-200)//越界
					{
						AngleError=360-FirstAngle+ImuData.Yaw;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder+AngleError*0.1;
						if(RightWheel.AimsEncoder>RAimEncoder*1.2)//限制输出
						{
							RightWheel.AimsEncoder=RAimEncoder*1.2;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder-AngleError*0.1;
						if(ThreeWheel.AimsEncoder<TAimEncoder*0.8)//限制输出
						{
							ThreeWheel.AimsEncoder=TAimEncoder*0.8;
						}
						
					}
					else
					{
						AngleError=FirstAngle-ImuData.Yaw;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder-AngleError*0.1;
						if(RightWheel.AimsEncoder<RAimEncoder*0.8)//限制输出
						{
							RightWheel.AimsEncoder=RAimEncoder*0.8;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder+AngleError*0.1;
						if(ThreeWheel.AimsEncoder>TAimEncoder*1.2)//限制输出
						{
							ThreeWheel.AimsEncoder=TAimEncoder*1.2;
						}	
					}
				}
			}
			else 				//向后
			{
				
			}
			
			//计算超时
			NowTime=GetOSSliceTime();
			if((NowTime-FirstTime)*2>settime)
			{
				OmniWheelscontrol(0,0,0,0);
				Trun_Flag=2;
			}
			break;
		}
		case 2:
		{
			Task6_Step=0;//查询队列中的下一个任务
			Trun_Flag=0;
			break;
		}
		default:break;
	}
}



/**************************************************************************
函数功能：UWB小车定位
入口参数：需要定位到的X,Y轴坐标 ,和三个基站的坐标
返回  值：无
**************************************************************************/
void UWBTurnToX(double x,double y,double x1,double y1,double x2,double y2,double x3,double y3)
{
	 static double oldX=0,oldY=0,result=0;
	 static u8 Turn_Step=0;
	 static int RAimEncoder=0,LAimEncoder=0,TAimEncoder=0,Dspeed=0;
	 UWBData.x1=x1;
	 UWBData.y1=y1;
	 UWBData.x2=x2;
	 UWBData.y2=y2;//6000;
	 UWBData.x3=x3;//2400;
	 UWBData.y3=y3; 
	
	
	 if(PSBKey.GREEN==0)//遥控器绿色按键没按
	 {
		Turn_Step=0;
		oldX=0;
		oldY=0;
		result=0;
		return;
	 }
	 switch(Turn_Step)
	 {
		 case 0:  //取初始角度
		 {
			 if(UWBData.StableFlag)//数据稳定
			 {
				 UWBData.StableFlag=0;
				 oldY=UWBData.Y;
				 oldX=UWBData.X;
				 Turn_Step=1;	
			 }
			 break;
		 }
		 case 1:
		 {
			 SetTurn(Straight,3000,200);
			 Turn_Step=2;
			 break;
		 }
		 case 2:  //角度矫正，平行于X轴
		 {
			 if(OSTaskStateGet(Task6)==TASK_PAUSING&&Task6_Step==0)//动作完成，任务6就会暂停
			 {
				  if(UWBData.StableFlag)//数据稳定
					{
						UWBData.StableFlag=0;
						result = atan ((UWBData.Y-oldY)/(UWBData.X-oldX)) * 180 / PI;  //弧度转化为度
						
						//角度修正
						if(result>=0)
						{
							if(UWBData.X>=oldX)
							{
								SetTurn(TurnRight,abs(result),100);
							}
							else
							{
								SetTurn(TurnLeft,180-result,100);
							}
						}
						else
						{
							if(UWBData.X>=oldX)
							{
								SetTurn(TurnLeft,abs(result),100);
							}
							else
							{
								SetTurn(TurnRight,180+result,100);
							}
						}
						Turn_Step=3;
					}
			 }
			 break;
		 }
		 case 3:  //走到X点，并再次角度矫正
		 {
			 oldY=UWBData.Y;//取坐标
			 oldX=UWBData.X;
			 if(UWBData.X<x)       //现在的X轴小于设定的，直行
			 {
				 SetTurn(Straight,30000,200);
				 Turn_Step=4;
			 }
			 else if(UWBData.X>x)  //现在的X轴小于设定的，旋转180直行 
			 {
				 SetTurn(TurnRight,180,100);
				 SetTurn(Straight,30000,200);
				 Turn_Step=4;
			 }
			 else
			 {
				 Turn_Step=4;
			 }
			 
			 break;
		 }
		 case 4:   //到达X轴
		 {
			 if(abs(UWBData.X-x)<20)//距离小于20cm
			 {
				 SetTurn(Stop,0,0);
				 //PSBKey.GREEN=0;
				 Turn_Step=5;
			 }
			 break;
		 }
		 case 5:   //再次矫正
		 {
			 if(OSTaskStateGet(Task6)==TASK_PAUSING&&Task6_Step==0)//动作完成，任务6就会暂停
			 {
				  if(UWBData.StableFlag)//数据稳定
					{
						UWBData.StableFlag=0;
						result = atan ((UWBData.Y-oldY)/(UWBData.X-oldX)) * 180 / PI;  //弧度转化为度
						
						//角度修正
						if(result>=0)
						{
							if(UWBData.X>=oldX)
							{
								SetTurn(TurnRight,abs(result),100);
							}
							else
							{
								SetTurn(TurnLeft,180-result,100);
							}
						}
						else
						{
							if(UWBData.X>=oldX)
							{
								SetTurn(TurnLeft,abs(result),100);
							}
							else
							{
								SetTurn(TurnRight,180+result,100);
							}
						}
						Turn_Step=6;
					}
			 }
			 break;
		 }
		 case 6:		//旋转90度平行于Y轴
		 {
			 if(UWBData.Y<y)       //现在的Y轴小于设定的，直行
			 {
				 SetTurn(TurnLeft,90,100);
				 Turn_Step=7;
			 }
			 else if(UWBData.Y>y)  //现在的X轴小于设定的，旋转180直行 
			 {
				 SetTurn(TurnRight,90,100);	
				 Turn_Step=7;
			 }
			 else
			 {
				 Turn_Step=7;
				 
			 }
			 break;
		 }
		 case 7: //旋转完成
		 {
			 if(OSTaskStateGet(Task6)==TASK_PAUSING&&Task6_Step==0)//动作完成，任务6就会暂停
			 {
				 OmniWheelscontrol(0,200,0,0);
				 LAimEncoder=LeftWheel.AimsEncoder;
				 RAimEncoder=RightWheel.AimsEncoder;
				 TAimEncoder=ThreeWheel.AimsEncoder;
				 Turn_Step=8;
			 }
			 break;
		 }
		 case 8: //平行Y轴走到设定点
		 {
			 if(abs(UWBData.Y-y)<20)//距离小于20cm
			 {
				 SetTurn(Stop,0,0);
				 PSBKey.GREEN=0;
				 Turn_Step=0;
			 }
			 else
			 {
				 //以X轴误差来矫正走直线
				 Dspeed=(UWBData.X-x)*2;
				 if(Dspeed>50){Dspeed=50;}
				 else if(Dspeed<-50){Dspeed=-50;}
				 OmniWheelscontrol(
				 Dspeed,200,0,0);
				 
				 
			 }
			 break;
		 }
		 default:break;
	 }
}



