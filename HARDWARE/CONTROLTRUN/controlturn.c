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

#define datatype int
#define MAX 35

//队列结构体
typedef struct 
{
    int front;//头
    int endline;//尾
    datatype data [MAX]; //队列数据  运动方向
	  datatype data1[MAX];//队列数据1  运动角度、时间
	  datatype data2[MAX];//队列数据3  运动速度
}queue;
queue Queue;

//初始化队列
void initQueue(void )
{
    Queue.front = Queue.endline = 0;
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
    else if(Queue.endline==MAX) //到了末尾循环
		{
				Queue.endline=0;
				Queue.data[Queue.endline]  = data;
				Queue.data1[Queue.endline] = data1;
				Queue.data2[Queue.endline] = data2;
        Queue.endline += 1; 
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
	
	//没去零飘或者队列为空返回
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
		case Stop:
		{
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
	angle=angle-10;
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
入口参数peed速度  settime直行时间
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
					//Trun_Flag=2;
				}
				else if(ImuData.Yaw<FirstAngle)
				{
					if(ImuData.Yaw-FirstAngle<-200)//越界
					{
						AngleError=360-FirstAngle+ImuData.Yaw;
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
					//Trun_Flag=2;
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





