/**
  ******************************************************************************
  * @file    controlturn.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   ����С���ĸ����˶����ƣ��磺��ĳ��������ת���ٶȣ�������������ֱ�ߵ�
  *          UWB�������ߵļ�ʵ�ֵ�
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

//���нṹ��
typedef struct 
{
	int front;//ͷ
	int endline;//β
	datatype data [MAX]; //��������   �˶�����
	datatype data1[MAX]; //��������1  �˶��Ƕȡ�ʱ��
	datatype data2[MAX]; //��������3  �˶��ٶ�
}queue;
queue Queue;

//��ʼ������
void initQueue(void )
{
	Queue.front = Queue.endline = 0;
	memset(Queue.data, 0,MAX);
	memset(Queue.data1,0,MAX);
	memset(Queue.data2,0,MAX);
}

//���
void enQueue(datatype data,datatype data1,datatype data2)
{
	//�ж����
	if (Queue.endline<MAX)
	{
		//�ж�Ϊ��
		if (Queue.front == Queue.endline)
		{
			initQueue();
		}
		Queue.data[Queue.endline]  = data;
		Queue.data1[Queue.endline] = data1;
		Queue.data2[Queue.endline] = data2;
		Queue.endline += 1; 
	}
	else if(Queue.endline==MAX) //����ĩβѭ������
	{
		Queue.endline=0;
		Queue.data[Queue.endline]  = data;
		Queue.data1[Queue.endline] = data1;
		Queue.data2[Queue.endline] = data2;
		Queue.front += 1; 
		return;
	}
    
}

//����
u8 deQueue(void)
{
	//�ж�Ϊ��
	if (Queue.front==MAX)
	{
		Queue.front=0;//����ĩβѭ����ͷ
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

//��������
void printfQueue(void)
{
	u8 i=0;
    for (i = Queue.front; i < Queue.endline; i++)
    {
        printf("%d\n", Queue.data[i]);
    }
}

//�ж϶����Ƿ�Ϊ��
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
�������ܣ�ȫ����С���˶��ܿ���
��ڲ����d�� 5ms����һ��
����  ֵ����
**************************************************************************/
static u8 Task6_Step=0,QueueTask=100,Time_Cnt=0;;
//��ת�����ܿ���
void AllControlTrun(void)
{
	
	//ûȥ��Ʈ����
	if(ImuData.ZeroFtlag==0)
	{
		return;
	}
	
	//�����л���ʱ500ms
	if(Task6_Step==0)
	{
		Time_Cnt++;
		if(Time_Cnt>100)
		{
			QueueTask=deQueue();
			Task6_Step=1;
			Time_Cnt=0;
		}
		return;//������������
	}
	switch(QueueTask)
	{
		case 0:						//����Ϊ�գ���ͣ����6
		{
			Task6_Step=0;
			OSTaskStateSet(Task6,TASK_PAUSING);
			break;
		}
		case Straight:    //ֱ��
		{
			ControlStraight(Queue.data2[Queue.front-1],Queue.data1[Queue.front-1]);
			break;
		}
		case Back:				//����
		{
			break;
		}
		case TurnLeft:    //����
		{
			ControlTrun(-abs(Queue.data2[Queue.front-1]),Queue.data1[Queue.front-1]);
			break;
		}
		case TurnRight:   //����
		{
			ControlTrun(abs(Queue.data2[Queue.front-1]),Queue.data1[Queue.front-1]);
			break;
		}
		case Stop:			  //��ֹ�������񣬲�����������
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
�������ܣ�����ȫ����С���˶�����
��ڲ����ddire�˶�״̬  angle�˶��ǶȻ���ʱ��  speed�˶��ٶ�
����  ֵ����
**************************************************************************/
void SetTurn(int dire,int angle,int speed)
{
	if(OSTaskStateGet(Task6)==TASK_PAUSING)
	{
		//������ͣ��ʼ������		˵������ָ���Ѿ���ɣ���ʼ������
		initQueue();
		OSTaskStateSet(Task6,TASK_RUNNING);
	}
	enQueue(dire,angle,speed);
}

/**************************************************************************
�������ܣ�����ȫ����С����ת����
��ڲ����speed������ٶ�  angel��ת�Ƕ�
����  ֵ����
**************************************************************************/
void ControlTrun(s8 speed,u16 angle)
{
	static u8 Trun_Flag=0;
	static float FirstAngle=0;
	if(angle>90)
	{
		angle=angle-10;
	}
	//ûȥ��Ʈ����
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
			if(speed>0)  //����
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
			else 				//����
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
			Task6_Step=0;//��ѯ�����е���һ������
			Trun_Flag=0;
			break;
		}
		default:break;
	}
}

/**************************************************************************
�������ܣ�����ȫ����С��ֱ�й���
��ڲ����speed�ٶ�  settimeֱ��ʱ�� ms
����  ֵ����
**************************************************************************/
void ControlStraight(int speed,int settime)
{
	static u8 Trun_Flag=0;
	static float FirstAngle=0,AngleError=0;
	static int RAimEncoder=0,LAimEncoder=0,TAimEncoder=0;
	static u32 NowTime=0,FirstTime=0;
	//ûȥ��Ʈ����
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
			if(speed>0)  //��ǰ
			{
				if(ImuData.Yaw>FirstAngle)
				{
					if(ImuData.Yaw-FirstAngle>200)//Խ��
					{
						AngleError=360-ImuData.Yaw+FirstAngle;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder-AngleError*0.1;//�Ƕ�ϵ��0.1�Ƚ��ȶ�
						if(RightWheel.AimsEncoder<RAimEncoder*0.8)//�������
						{
							RightWheel.AimsEncoder=RAimEncoder*0.8;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder+AngleError*0.1;
						if(ThreeWheel.AimsEncoder>TAimEncoder*1.2)//�������
						{
							ThreeWheel.AimsEncoder=TAimEncoder*1.2;
						}
					}
					else  //��Խ��
					{
						AngleError=ImuData.Yaw-FirstAngle;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder+AngleError*0.1;
						if(RightWheel.AimsEncoder>RAimEncoder*1.2)//�������
						{
							RightWheel.AimsEncoder=RAimEncoder*1.2;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder-AngleError*0.1;
						if(ThreeWheel.AimsEncoder<TAimEncoder*0.8)//�������
						{
							ThreeWheel.AimsEncoder=TAimEncoder*0.8;
						}
					}
				}
				else if(ImuData.Yaw<FirstAngle)
				{
					if(ImuData.Yaw-FirstAngle<-200)//Խ��
					{
						AngleError=360-FirstAngle+ImuData.Yaw;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder+AngleError*0.1;
						if(RightWheel.AimsEncoder>RAimEncoder*1.2)//�������
						{
							RightWheel.AimsEncoder=RAimEncoder*1.2;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder-AngleError*0.1;
						if(ThreeWheel.AimsEncoder<TAimEncoder*0.8)//�������
						{
							ThreeWheel.AimsEncoder=TAimEncoder*0.8;
						}
						
					}
					else
					{
						AngleError=FirstAngle-ImuData.Yaw;
						RightWheel.AimsEncoder=RightWheel.AimsEncoder-AngleError*0.1;
						if(RightWheel.AimsEncoder<RAimEncoder*0.8)//�������
						{
							RightWheel.AimsEncoder=RAimEncoder*0.8;
						}
						ThreeWheel.AimsEncoder=ThreeWheel.AimsEncoder+AngleError*0.1;
						if(ThreeWheel.AimsEncoder>TAimEncoder*1.2)//�������
						{
							ThreeWheel.AimsEncoder=TAimEncoder*1.2;
						}	
					}
				}
			}
			else 				//���
			{
				
			}
			
			//���㳬ʱ
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
			Task6_Step=0;//��ѯ�����е���һ������
			Trun_Flag=0;
			break;
		}
		default:break;
	}
}



/**************************************************************************
�������ܣ�UWBС����λ
��ڲ�������Ҫ��λ����X,Y������ ,��������վ������
����  ֵ����
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
	
	
	 if(PSBKey.GREEN==0)//ң������ɫ����û��
	 {
		Turn_Step=0;
		oldX=0;
		oldY=0;
		result=0;
		return;
	 }
	 switch(Turn_Step)
	 {
		 case 0:  //ȡ��ʼ�Ƕ�
		 {
			 if(UWBData.StableFlag)//�����ȶ�
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
		 case 2:  //�ǶȽ�����ƽ����X��
		 {
			 if(OSTaskStateGet(Task6)==TASK_PAUSING&&Task6_Step==0)//������ɣ�����6�ͻ���ͣ
			 {
				  if(UWBData.StableFlag)//�����ȶ�
					{
						UWBData.StableFlag=0;
						result = atan ((UWBData.Y-oldY)/(UWBData.X-oldX)) * 180 / PI;  //����ת��Ϊ��
						
						//�Ƕ�����
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
		 case 3:  //�ߵ�X�㣬���ٴνǶȽ���
		 {
			 oldY=UWBData.Y;//ȡ����
			 oldX=UWBData.X;
			 if(UWBData.X<x)       //���ڵ�X��С���趨�ģ�ֱ��
			 {
				 SetTurn(Straight,30000,200);
				 Turn_Step=4;
			 }
			 else if(UWBData.X>x)  //���ڵ�X��С���趨�ģ���ת180ֱ�� 
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
		 case 4:   //����X��
		 {
			 if(abs(UWBData.X-x)<20)//����С��20cm
			 {
				 SetTurn(Stop,0,0);
				 //PSBKey.GREEN=0;
				 Turn_Step=5;
			 }
			 break;
		 }
		 case 5:   //�ٴν���
		 {
			 if(OSTaskStateGet(Task6)==TASK_PAUSING&&Task6_Step==0)//������ɣ�����6�ͻ���ͣ
			 {
				  if(UWBData.StableFlag)//�����ȶ�
					{
						UWBData.StableFlag=0;
						result = atan ((UWBData.Y-oldY)/(UWBData.X-oldX)) * 180 / PI;  //����ת��Ϊ��
						
						//�Ƕ�����
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
		 case 6:		//��ת90��ƽ����Y��
		 {
			 if(UWBData.Y<y)       //���ڵ�Y��С���趨�ģ�ֱ��
			 {
				 SetTurn(TurnLeft,90,100);
				 Turn_Step=7;
			 }
			 else if(UWBData.Y>y)  //���ڵ�X��С���趨�ģ���ת180ֱ�� 
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
		 case 7: //��ת���
		 {
			 if(OSTaskStateGet(Task6)==TASK_PAUSING&&Task6_Step==0)//������ɣ�����6�ͻ���ͣ
			 {
				 OmniWheelscontrol(0,200,0,0);
				 LAimEncoder=LeftWheel.AimsEncoder;
				 RAimEncoder=RightWheel.AimsEncoder;
				 TAimEncoder=ThreeWheel.AimsEncoder;
				 Turn_Step=8;
			 }
			 break;
		 }
		 case 8: //ƽ��Y���ߵ��趨��
		 {
			 if(abs(UWBData.Y-y)<20)//����С��20cm
			 {
				 SetTurn(Stop,0,0);
				 PSBKey.GREEN=0;
				 Turn_Step=0;
			 }
			 else
			 {
				 //��X�������������ֱ��
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



