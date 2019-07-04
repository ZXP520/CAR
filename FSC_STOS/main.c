/**
  ******************************************************************************
  * @file    main.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   ����ϵͳ�������������ڸ����ܵ�ʵ��
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fsc_stos.h"  //ʹ�ö������ں�
#include "sys.h"
#include "timer.h"
#include "include.h"
#include "led.h"
#include "runcontrol.h"
#include "control.h"
#include "gy85.h"
#include "kalman.h"
#include "dealdata.h"
#include "stm32f10x_it.h" 
#include "Encoder.h"
#include "bsp_usart.h"
#include "errordetect.h"
#include "myiic.h"
#include "controlturn.h"	
#include <math.h>


/******************************�����������*************************/
void Task1(void); //����1����
void Task2(void); //����2����
void Task3(void); //����3����
void Task4(void); //����4����
void Task5(void); //����5����
void Task6(void); //����6����

#define Task1_StkSize 128       //����1��ջ��С����С���⣩
#define Task2_StkSize 128       //����2��ջ��С (��ͬ�����С���Բ�һ��)
#define Task3_StkSize 128       //����3��ջ��С (Ӧ���ݾ��������С����)
#define Task4_StkSize 128       //����4��ջ��С������ú����й����в��ɸı䣩
#define Task5_StkSize 128       //����5��ջ��С
#define Task6_StkSize 128       //����6��ջ��С

__align(8) OS_STK Task1_Stk[Task1_StkSize]; //����1��ջ
__align(8) OS_STK Task2_Stk[Task2_StkSize]; //����2��ջ
__align(8) OS_STK Task3_Stk[Task3_StkSize]; //����3��ջ
__align(8) OS_STK Task4_Stk[Task4_StkSize]; //����4��ջ
__align(8) OS_STK Task5_Stk[Task5_StkSize]; //����5��ջ
__align(8) OS_STK Task6_Stk[Task6_StkSize]; //����6��ջ
/*******************************************************************/
/***************************************�û��������񲢼����ں�����***************************************/
int main(void)
{     
	//ȫ�ֳ�ʹ��,�Ƽ�����������ʹ�õ��ĳ�ʹ�����ڴ˴���task�����õ��ĳ�ʹ������task��
	/************************************************************************************/	
	/*------------------ȫ�ֳ�ʹ����-------------------*/
	NVIC_Configuration(); 	 			  
	USART2_Config(115200);
	USART3_Config(115200);
	Time_Config();							
	InitGY85();
	Init_PID();
	Adc_Init();
	Init_ErrorDetect();
	
	//LeftWheelSpeedSet(196);
	//RightWheelSpeedSet(196);/
	//ThreeWheelSpeedSet(196);
	//FourWheelSpeedSet(196);
	//SetLeft_Pwm(0,1);
	//SetRight_Pwm(400,1);
	//SetThree_Pwm(400,1);
	//SetFour_Pwm(400,1);
	//OmniWheelscontrol(0,100,0,0);
		

	  
	
	/************************************************************************************/	
    OSInit(); //ϵͳ��ʹ��
	/********************************��ϵͳ�д�������***********************************/
    OSTaskCreate("Task1",Task1,(OS_STK*)&Task1_Stk[Task1_StkSize-1],TASK_RUNNING); //��������1
    OSTaskCreate("Task2",Task2,(OS_STK*)&Task2_Stk[Task2_StkSize-1],TASK_RUNNING); //��������2
    OSTaskCreate("Task3",Task3,(OS_STK*)&Task3_Stk[Task3_StkSize-1],TASK_RUNNING); //��������3 ������ͣ
	OSTaskCreate("Task4",Task4,(OS_STK*)&Task4_Stk[Task4_StkSize-1],TASK_RUNNING); //��������4
	OSTaskCreate("Task5",Task5,(OS_STK*)&Task5_Stk[Task5_StkSize-1],TASK_RUNNING); //��������5 ������ͣ
	OSTaskCreate("Task6",Task6,(OS_STK*)&Task6_Stk[Task6_StkSize-1],TASK_PAUSING); //��������6 
	/***********************************************************************************/
    OSStart();//OS��ʼ����
}

/*********************************�û�����ʵ�������************************************/

/**************************************************************************
����1  �õ����������� 
**************************************************************************/
void Task1(void)
{ 	
	static u16 time_cnt=0;
	while(1) 
	{
		//PID����Ӧ�÷ŵ��ж��е���	 
		Get_Encoder();
		if(time_cnt<100)
		{
		 time_cnt++;
		}
		else
		{
		  UWBTurnToX(50,50,0,0,0,6000,2400,0);
		}
	OS_delayMs(10); 
	}	
}

/**************************************************************************
����2  PID����
**************************************************************************/
void Task2(void)  
{
	while(1) 
	 {
		//PID����ʹ����
		#if PID_ENABLE==1 
		OSSchedLock();         //�����л����� 
		RunWheelcontrol();
		OSSchedUnlock();
		#endif	 
		OS_delayMs(10);			
	 }			
}

/**************************************************************************
����3  ����2�������ݸ�ROS
**************************************************************************/
void Task3(void)
{	
	static u8 flag=1;
	while(1) 
	{	 
		//Ӳ����ʼ���ɹ��򴮿ڷ������������ݶ�ȡƵ��Ϊ50hz
		SendEncoderAndIMU20Ms(DealData_Rx.Hardware_Init);	
		if(flag)
		{
			
			//SetTurn(TurnRight,120,100);
			//SetTurn(Straight,5000,200);
			//SetTurn(TurnLeft,120,100);
			//SetTurn(TurnRight,360,100);
			//SetTurn(Straight,5000,200);
			//SetTurn(TurnRight,90,100);
			//SetTurn(TurnLeft,90,100);
			
			flag=0;
		}
		OS_delayMs(5);				 
	 }			
}

/**************************************************************************
����4  ����3��ӡ�����Ҽ��Ӳ����ʼ��
**************************************************************************/
void Task4(void) //����4  
{
	while(1) 
	 {		
		 //�������5������ͣ״̬����Ӳ����ʼ���ɹ�����������5
		 if(OSTaskStateGet(Task5)==TASK_PAUSING && DealData_Rx.Hardware_Init)
		 {
			 OSTaskStateSet(Task5,TASK_RUNNING);
		 }
		 
		 Get_PowerData();//�������
		 
		 if(OSTaskStateGet(Task6)==TASK_PAUSING)
		 {
			 //u3_printf("N:%.2f\n",ImuData.Yaw);
		 }
		 u3_printf("%.2f	%.2f	%.2f	%.2f	%.2f	%.2f\n",Angle_accX,Angle_accY,angle2,Gx,Gz,ImuData.Yaw);
		 OS_delayMs(500); 			 //500ms����һ��
	 }
}

/**************************************************************************
����5  100MS���ϼ�⣬���й���1S�ϴ�һ��
**************************************************************************/
void Task5(void)  
{
	while(1) 
	{	
		//���ϼ��
		ErrorDetect();
		OS_delayMs(100); 				//100Ms��һ��
	}
}

/**************************************************************************
����6 5MSת�ǵȿ��� �˶���������
**************************************************************************/
void Task6(void)  
{
	while(1) 
	{	
		AllControlTrun();
		OS_delayMs(5); 				//5Ms��һ��
	}
}
/********************************************************************************************/

