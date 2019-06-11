/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
1.  系统提供精确小延时函数:
    delay_us(); 用于两条语句代码之间的微秒级延时,单位:us微秒 (系统锁定切换直到延时完成)
	  delay_ms(); 用于两条语句代码之间的毫秒级延时,单位:ms毫秒 (系统锁定切换直到延时完成)
    大延时函数
    OS_delayMs(); 用于任务内两个函数之间的延时,单位:ms毫秒  （调用后立即切换至其他任务直到延时间到立即被系统切换回来）
		
		如要在其他c文件中使用延时函数，复制以下声明到c文件的开头处,或在c文件里包含FSC_STOS.h头文件
		void delay_us(unsigned int nus);
		void delay_ms(unsigned int nms);		

2.添加任务：
	*添加步骤：(以添加Task13为例,任务名称任意，只要对应的所有名称一致即可。这里为方便取名为Task13)
	*1.void Task13(void);//任务13声明 
  *2.#define Task13_StkSize 128       //任务13堆栈大小
  *3.OS_STK Task13_Stk[Task13_StkSize];//任务13堆栈
	*4.OSTaskCreate("Task13",Task13,(OS_STK*)&Task13_Stk[Task13_StkSize-1],TASK_RUNNING);//在main()函数里创建新任务，堆栈大小>=64,视具体任务内代码决定
	*5.void Task13(void) //任务13
		{
			 while(1) 
			 {		
				 //printf("Task13 is running\r\n");//示例代码，使用时删除	
				 //OS_delayMs(1000); 				       //示例代码，使用时删除	
			 }
		}

************************************************/

//(双击fsc_stos，右键->Open document "fsc_stos.h"即可打开，其他同理 )
#include "fsc_stos.h"  //使用多任务内核

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


/******************************创建任务参数*************************/
void Task1(void); //任务1声明
void Task2(void); //任务2声明
void Task3(void); //任务3声明
void Task4(void); //任务4声明
void Task5(void); //任务5声明
void Task6(void); //任务6声明

#define Task1_StkSize 128       //任务1堆栈大小（大小任意）
#define Task2_StkSize 128       //任务2堆栈大小 (不同任务大小可以不一致)
#define Task3_StkSize 128       //任务3堆栈大小 (应根据具体任务大小设置)
#define Task4_StkSize 128       //任务4堆栈大小（分配好后运行过程中不可改变）
#define Task5_StkSize 128       //任务5堆栈大小
#define Task6_StkSize 128       //任务6堆栈大小

__align(8) OS_STK Task1_Stk[Task1_StkSize]; //任务1堆栈
__align(8) OS_STK Task2_Stk[Task2_StkSize]; //任务2堆栈
__align(8) OS_STK Task3_Stk[Task3_StkSize]; //任务3堆栈
__align(8) OS_STK Task4_Stk[Task4_StkSize]; //任务4堆栈
__align(8) OS_STK Task5_Stk[Task5_StkSize]; //任务5堆栈
__align(8) OS_STK Task6_Stk[Task6_StkSize]; //任务6堆栈
/*******************************************************************/
/***************************************用户创建任务并加入内核运行***************************************/
int main(void)
{     
	  //全局初使化,推荐把所有任务都使用到的初使化放在此处，task独立用到的初使化放在task内
    /************************************************************************************/	
	  /*------------------全局初使化区-------------------*/
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
    OSInit(); //系统初使化
	  /********************************在系统中创建任务***********************************/
    OSTaskCreate("Task1",Task1,(OS_STK*)&Task1_Stk[Task1_StkSize-1],TASK_RUNNING); //创建任务1
    OSTaskCreate("Task2",Task2,(OS_STK*)&Task2_Stk[Task2_StkSize-1],TASK_RUNNING); //创建任务2
    OSTaskCreate("Task3",Task3,(OS_STK*)&Task3_Stk[Task3_StkSize-1],TASK_RUNNING); //创建任务3 任务暂停
	  OSTaskCreate("Task4",Task4,(OS_STK*)&Task4_Stk[Task4_StkSize-1],TASK_RUNNING); //创建任务4
		OSTaskCreate("Task5",Task5,(OS_STK*)&Task5_Stk[Task5_StkSize-1],TASK_PAUSING); //创建任务5 任务暂停
		OSTaskCreate("Task6",Task6,(OS_STK*)&Task6_Stk[Task6_StkSize-1],TASK_PAUSING); //创建任务6 
	  /***********************************************************************************/
    OSStart();//OS开始运行
}
/***********************************************************************************/
/*把void taskX(void)看成普通的int main(void)使用即可,这里相当于多个main()在轮流运行*/

/****************************************用户全局变量及宏定义区*****************************************/

/*******************************************************************************************************/
/*********************************用户任务实体代码区************************************/

/**************************************************************************
任务1  得到编码器数据 
**************************************************************************/
void Task1(void)
{ 	
	while(1) 
	 {
		 //PID控制应该放到中断中调速	 
		 Get_Encoder();
  	 OS_delayMs(10); 
	 }	
}

/**************************************************************************
任务2  PID调速
**************************************************************************/
void Task2(void)  
{
	while(1) 
	 {
		 //PID控制使能了
			#if PID_ENABLE==1 
			OSSchedLock();         //任务切换上锁 
			RunWheelcontrol();
		  OSSchedUnlock();
			#endif	 
  	  OS_delayMs(10);			
	 }			
}

/**************************************************************************
任务3  串口2发送数据给ROS
**************************************************************************/
void Task3(void)
{	
	static u8 flag=1;
	while(1) 
	 {	 
		 //硬件初始化成功则串口发送陀螺仪数据读取频率为50hz
		 SendEncoderAndIMU20Ms(DealData_Rx.Hardware_Init);	
			if(flag)
			{
				SetTurn(TurnRight,120,100);
				SetTurn(Straight,5000,200);
				SetTurn(TurnLeft,120,100);
				SetTurn(TurnRight,360,100);
				SetTurn(Straight,5000,200);
				SetTurn(TurnRight,90,100);
				SetTurn(TurnLeft,90,100);
				
				flag=0;
			}
  	 OS_delayMs(5);				 
	 }			
}

/**************************************************************************
任务4  串口3打印数据且检测硬件初始化
**************************************************************************/
void Task4(void) //任务4  
{
	while(1) 
	 {		
		 //如果任务5处于暂停状态，且硬件初始化成功，则开启任务5
		 if(OSTaskStateGet(Task5)==TASK_PAUSING && DealData_Rx.Hardware_Init)
		 {
			 OSTaskStateSet(Task5,TASK_RUNNING);
		 }
		 
		 Get_PowerData();//计算电量
		 
		 if(OSTaskStateGet(Task6)==TASK_PAUSING)
		 {
			 //u3_printf("N:%.2f\n",ImuData.Yaw);
		 }
		 //u3_printf("%.2f	%.2f	%.2f	%.2f	%.2f	%.2f\n",Angle_accX,Angle_accY,angle2,Gx,Gy,Gz);
  	 OS_delayMs(500); 			 //500ms进入一次
	 }
}

/**************************************************************************
任务5  100MS故障检测，如有故障1S上传一次
**************************************************************************/
void Task5(void)  
{
	while(1) 
	 {	
		 //故障检测
		 //ErrorDetect();
     OS_delayMs(100); 				//100Ms进一次
	 }
}

/**************************************************************************
任务6 5MS转角等控制 运动控制任务
**************************************************************************/
void Task6(void)  
{
	while(1) 
	 {	
		 AllControlTrun();
     OS_delayMs(5); 				//5Ms进一次
	 }
}
/********************************************************************************************/

