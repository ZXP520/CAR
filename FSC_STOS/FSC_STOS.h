#ifndef _FSC_STOS_H_
#define _FSC_STOS_H_

/****************************如果不是stm32f1xx,请改为对应芯片的头文件*******************************************/
#include "stm32f10x.h"
//#include "stm32f4xx.h"
/***************************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>          
/****************************************用户可自定义***********************************************************/

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define OS_CLOCK_TIME       2000     //任务时间切片,每个任务持续运行的时间，单位:微秒 us  
#define OS_MAX_TASKS        16       //任务数=用户任务数+2  任务数:3-255  用户根据实际需要的任务数量修改
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIMER_SIZE          8       //系统虚拟定时器数量(任意值) 
#define FLAG_SIZE           8       //标志数量(任意值) 
#define MUTEX_SIZE          8       //互斥数量(任意值) 
#define MBOX_SIZE           8       //邮箱数量(任意值) 
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/***************************************************************************************************************/

#define TASK_NAME_SIZE          32  //任务名       字符最大长度
#define OS_PERIP_USART_BUFF_LEN 32  //系统指令     字符最大长度
#define OS_CMD_LEVEL            8   //系统指令层数

#define TaskIDLE_StkSize        16  //空闲任务堆栈大小
#define TaskManage_StkSize      128 //任务管理器任务堆栈大小

#define TASK_CREATING    0           //创建态
#define TASK_RUNNING     1           //运行态
#define TASK_PAUSING     2           //暂停态
#define TASK_BACKRUNNING 3           //后台运行态
#define TASK_DELETING    4           //删除态
                                   
#define OS_FALSE 0                  //假
#define OS_TRUE  1                  //真

typedef unsigned char  INT8U;            
typedef unsigned short INT16U;           
typedef unsigned int   INT32U;           
typedef unsigned int   OS_STK;

typedef struct
{
	INT32U Timer[TIMER_SIZE];      //虚拟定时器
	INT8U  FLAG[FLAG_SIZE];        //标志量
  INT8U  MUTEX[MUTEX_SIZE];      //互斥量
	INT32U *MBOX[MBOX_SIZE];       //邮箱
}OS_SYS;
extern OS_SYS OS_Sys;            //系统功能结构体(供用户使用)

typedef struct
{
	INT8U  OS_USART_COUNT;
	char   OS_USART_RX_BUFF[OS_PERIP_USART_BUFF_LEN];	
}OS_PERIP;
extern OS_PERIP OS_Perip;        //系统串口指令接收(用户不可用)

typedef struct
{
	INT8U  OSRunning;
	INT8U  OSIntNesting; 
	INT8U  OSLockNesting; 
	INT32U OSCtxSwCtr; 
	INT16U OSTaskNext;
	INT8U  OSTaskNextRunFlag;
	INT8U  OSTaskSwitchBack;
	INT32U TaskTickLeft;
	INT32U TaskTimeSliceCount; 
}OS_SYSTEM;
extern OS_SYSTEM OS_System; 

typedef struct 
{
	OS_STK    *OSTCBStkPtr;  
  INT32U    OSTCBTaskAdd;	
	INT16U    OSTCBTaskNum; 
	char      OSTCBTaskStr[TASK_NAME_SIZE];
	INT8U     OSTCBStat;
	INT32U    OSTCBTaskTSCnt;     
  INT32U    OSTCBTaskCPUOccRate;  
} OS_TCB; 
extern OS_TCB OSTCBTbl[OS_MAX_TASKS];  
extern OS_TCB *OSTCBRun; 
extern OS_TCB *OSTCBCur; 
extern OS_TCB *OSTCBNext;

extern u32 SysTimeCnt;

/***************************************此处有注释的函数用户可任意调用****************************************/                          
void OSStart(void); 
void OSInit(void);
void OSCtxSw(void); 
void OSTaskCreate(char* taskstr,void (*task),OS_STK *p_tos,INT8U taskstate); //任务创建函数(仅在main函数中使用)
INT8U OSTaskStateGet(void* Taskx); //获取任务状态
void OSTaskStateSet(void* Taskx,INT8U TaskState); //任务状态设置,参数1：任务各， 参数2：任务状态
void OSTaskSwitchBack(void* Taskx); //任务跳转  带返回
void OSTaskSwitch(void* Taskx); //任务跳转  不带返回
void OSSchedLock(void);                      //任务切换上锁函数(有的代码必须一次性完成，中途不能被切换出去,代码处理完后必须调用OSSchedUnlock()解除)  
void OSSchedUnlock(void);                    //任务切换解锁函数
void OSIntEnter(void);					             //进入中断时调用  
void OSIntExit(void);	                       //退出中断时调用    
void OSFlagPost(uint16_t FNum);              //发送标志量
void OSFlagPend(uint16_t FNum);              //等待标志量
void OSMutexPost(uint16_t MNum);             //发送互斥量
void OSMutexPend(uint16_t MNum);             //等待互斥量
void OSMboxPost(uint16_t MNum,void* fp);     //发送邮件(地址)
void* OSMboxPend(uint16_t MNum);             //等待邮件
INT32U GetOSSliceTime(void);                 //获取OS节拍数
INT32U GetOSTimerVal(void);                  //获取系统定时器当前计数值(默认为9MHz，每次计数用时1/9 us)
void delay_us(INT32U nus);                   //微秒延时函数
void delay_ms(INT32U nms);	                 //毫秒延时函数
void OS_delayMs(volatile INT32U nms);        //任务延时函数
/****************************************************************************************************************/

#endif

