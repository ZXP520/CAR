#ifndef __INCLUDE_H
#define __INCLUDE_H
#include "sys.h"
#include "stm32f10x_tim.h"
#include "adc.h"
#include "stdio.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//版本  0-差速，1-全向，2-四轮；
//版本定义
#define  VERSION     1
//PID控制使能
#define PID_ENABLE   1

extern void Task1(void); //ÈÎÎñ1ÉùÃ÷
extern void Task2(void); //ÈÎÎñ2ÉùÃ÷
extern void Task3(void); //ÈÎÎñ3ÉùÃ÷
extern void Task4(void); //ÈÎÎñ4ÉùÃ÷
extern void Task5(void); //ÈÎÎñ5ÉùÃ÷
extern void Task6(void); //ÈÎÎñ6ÉùÃ÷
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif



