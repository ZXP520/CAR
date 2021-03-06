#ifndef __CONTROLTURN_H
#define __CONTROLTURN_H
#include "sys.h"
  /**************************************************************************
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//运动状态枚举
typedef enum 
{
	Straight=1,
	Back,
	TurnLeft,
	TurnRight,
	Stop,
}MoveDire;//查询命令
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//设置运动状态  可调用
void SetTurn(int dire,int angle,int speed);
void AllControlTrun(void);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void ControlTrun(s8 speed,u16 angle);
void ControlStraight(int speed,int settime);
void UWBTurnToX(double x,double y,double x1,double y1,double x2,double y2,double x3,double y3);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
