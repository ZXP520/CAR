#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************

**************************************************************************/
#define VERSIONNUMBER   (s16)0x0100  //版本1.00
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define PI 3.14159265
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#if		VERSION==0
	#define Wheel_D          	67   //mm轮子直径
	#define Wheel_SPACING		  263  //mm轮间距
#elif VERSION==1
	#define Wheel_D          	59   //mm轮子直径
	#define Wheel_SPACING		  157  //mm轮间距
#endif
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define Wheel_RATIO     	56   //减速比
#define ENCODER_LINE     	11   //编码器线数
#define SPEED_TO_ENCODER  (float)(4*Wheel_RATIO*ENCODER_LINE/(Wheel_D*PI*100))				//速度转编码脉冲  (个/10ms)(4为编码器模式，一个脉冲四个计数)(连续除应该用括号改成乘以，不然会得0)
#define TIM8_Period  1200			 //TIME8重装值
#define MAXSPEED     500       //最大速度mm/s
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef struct
{
  _Bool 	Direct;		//方向
  int 	AimsEncoder;//目标脉冲数
	int 	MotoPwm;		//轮子PWM
	s16   NowSpeed;		//轮子当前速度
	s16   AimSpeed;   //轮子目标速度
	
	//整个轮子的状态AllWheel
	//u8    stop_flag;  //停止标志
	union
	{
			struct bit_feild
			{
					char bit0: 1;  //急停标志
					char bit1: 1;  //前左轮
					char bit2: 1;  //前右轮
					char bit3: 1;  //左轮
					char bit4: 1;  //右轮
					char bit5: 1;  //陀螺仪
					char bit6: 1;  //加速度计
					char bit7: 1;  //磁力计
					char bit8: 1;  //电池电压
					char bit9: 1;
					char bit10:1;
					char bit11:1;
					char bit12:1;
					char bit13:1;
					char bit14:1;
					char bit15:1;
			} bits;
			s16 data;
	}Erroe_flag;
	u8    Electricity;    //电量
	
}Wheel;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*曾量式PID算法，接口参数结构类型*/
typedef struct
{
	/*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float errLast;//上次的误差
 float ctrOut;//控制量输出
	
}PID_AddType;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*绝对式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float ctrOut;//控制量输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern Wheel LeftWheel,RightWheel,ThreeWheel,FourWheel,AllWheel;//定义左右轮结构体
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Init_PID(void);
void PID_AbsoluteMode(PID_AbsoluteType* PID);
void RunWheelcontrol(void);
void SetLeft_Pwm(int moto,u8 mode);
void SetRight_Pwm(int moto,u8 mode);
void SetThree_Pwm(int moto,u8 mode);
void SetFour_Pwm(int moto,u8 mode);
void Xianfu_Pwm(void);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static int myabs(int a);
static int LeftIncremental_PI (int Encoder,int Target);
static int RightIncremental_PI (int Encoder,int Target);
static int ThreeIncremental_PI (int Encoder,int Target);
static int FourIncremental_PI (int Encoder,int Target);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//左轮速度设置
void LeftWheelSpeedSet(int speed);
//右轮速度设置
void  RightWheelSpeedSet(int speed);
//三轮速度设置
void  ThreeWheelSpeedSet(int speed);
//四轮速度设置
void  FourWheelSpeedSet(int speed);
//三轮全向轮运动控制
void OmniWheelscontrol(s16 Vx,s16 Vy,s16 W,s16 a);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
