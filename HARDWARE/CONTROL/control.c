#include "control.h"		
#include "sys.h"
#include "include.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include <math.h>
#include "Encoder.h"
#include "errordetect.h"
#include "fsc_stos.h"

Wheel LeftWheel,RightWheel,ThreeWheel,FourWheel,AllWheel;//定义左右轮结构体

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : 左轮速度设置
* Input          : 左轮速度 
* Output         : None
* Return         : None 
****************************************************************************** */
void LeftWheelSpeedSet(int speed)
{
//左轮反向 差速
#if	VERSION==0
	  speed=-speed;
#endif
	if(speed>=0)//正方向
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		LeftWheel.Direct=1;
	}
	else        //反方向
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		LeftWheel.Direct=0;
	}	
	LeftWheel.AimSpeed=speed;   //目标速度
	LeftWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5四舍五入
}

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : 右轮速度设置
* Input          : 右轮速度 
* Output         : None
* Return         : None 
****************************************************************************** */
void  RightWheelSpeedSet(int speed)
{
	if(speed>=0)//正方向
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		RightWheel.Direct=1;
	}
	else        //反方向
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		RightWheel.Direct=0;
	}
	RightWheel.AimSpeed=speed;
	RightWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5四舍五入
}

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : 三轮速度设置
* Input          : 三轮速度 
* Output         : None
* Return         : None 
****************************************************************************** */
void  ThreeWheelSpeedSet(int speed)
{
	if(speed>=0)//正方向
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		ThreeWheel.Direct=1;
	}
	else        //反方向
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		ThreeWheel.Direct=0;
	}
	ThreeWheel.AimSpeed=speed;
	ThreeWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5四舍五入
}


/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : 四轮速度设置
* Input          : 四轮速度 
* Output         : None
* Return         : None 
****************************************************************************** */
void  FourWheelSpeedSet(int speed)
{
	if(speed>=0)//正方向
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		FourWheel.Direct=1;
	}
	else        //反方向
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		FourWheel.Direct=0;
	}
	FourWheel.AimSpeed=speed;
	FourWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5四舍五入
}

/**************************************************************************
函数功能：PID运动控制
					10ms进一次
					在任务2中调用
**************************************************************************/
void RunWheelcontrol(void)
{	
	float temp=0;
	
	//停车标志 进入抱死状态
	if(AllWheel.Erroe_flag.bits.bit0||(LeftWheel.AimSpeed==0&&RightWheel.AimSpeed==0&&ThreeWheel.AimSpeed==0&&FourWheel.AimSpeed==0))
	{
		TIM_SetCompare1(TIM8,0); 
		TIM_SetCompare2(TIM8,0);
		TIM_SetCompare3(TIM8,0); 
		TIM_SetCompare4(TIM8,0); 
		
		TIM_SetCompare1(TIM4,0); 
		TIM_SetCompare2(TIM4,0); 
		TIM_SetCompare3(TIM4,0); 
		TIM_SetCompare4(TIM4,0); 
		return;
	}
	
	//获得PID调速后的PWM
	LeftWheel.MotoPwm =myabs( LeftIncremental_PI(abs(GetEncoder.V3) ,LeftWheel.AimsEncoder ));//获得PID调速后的PWM
	RightWheel.MotoPwm=myabs(RightIncremental_PI(abs(GetEncoder.V1) ,RightWheel.AimsEncoder));
	ThreeWheel.MotoPwm=myabs(ThreeIncremental_PI(abs(GetEncoder.V5) ,ThreeWheel.AimsEncoder));
	FourWheel.MotoPwm =myabs( FourIncremental_PI(abs(GetEncoder.V2) ,ThreeWheel.AimsEncoder));
	
	Xianfu_Pwm();//限幅

	//设置PWM与方向
	SetLeft_Pwm (LeftWheel.MotoPwm  ,LeftWheel.Direct );
	SetRight_Pwm(RightWheel.MotoPwm ,RightWheel.Direct);
	SetThree_Pwm(ThreeWheel.MotoPwm ,ThreeWheel.Direct);
	SetFour_Pwm (FourWheel.MotoPwm  ,FourWheel.Direct );
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM mode(1为前进， 0为后退)
返回  值：无
**************************************************************************/
void SetLeft_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare1(TIM8,TIM8_Period); 
		TIM_SetCompare2(TIM8,TIM8_Period-moto); 
	}
	else
	{
		TIM_SetCompare1(TIM8,TIM8_Period-moto); 
		TIM_SetCompare2(TIM8,TIM8_Period); 
	}
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM mode(1为前进， 0为后退)
返回  值：无
**************************************************************************/
void SetRight_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare3(TIM8,TIM8_Period); 
		TIM_SetCompare4(TIM8,TIM8_Period-moto); 
		
		
	}
	else
	{	
		TIM_SetCompare3(TIM8,TIM8_Period-moto); 
		TIM_SetCompare4(TIM8,TIM8_Period); 
	}
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM mode(1为前进， 0为后退)
返回  值：无
**************************************************************************/
void SetThree_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare1(TIM4,TIM8_Period); 
		TIM_SetCompare2(TIM4,TIM8_Period-moto); 
		
	}
	else
	{	
		TIM_SetCompare1(TIM4,TIM8_Period-moto); 
		TIM_SetCompare2(TIM4,TIM8_Period); 
	}
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM mode(1为前进， 0为后退)
返回  值：无
**************************************************************************/
void SetFour_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare3(TIM4,TIM8_Period); 
		TIM_SetCompare4(TIM4,TIM8_Period-moto); 
	}
	else
	{	
		TIM_SetCompare3(TIM4,TIM8_Period-moto); 
		TIM_SetCompare4(TIM4,TIM8_Period); 
	}
}



/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  //TIM8_Period=1200;    //===PWM满幅是1200 限制在1200
	  if(LeftWheel.MotoPwm<-TIM8_Period)  LeftWheel.MotoPwm=-TIM8_Period;	
		if(LeftWheel.MotoPwm>TIM8_Period)   LeftWheel.MotoPwm=TIM8_Period;	

		if(RightWheel.MotoPwm<-TIM8_Period) RightWheel.MotoPwm=-TIM8_Period;	
		if(RightWheel.MotoPwm>TIM8_Period)  RightWheel.MotoPwm=TIM8_Period;	
	
		if(ThreeWheel.MotoPwm<-TIM8_Period) ThreeWheel.MotoPwm=-TIM8_Period;	
		if(ThreeWheel.MotoPwm>TIM8_Period)  ThreeWheel.MotoPwm=TIM8_Period;	
	
		if(FourWheel.MotoPwm<-TIM8_Period) FourWheel.MotoPwm=-TIM8_Period;	
		if(FourWheel.MotoPwm>TIM8_Period)  FourWheel.MotoPwm=TIM8_Period;	
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
static int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=0;//temp=-a;  
	  else temp=a;
	  return temp;
}

float Amplitude_PKP=20,Amplitude_PKI=0.1,Amplitude_PKD=25,Amplitude_VKP=2,Amplitude_VKI=3; //PID调试相关参数
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

PID_AddType LeftPID,RightPID,ThreePID,FourPID;

//PID参数初始化
void Init_PID(void)
{
	LeftPID.kp=120;
	LeftPID.ki=3;
	
	RightPID.kp=120;
	RightPID.ki=3;
	
	ThreePID.kp=120;
	ThreePID.ki=3;
	
	FourPID.kp=120;
	FourPID.ki=3;
}

//左PID

static int LeftIncremental_PI (int Encoder,int Target)
{ 	
	LeftPID.errNow=Target-Encoder;  																												//计算偏差
	LeftPID.ctrOut+=LeftPID.kp*(LeftPID.errNow-LeftPID.errLast)+LeftPID.ki*LeftPID.errNow;	//增量式PI控制器
	LeftPID.errLast=LeftPID.errNow;																													//保存上一次偏差 
	if(Target==0){LeftPID.ctrOut=0;}
	if(LeftPID.ctrOut>TIM8_Period){LeftPID.ctrOut=TIM8_Period;}
	else if(LeftPID.ctrOut<0){LeftPID.ctrOut=0;}
	return LeftPID.ctrOut;																																	//增量输出
}

//右PID
static int RightIncremental_PI (int Encoder,int Target)
{ 	
	RightPID.errNow=Target-Encoder;  																												//计算偏差
	RightPID.ctrOut+=RightPID.kp*(RightPID.errNow-RightPID.errLast)+RightPID.ki*RightPID.errNow;	//增量式PI控制器
	RightPID.errLast=RightPID.errNow;																													//保存上一次偏差 
	if(Target==0){RightPID.ctrOut=0;}
	if(RightPID.ctrOut>TIM8_Period){RightPID.ctrOut=TIM8_Period;}
	else if(RightPID.ctrOut<0){RightPID.ctrOut=0;}
	return RightPID.ctrOut;																																	//增量输出
}

//三PID
static int ThreeIncremental_PI (int Encoder,int Target)
{ 	
	ThreePID.errNow=Target-Encoder;  																												//计算偏差
	ThreePID.ctrOut+=ThreePID.kp*(ThreePID.errNow-ThreePID.errLast)+ThreePID.ki*ThreePID.errNow;	//增量式PI控制器
	ThreePID.errLast=ThreePID.errNow;																													//保存上一次偏差 
	if(Target==0){ThreePID.ctrOut=0;}
	if(ThreePID.ctrOut>TIM8_Period){ThreePID.ctrOut=TIM8_Period;}
	else if(ThreePID.ctrOut<0){ThreePID.ctrOut=0;}
	return ThreePID.ctrOut;																																	//增量输出
}

//四PID
static int FourIncremental_PI (int Encoder,int Target)
{ 	

	FourPID.errNow=Target-Encoder;  																												//计算偏差
	FourPID.ctrOut+=FourPID.kp*(FourPID.errNow-FourPID.errLast)+FourPID.ki*FourPID.errNow;	//增量式PI控制器
	FourPID.errLast=FourPID.errNow;																													//保存上一次偏差 
	if(Target==0){FourPID.ctrOut=0;}
	if(FourPID.ctrOut>TIM8_Period){FourPID.ctrOut=TIM8_Period;}
	else if(FourPID.ctrOut<0){FourPID.ctrOut=0;}
	return FourPID.ctrOut;																																	//增量输出
	
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
float Position_KP=20,Position_KI=2,Position_KD=0;      //PID系数
int Position_PID (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}


//绝对式PID算法
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //读取现在的误差，用于kp控制

 PID->errI += PID->errNow; //误差积分，用于ki控制

 if(PID->errILim != 0)	   //微分上限和下限
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

 PID->errOld = PID->errNow;	//保存现在的误差
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

}



//全向轮运动控制
/*
		Va     					cos@									sin@					L					Vx
		Vb  =	 -cos60cos@+sin60sin@		-cos60sin@-sin60cos@	L			*		Vy
		Vc		 -sin30cos@+cos30sin@		-sin30sin@+cos30cos@	L					 W

		其中@为小车坐标系与世界坐标系的夹角    
		W为小车自身的角速度
		Vx为X轴速度
		Vy为Y轴速度

		Va为a轮子速度
		Vb为b轮子速度
		Vc为c轮子速度
		L为轮子到中心的距离
    顺时针为正
*/
//#define  L 157 //轮子到中心的距离
//旋转顺时针为正
void OmniWheelscontrol(s16 Vx,s16 Vy,s16 W,s16 a)
{
	static double Va,Vb,Vc,DW;
	Vx=-Vx;
	DW=-W/100;//数据放大了100倍
	//DW=-W;
	Va=Vx*cos(a)+Vy*sin(a)+DW*Wheel_SPACING;
	Vb=Vx*(-cos(PI/3)*cos(a)+sin(PI/6)*sin(a))+Vy*(-cos(PI/3)*sin(a)-sin(PI/3)*cos(a))+DW*Wheel_SPACING;
	Vc=Vx*(-sin(PI/6)*cos(a)+cos(PI/6)*sin(a))+Vy*(-sin(PI/6)*sin(a)+cos(PI/6)*cos(a))+DW*Wheel_SPACING;
	
	LeftWheelSpeedSet ( Va);
	RightWheelSpeedSet( Vb);
	ThreeWheelSpeedSet( Vc);

	//逆时针转动
	//LeftWheelSpeedSet( 200);
	//RightWheelSpeedSet(200);
	//ThreeWheelSpeedSet(200);
}











