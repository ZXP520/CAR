/**
  ******************************************************************************
  * @file    timer.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   本系统中所有定时器的初始化
  ******************************************************************************
  * @attention 编码器模式用到了4个定时器，PWM输出两个。注意高级定时器PWM输出时的
  * 区别，需要使能朱输出。还有JTG引脚的重定义问题需要注意。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "led.h"
#include "stm32f10x_tim.h"
#include "control.h"	
#include "include.h"
#include "kalman.h"
/**********************************************************
** 函数名:   TIM8_PWM_Init
** 功能描述: 定时器8PWM输出模式初始化
** 输入参数: arr：自动重装值
						 psc：时钟预分频数
** 输出参数: 无
***********************************************************/ 
void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(TIM8_PWM_TIM_CLK, ENABLE);   //增加了I2C: RCC_APB1Periph_I2C1
 	RCC_APB2PeriphClockCmd(TIM8_PWM_PIN_CLK | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能	                                                                     	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM8, ENABLE); //完全重映射 TIM8_CH2->PC6/7/8/9
	
	//设置该引脚为复用输出功能,输出TIM8 CH1\CH2\CH3\CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = TIM8_PWM_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TIM8_PWM_PORT, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH1 PC6
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH2 PC7
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH3 PC8
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH4 PC9
	
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	PB6  
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB7
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB8
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB9
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	
	TIM_SetCompare1(TIM8,0); //左转
	TIM_SetCompare2(TIM8,0);
	TIM_SetCompare3(TIM8,0);  
	TIM_SetCompare4(TIM8,0);	
		
}

	
/**********************************************************
** 函数名:   TIM4_PWM_Init
** 功能描述: 定时器4PWM输出模式初始化
** 输入参数: arr：自动重装值
						 psc：时钟预分频数
** 输出参数: 无
***********************************************************/
void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(TIM4_PWM_TIM_CLK, ENABLE);   //增加了I2C: RCC_APB1Periph_I2C1
 	RCC_APB2PeriphClockCmd(TIM4_PWM_PIN_CLK | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能	                                                                     	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //完全重映射 TIM4_CH2->PB6/7/8/9
	
	//设置该引脚为复用输出功能,输出TIM4 CH1\CH2\CH3\CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = TIM4_PWM_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TIM4_PWM_PORT, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH1 PC6
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH2 PC7
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH3 PC8
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH4 PC9
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH1预装载使能	PB6  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH2预装载使能	PB7
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH2预装载使能	PB8
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH2预装载使能	PB9
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	
	TIM_SetCompare1(TIM4,0); //左转
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);  
	TIM_SetCompare4(TIM4,0);	
		
}
/**********************************************************
** 函数名: TIM1_Configuration
** 功能描述: 定时器1编码器模式初始化函数
** 输入参数: 无
** 输出参数: 无
***********************************************************/ 
void TIM1_Configuration(void)//编码器接口设置TIM1/PA8-B相  PA9-A相
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(TIM1_ENCODER_TIM_CLK, ENABLE);
 	RCC_APB2PeriphClockCmd(TIM1_ENCODER_PIN_CLK, ENABLE); 
	                                                       			
	GPIO_InitStructure.GPIO_Pin = TIM1_ENCODER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
	GPIO_WriteBit(TIM1_ENCODER_PORT, TIM1_ENCODER_PIN,Bit_SET);

	TIM_TimeBaseStructure.TIM_Period = EncoderPeriod; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	
	//设置定时器3位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 8;      
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);     
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
	TIM1->CNT = 0;
	TIM_Cmd(TIM1, ENABLE);
}

/**********************************************************
** 函数名: TIM2_Configuration
** 功能描述: 定时器2编码器模式初始化函数
** 输入参数: 无
** 输出参数: 无
***********************************************************/	 
void TIM2_Configuration(void)//编码器接口设置TIM2/PA15-B相  PB3-A相
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
 
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);  //关闭JTAG接口 开启SWD
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(TIM2_ENCODER_PIN_CLK,ENABLE); 
    RCC_APB2PeriphClockCmd(TIM2_ENCODER_PIN_CLK1,ENABLE);
     
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);       //TIM2引脚重定向 
                                                                                                                                 
    GPIO_InitStructure.GPIO_Pin = TIM2_ENCODER_PIN;             //PA15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(TIM2_ENCODER_PORT, &GPIO_InitStructure);                
    GPIO_WriteBit(TIM2_ENCODER_PORT, TIM2_ENCODER_PIN,Bit_SET);
 
    GPIO_InitStructure.GPIO_Pin = TIM2_ENCODER_PIN1;              //PB3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(TIM2_ENCODER_PORT1, &GPIO_InitStructure);                 
    GPIO_WriteBit(TIM2_ENCODER_PORT1, TIM2_ENCODER_PIN1,Bit_SET);
     
    TIM_DeInit(TIM2);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
    TIM_TimeBaseStructure.TIM_Period = EncoderPeriod; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 
	//设置定时器2为编码器模式  IT1 IT2为上升沿计数
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure); 
    TIM_ICInitStructure.TIM_ICFilter = 6;  //输入滤波器
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);  //清除所有标志位
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //允许中断更新
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);
}

/**********************************************************
** 函数名: TIM3_Configuration
** 功能描述: 定时器3编码器模式初始化函数
** 输入参数: 无
** 输出参数: 无
***********************************************************/ 
void TIM3_Configuration(void)//编码器接口设置TIM3/PA6-A相  PA7-B相
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(TIM3_ENCODER_TIM_CLK, ENABLE);
 	RCC_APB2PeriphClockCmd(TIM3_ENCODER_PIN_CLK, ENABLE);  
	                                                       				
	GPIO_InitStructure.GPIO_Pin = TIM3_ENCODER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   
	GPIO_Init(TIM3_ENCODER_PORT, &GPIO_InitStructure);				 
	GPIO_WriteBit(TIM3_ENCODER_PORT, TIM3_ENCODER_PIN,Bit_SET);
	//GPIO_WriteBit(GPIOA, GPIO_Pin_7,Bit_SET); 

	TIM_TimeBaseStructure.TIM_Period = EncoderPeriod; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	//设置定时器3位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 8;      
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);     
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
	TIM3->CNT = 0;
	TIM_Cmd(TIM3, ENABLE);
}


/**********************************************************
** 函数名: TIM5_Configuration
** 功能描述: 定时器5编码器模式初始化函数
** 输入参数: 无
** 输出参数: 无
***********************************************************/ 
void TIM5_Configuration(void)//编码器接口设置TIM5/PA0-A相  PA1-B相
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(TIM5_ENCODER_TIM_CLK, ENABLE);
 	RCC_APB2PeriphClockCmd(TIM5_ENCODER_PIN_CLK , ENABLE); 
	                                                       				
	GPIO_InitStructure.GPIO_Pin = TIM5_ENCODER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	GPIO_Init(TIM5_ENCODER_PORT, &GPIO_InitStructure);				 
	GPIO_WriteBit(TIM5_ENCODER_PORT, TIM5_ENCODER_PIN,Bit_SET);
	//GPIO_WriteBit(GPIOA, GPIO_Pin_1,Bit_SET); 

	TIM_TimeBaseStructure.TIM_Period = EncoderPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	//设置定时器5位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 8;  
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update); 
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM5->CNT = 0;
	TIM_Cmd(TIM5, ENABLE);
}

/**********************************************************
** 函数名: TIM6_Int_Init
** 功能描述:  基本定时器配置
** 输入参数: 无
** 输出参数: 无
** 说明：定时时间=(预分频数+1）*（计数值+1) /TIM6时钟(72MHz)，单位(s)
   这里溢出时间t=(7200*10000)/72000000s=1s
***********************************************************/
void TIM6_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	NVIC_InitTypeDef           NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //使能TIM6时钟
	
	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);  //根据指定的参数初始化TIMx的时间基数单位
	
	//允许更新中断，触发方式中断
	TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);     //使能指定的TIM6中断,允许更新中断
	//	TIM_ITConfig(TIM6,TIM_IT_Trigger,ENABLE);
	
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM6, ENABLE);     //使能定时器6
}
 
/**********************************************************
** 函数名: TIM6_IRQHandler
** 功能描述: 定时器6的更新中断服务程序
** 输入参数: 无
** 输出参数: 无
***********************************************************/
u32 Time6_Cnt=0;
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		Time6_Cnt++;
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);//清除更新中断标志位
	}
}


/**********************************************************
** 函数名: Time_Config
** 功能描述: 定时器初始化函数
** 输入参数: 无
** 输出参数: 无
***********************************************************/

void Time_Config(void)
{
	TIM1_Configuration();
	TIM2_Configuration();
	TIM3_Configuration();
	TIM4_PWM_Init(TIM8_Period-1,30-1);  //分频。PWM频率=72000/5/1200=12Khz
	TIM5_Configuration();
	//TIM6_Int_Init(72-1,100);           //100us	
	TIM8_PWM_Init(TIM8_Period-1,30-1);	//分频。PWM频率=72000/5/1200=12Khz
}



