#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   

#define MOTORB_ENCODER_TIM_CLK  	RCC_APB1Periph_TIM3
#define MOTORB_ENCODER_CLK				RCC_APB2Periph_GPIOA
#define MOTORB_ENCODER_PIN        GPIO_Pin_6
#define MOTORB_ENCODER_PORT       GPIOA
#define MOTORB_ENCODER_TIM        TIM3
#define ICx_FILTER                6

#define EncoderPeriod  20000


extern u8  TIM3CH1_CAPTURE_STA;		   				
extern u16	TIM3CH1_CAPTURE_VAL;	
extern u8  TIM3CH2_CAPTURE_STA;		    				
extern u16	TIM3CH2_CAPTURE_VAL;	
extern u8  TIM3CH3_CAPTURE_STA;				    				
extern u16	TIM3CH3_CAPTURE_VAL;	
extern u8  TIM3CH4_CAPTURE_STA;			    				
extern u16	TIM3CH4_CAPTURE_VAL;	

extern u32 LeftEncoder_Cnt,RightEncoder_Cnt;//左右轮计数全局化
extern u32 Time6_Cnt;

void TIM2_Int_Init(u16 arr,u16 psc);
void TIM8_PWM_Init(u16 arr,u16 psc);
void TIM3_Cap_Init(u16 arr,u16 psc);
void Time_Config(void);
#endif
