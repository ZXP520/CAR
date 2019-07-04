#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
--------------------------------------定时器引脚图---------------------------------------------------
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/ 

/*++++++++++++++++++++++++++++++++++TIME1_ENCODER+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIM1_ENCODER_TIM_CLK  	RCC_APB2Periph_TIM1
#define TIM1_ENCODER_PIN_CLK	RCC_APB2Periph_GPIOA
#define TIM1_ENCODER_PIN        GPIO_Pin_8|GPIO_Pin_9
#define TIM1_ENCODER_PORT       GPIOA

/*++++++++++++++++++++++++++++++++++TIME2_ENCODER+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIM2_ENCODER_TIM_CLK  	RCC_APB1Periph_TIM2
#define TIM2_ENCODER_PIN_CLK	RCC_APB2Periph_GPIOA 
#define TIM2_ENCODER_PIN_CLK1	RCC_APB2Periph_GPIOB
#define TIM2_ENCODER_PIN        GPIO_Pin_15
#define TIM2_ENCODER_PIN1  		GPIO_Pin_3
#define TIM2_ENCODER_PORT       GPIOA
#define TIM2_ENCODER_PORT1      GPIOB

/*++++++++++++++++++++++++++++++++++TIME3_ENCODER+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIM3_ENCODER_TIM_CLK  	RCC_APB1Periph_TIM3
#define TIM3_ENCODER_PIN_CLK	RCC_APB2Periph_GPIOA
#define TIM3_ENCODER_PIN        GPIO_Pin_6|GPIO_Pin_7
#define TIM3_ENCODER_PORT       GPIOA

/*++++++++++++++++++++++++++++++++++TIME4_PWM+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIM4_PWM_TIM_CLK  		RCC_APB1Periph_TIM4
#define TIM4_PWM_PIN_CLK		RCC_APB2Periph_GPIOB
#define TIM4_PWM_PIN        	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9
#define TIM4_PWM_PORT       	GPIOB

/*++++++++++++++++++++++++++++++++++TIME5_ENCODER+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIM5_ENCODER_TIM_CLK  	RCC_APB1Periph_TIM5
#define TIM5_ENCODER_PIN_CLK	RCC_APB2Periph_GPIOA
#define TIM5_ENCODER_PIN        GPIO_Pin_0|GPIO_Pin_1
#define TIM5_ENCODER_PORT       GPIOA

/*++++++++++++++++++++++++++++++++++TIME8_PWM+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIM8_PWM_TIM_CLK  		RCC_APB2Periph_TIM8
#define TIM8_PWM_PIN_CLK		RCC_APB2Periph_GPIOC
#define TIM8_PWM_PIN        	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9
#define TIM8_PWM_PORT       	GPIOC

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define EncoderPeriod  20000
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern u32 LeftEncoder_Cnt,RightEncoder_Cnt;//左右轮计数全局化
extern u32 Time6_Cnt;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM8_PWM_Init(u16 arr,u16 psc);
void TIM3_Cap_Init(u16 arr,u16 psc);
void Time_Config(void);
#endif
