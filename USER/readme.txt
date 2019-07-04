全向轮小底盘程序使用说明：

	1.程序版本 V1.0.0
	
	2.硬件参数定义：
		硬件参数定义在"control.h"中，参数如下：
		轮子个数 	3
		轮子直径 	59mm
		轮间距   	157mm
		轮间角   	120度
		电机减速比  56
		编码器线速  11
		
	3.程序结构：
		main.c			整个系统的主函数，各个功能的初始化和控制都在这
		timer.c 		整个系统的定时器初始和PWM输出功能和编码器数据的采集功能的配置
		control.c		用于小车轮子运动控制，和PID算法的实现等
		dealdata.c		用于与上位通信协议的实现和遥控的实现，与UWB定位数据的处理等
		Encoder.c		用于小车编码器数据的读取
		myiic.c			模拟IIC的驱动程序
		gy85.c			GY85模块的驱动程序
		adc.c			ADC小车电量的读取驱动程序
		errordetect.c	整机故障检测程序
		kalman.c		用于gy85的原始数据转换，和角速度积分算出yaw等
		controlturn.c	小车行为控制的实现，旋转特定角度，走直线等
		bsp_usart.c		串口2和串口3的DMA的初始化等
		fsc_stos.c		os系统的实现，一般不要改动
		fsc_stos.h		在其中可改变os的一些参数