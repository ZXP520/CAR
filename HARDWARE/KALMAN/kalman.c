#include "kalman.h"
#include "gy85.h"
#include "math.h"
#include "fsc_stos.h"
#include "errordetect.h"
#include "timer.h"

float Accel_x;	     //X轴加速度值暂存
float Accel_y;	     //Y轴加速度值暂存
float Accel_z;	     //Z轴加速度值暂存

float Gyro_x;		 //X轴陀螺仪数据暂存
float Gyro_y;        //Y轴陀螺仪数据暂存
float Gyro_z;		 //Z轴陀螺仪数据暂存

float Bmp_x;
float Bmp_y;
float Bmp_z;

//float Angle_gy;    //由角速度计算的倾斜角度
float Angle_x_temp;  //由加速度计算的x倾斜角度
float Angle_y_temp;  //由加速度计算的y倾斜角度
float Angle_z_temp;  //由加速度计算的y倾斜角度

float Angle_X_Final; //X最终倾斜角度
float Angle_Y_Final; //Y最终倾斜角度
float Angle_Z_Final; //z最终倾斜角度


//角度计算
void Angle_Calcu(void)	 
{
	static int yaw=0;
	//范围为2g时，换算关系：16384 LSB/g
	//deg = rad*180/3.14
	float x,y,z;
	
	Accel_x = ImuData.NAccelData[0];//GetData(0xA6,ACCEL_XOUT_H); //x轴加速度值暂存
	Accel_y = ImuData.NAccelData[1];//GetData(0xA6,ACCEL_YOUT_H); //y轴加速度值暂存
	Accel_z = ImuData.NAccelData[2];//GetData(0xA6,ACCEL_ZOUT_H); //z轴加速度值暂存
	Gyro_x  = ImuData.NGYData[0];//GetData(SlaveAddress,GYRO_XOUT_H);  //x轴陀螺仪值暂存
	Gyro_y  = ImuData.NGYData[1];//GetData(SlaveAddress,GYRO_YOUT_H);  //y轴陀螺仪值暂存
	Gyro_z  = ImuData.NGYData[2];//GetData(SlaveAddress,GYRO_ZOUT_H);  //z轴陀螺仪值暂存
	
	
	//Bmp_x   = GetData(0x3C,0x03);
	//Bmp_y		= GetData(0x3C,0x05);
	//Bmp_z		= GetData(0x3C,0x07);
	
	//yaw += Gyro_z*0.02/32.768f;
	
	//u2_printf("%d %d \n",(int)Gyro_z,yaw);
	
	//处理x轴加速度
	if(Accel_x<32764) x=Accel_x/16384;
	else              x=1-(Accel_x-49152)/16384;
	
	//处理y轴加速度
	if(Accel_y<32764) y=Accel_y/16384;
	else              y=1-(Accel_y-49152)/16384;
	
	//处理z轴加速度
	if(Accel_z<32764) z=Accel_z/16384;
	else              z=(Accel_z-49152)/16384;

	//用加速度计算三个轴和水平面坐标系之间的夹角
	Angle_x_temp=(atan(y/z))*180/3.14;
	Angle_y_temp=(atan(x/z))*180/3.14;
	Angle_z_temp=(atan(y/x))*180/3.14;

	//角度的正负号											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
	if(Accel_z<32764) Angle_z_temp = +Angle_z_temp;
	if(Accel_z>32764) Angle_z_temp = -Angle_z_temp;
	
	//角速度
	//向前运动
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	//向前运动
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	//向前运动
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //角速度积分得到倾斜角度.越大积分出来的角度越大
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //卡尔曼滤波计算Y倾角
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //卡尔曼滤波计算Y倾角
	Kalman_Filter_Z(Angle_z_temp,Gyro_z);
															  
} 


//卡尔曼参数		
float Q_angle = 0.001;  
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dt为kalman滤波器采样时间;
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_X_Final += (Gyro - Q_bias) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final += K_0 * Angle_err;	 //后验估计
	Q_bias        += K_1 * Angle_err;	 //后验估计
	Gyro_x         = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Y(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_Y_Final += (Gyro - Q_bias) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Z(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_Y_Final += (Gyro - Q_bias) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Z_Final	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

//二阶互补滤波
float K2 =0.01; // 权值
float x1,x2,y1;
float edt=20*0.001;//采样时间
float angle2;
void Erjielvbo(float angle_m,float gyro_m)//输入角度值
{
    x1=(angle_m-angle2)*(1-K2)*(1-K2);
    y1=y1+x1*edt;
    x2=y1+2*(1-K2)*(angle_m-angle2)+gyro_m;
    angle2=angle2+ x2*edt;
}


//====一下三个定义了陀螺仪的偏差===========
#define Gx_offset -2.74725
#define Gy_offset -0.7326
#define Gz_offset 2.930403
#define pi 3.141592653

int16_t ax,ay,az;
int16_t gx,gy,gz;//存储原始数据
float aax,aay,aaz,ggx,ggy,ggz;//存储量化后的数据
float Ax,Ay,Az;//单位 g(9.8m/s^2)
float Gx,Gy,Gz;//单位 °/s
float Angle_accX,Angle_accY,Angle_accZ;//存储加速度计算出的角度

static void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
//角度计算
void Angle_Calcu1(void)
{
	 static u8 first_flag=1;
	 static u32 NewTime=0,LastTime=0;
	 ax=ImuData.NAccelData[0];
	 ay=ImuData.NAccelData[1];
	 az=ImuData.NAccelData[2];
	 
	 //取第一次的值取零飘
	 if(ImuData.ZeroFtlag==0)
	 {
		 ImuData.OffsetGYData[0]=ImuData.NGYData[0];
		 ImuData.OffsetGYData[1]=ImuData.NGYData[1];
		 ImuData.OffsetGYData[2]=ImuData.NGYData[2];
		 ImuData.ZeroFtlag=1;
	 }
	 //去零飘
	 gx=ImuData.NGYData[0]-ImuData.OffsetGYData[0];
	 gy=ImuData.NGYData[1]-ImuData.OffsetGYData[1];
	 gz=ImuData.NGYData[2]-ImuData.OffsetGYData[2];

	//======一下三行是对加速度进行量化，得出单位为g的加速度值-2g量程
   Ax=ax/16.384;
   Ay=ay/16.384;
   Az=az/16.384;
   //==========以下三行是用加速度计算三个轴和水平面坐标系之间的夹角
   Angle_accX= atan(Ax / Az)*180/ pi;     //加速度仪，反正切获得弧度值，乘以180度/pi 
   Angle_accY= atan(Ay / Az)*180/ pi;   //获得角度值，乘以-1得正
	 Angle_accZ= atan(Ax / Ay)*180/ pi;
   //==========以下三行是对角速度做量化-2000°量程灵敏度因子14.375==========
   ggx=gx/14.375;
   ggy=gy/14.375;
   ggz=gz/14.375;
	 if(abs(ggz)<0.1){ggz=0;}
  
	//算积分时间
	NewTime=GetOSSliceTime();
	//NewTime=Time6_Cnt;
  //下面三行就是通过对角速度积分实现各个轴的角度测量，当然假设各轴的起始角度都是0
  Gx=Gx+(ggx-0)*20/1000;
  Gy=Gy+(ggy-0)*20/1000;
  Gz=Gz+(ggz-0)*(NewTime-LastTime)*2/1000;
	if(Gz<0){Gz=359;}
	else if(Gz>360){Gz=0;}
	ImuData.Yaw=Gz;
	LastTime=NewTime;
}


//---------------------------------------------------------------------------------------------------
// ????

#define Kp 2.0f                        // ??????????????/???
#define Ki 0.001f                // ????????????????
#define halfT 0.001f                // ???????

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // ??????,??????
float exInt = 0, eyInt = 0, ezInt = 0;        // ?????????

float Yaw,Pitch,Roll;  //???,???,???


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
				gx=gx-Gx_offset;
				gy=gy-Gy_offset;
				gz=gz-Gz_offset;
				
				
        // ?????
        norm = sqrt(ax*ax + ay*ay + az*az);      
        ax = ax / norm;                   //???
        ay = ay / norm;
        az = az / norm;      

        // ???????
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // ???????????????????????????
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);

        // ??????????
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

        // ?????????
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        // ??????????
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

        // ?????
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;

        //Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,?????
        //Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                
}
