//
extern float Angle_X_Final;
extern float Angle_Y_Final;
extern float Angle_Z_Final;

extern float Gx,Gy,Gz;//��λ ��/s
extern float Angle_accX,Angle_accY,Angle_accZ;//�洢���ٶȼ�����ĽǶ�
extern float angle2;
extern float Yaw,Pitch,Roll;  //???,???,???

void Angle_Calcu(void);
void Angle_Calcu1(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void Kalman_Filter_Z(float Accel,float Gyro);

