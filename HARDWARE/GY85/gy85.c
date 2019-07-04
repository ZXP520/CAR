/**
  ******************************************************************************
  * @file    gy85.c
  * @author  zxp
  * @version V1.0.0
  * @date    2019-06-19
  * @brief   GY85ģ�����������
  ******************************************************************************
  * @attention �Ĵ�����ַ��Ӧ�ĸߵ�λ����
  ******************************************************************************
  */
#include "gy85.h"
#include "myiic.h"

/*******************************************************************************
* Function Name  : InitGY85
* Description    : GY85ģ���ʼ��
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void InitGY85(void) //��ʼ��Mpu6050
{
	I2C_GPIO_Config();

	//�����Ƴ�ʼ��
	IIC_Write_One_Byte(HMC5883L_Addr,0x0B,0x01); 
	IIC_Write_One_Byte(HMC5883L_Addr,0x20,0x40);
	IIC_Write_One_Byte(HMC5883L_Addr,0x21,0x01);
	IIC_Write_One_Byte(HMC5883L_Addr,0x09,0x0d); //


	//���ٶȼƳ�ʼ��
	IIC_Write_One_Byte(ADXL345_Addr,0x31,0x0B);   //������Χ,����16g��13λģʽ
	IIC_Write_One_Byte(ADXL345_Addr,0x2C,0x1A);   //����ģʽ100HZ���
	IIC_Write_One_Byte(ADXL345_Addr,0x2D,0x08);   //ѡ���Դģʽ   �ο�pdf24ҳ
	IIC_Write_One_Byte(ADXL345_Addr,0x2E,0x80);   //ʹ�� DATA_READY �ж�

	//�����ǳ�ʼ��
	IIC_Write_One_Byte(ITG3205_Addr,PWR_M, 0x80);   //��λ
	IIC_Write_One_Byte(ITG3205_Addr,SMPL,  0x07);    //
	IIC_Write_One_Byte(ITG3205_Addr,DLPF,  0x1E);    //��2000�� ��ͨ�˲�5hz ������1khz
	IIC_Write_One_Byte(ITG3205_Addr,INT_C, 0x00 );  //�ж�
	IIC_Write_One_Byte(ITG3205_Addr,PWR_M, 0x00);   //Ĭ��
}

/*******************************************************************************
* Function Name  : GetData
* Description    : �������
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address) //���16λ����
{
	char H,L;
	H=IIC_Read_One_Byte(SlaveAddr,REG_Address);
	L=IIC_Read_One_Byte(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}

/*******************************************************************************
* Function Name  : GetData
* Description    : ���QMC5883����
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address) //���16λ����
{
	char H,L;
	L=IIC_Read_One_Byte(SlaveAddr,REG_Address);
	H=IIC_Read_One_Byte(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}




