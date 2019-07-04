#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	DATAHEAD    0xDEED //����ͷ
#define RXDATALENTH 0x0A //�������ݳ���
#define TXDATALENTH 0x0008 //�������ݳ���
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//�ֱ�����
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15


#define PSB_SQUARE      26

typedef struct
{
	u8  UP;
	u8	RIGHT;
	u8  DOWN;
	u8  LEFT;
	u8  L2;
	u8  R2;
	u8  L1;
	u8	R1;
	u8	GREEN;
	u8 	RED;
	u8	BLUE;
	u8  PINK;
	
	s16 PSS_LY;
	s16 PSS_LX;
	s16 PSS_RY;
	s16 PSS_RX;
	
}PSBKEY;
extern PSBKEY PSBKey;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//���ݽ��սṹ��
typedef struct
{
	_Bool Success_Flag;	//���ճɹ���־
	_Bool Hardware_Init;	//Ӳ����ʼ�����ճɹ���־
	u8    FrameLength;		//֡����
	u16   CMD;					//��������
	u8    DataNum;				//�������
	s16   CheckSum;			//У����
	u8    Respond_Flag; //��Ӧ��־ 1����Ӧ 0������Ӧ
	
}DEALDATA_RX;
extern DEALDATA_RX DealData_Rx;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef enum 
{
	WhellDiameter=0,	//����ֱ��
	WheelBase,				//�������
	WhellRollSpeed,   //����ת��
	WhellSpeed,				//�����ٶ�
	ReductionRatio,   //���Ӽ��ٱ�
	WhellAcceleration,//���Ӽ��ٶ�
	EncoderLine, 			//����������
	EncoderValue,     //������ֵ
	IMUData,					//����������
	UltrasonicData,   //����������
	EmergencyStop,    //��ͣ״̬
	VersionNumber,    //�汾��
	RemainingBattery, //ʣ�����
	HardwareParameter //Ӳ������
	
}InquireCMD;//��ѯ����
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef enum 
{
	SWhellRollSpeed=0x8000,   //����ת��
	SWhellSpeed,							//�����ٶ�
	STurningRadius,     			//���ӹ���뾶
	SWhellAcceleration,       //���Ӽ��ٶ�
	SChassisAttitude,  				//������̬
	SPSRawData,								//PSԭʼ����
	UploadData=0xF000					//�ϴ���������
	
}SetCMD;//��������
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern union TEMPDATA{
		u16  UInTempData[15];
    s16  InTempData[15];
    u8 	 ChTempData[30];
}TempTxData,TempRxData;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef struct
{
	u32 A0;
	u32 A1;
	u32 A2;
	u32 A3;
	u32 x1;
	u32 y1;
	u32 x2;
	u32 y2;
	u32 x3;
	u32 y3;
	double X;
	double Y;
	u8  StableFlag;//�ȶ���־
	double AverageX;
	double AverageY;
}UWBDATA;
extern UWBDATA UWBData;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DealRXData(void);
void SendEncoderAndIMU20Ms(u8 sendflag);
void GetIMU0ffset(void);
void DealPSData(void);
void DealUWBData(void);
void calcPhonePosition   (double x1, double y1, double d1,
													double x2, double y2, double d2,
													double x3, double y3, double d3);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
