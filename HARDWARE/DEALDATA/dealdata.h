#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD  0x0000A5A5 //数据头
#define RXDATALENTH 0x00000008 //接收数据长度
#define TXDATALENTH 0x00000008 //发送数据长度
void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
