#ifndef	_GOKIT_H_
#define _GOKIT_H_
#include "protocol.h"

#define   MAX_SEND_NUM                          3   
#define   MAX_SEND_TIME                         200

extern  unsigned char uart_buf[1024];
extern SoftwareSerial mySerial;
extern m2w_returnMcuInfo m_m2w_returnMcuInfo;
void GoKit_Init();
int McuStatusInit();
void WiFi_Reset();
void WiFi_Config();
unsigned long gokit_time_ms();
unsigned long gokit_time_m();
void SendToUart(unsigned char  *buf, unsigned short packLen, unsigned char  tag);
unsigned short exchangeBytes(unsigned short value);
int get_onepackage(unsigned char *buf);
int send_onepackage( unsigned char *buf,int len );
void CmdGetMcuInfo(uint8_t sn);
void SendCommonCmd(uint8_t cmd, uint8_t sn);
void  SendErrorCmd(uint8_t error_no, uint8_t sn);
#endif
