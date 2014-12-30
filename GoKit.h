#ifndef	_GOKIT_H_
#define _GOKIT_H_
#include <DHT.h>
#include "protocol.h"

#define   DEBUG           0
/*************************** HAL define ***************************/
#define   DHTPIN          3
#define   DHTTYPE         DHT11 
#define   KEY1            6
#define   KEY2            7
#define   KEY1_SHORT_PRESS  1
#define   KEY1_LONG_PRESS   2
#define   KEY2_SHORT_PRESS  4
#define   KEY2_LONG_PRESS   8
#define   NO_KEY            0

#define   KEY_LONG_TIMER    3   //( 3s )

#define   MOTOR_PINA      4
#define   MOTOR_PINB      5
#define   MOTOR_MAX       100
#define   MOTOR_MAX1      -100
#define   MOTOR_MIN       0
/******************************************************************/

#define   MAX_SEND_NUM    3   
#define   MAX_SEND_TIME   200
#define   MAX_UART_LEN    100
#define   UART_RX_BUF_SIZE 100

extern  unsigned char uart_buf[MAX_UART_LEN];
extern SoftwareSerial mySerial;
extern DHT dht;
extern unsigned long last_time;
extern m2w_returnMcuInfo m_m2w_returnMcuInfo;
extern w2m_controlMcu            m_w2m_controlMcu;             //控制MCU
extern m2w_mcuStatus             m_m2w_mcuStatus;              //MCU当前状态
extern m2w_mcuStatus             m_m2w_mcuStatus_reported;     //上次MCU的状态
extern w2m_reportModuleStatus    m_w2m_reportModuleStatus;     //WIFI模组状态

void GoKit_Init();
int McuStatusInit();
void WiFi_Reset();
void WiFi_Config();
void gokit_motor_init();
void gokit_motorstatus( char motor_speed );
void gokit_timer();
unsigned long gokit_time_ms();
unsigned long gokit_time_s();
void gokit_DHT11_Read_Data( unsigned char *temperature,unsigned char *humidity);

char gokit_key1down();
char gokit_key2down();
char gokit_keydown();

void gokit_IR_event();
void gokit_setColorRGB(byte red, byte green, byte blue);

void gokit_ResetWiFi();
void gokit_sendAirlink();
void gokit_sendApCmd();
void gokit_ReportStatus(uint8_t tag);
void SendToUart(unsigned char  *buf, unsigned short packLen, unsigned char  tag);
unsigned short exchangeBytes(unsigned short value);
int get_onepackage(unsigned char *buf);
int send_onepackage( unsigned char *buf,int len );
void CmdGetMcuInfo(uint8_t sn);
void SendCommonCmd(uint8_t cmd, uint8_t sn);
void  SendErrorCmd(uint8_t error_no, uint8_t sn);
#endif
