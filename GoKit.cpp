#include <Arduino.h>
#include <SoftwareSerial.h>
#include "GoKit.h"


m2w_returnMcuInfo         m_m2w_returnMcuInfo;
pro_commonCmd             m_pro_commonCmd;              //通用命令 心跳 ack等待复用
m2w_setModule             m_m2w_setModule;              //配置模块
w2m_controlMcu            m_w2m_controlMcu;             //控制MCU
m2w_mcuStatus             m_m2w_mcuStatus;              //MCU当前状态
m2w_mcuStatus             m_m2w_mcuStatus_reported;     //上次MCU的状态
w2m_reportModuleStatus    m_w2m_reportModuleStatus;     //WIFI模组状态
pro_errorCmd              m_pro_errorCmd;               //错误命令帧

void GoKit_Init()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  attachInterrupt(0, WiFi_Config, RISING);//当int.0上升沿触发,触发中断函数sendrandNunberCom pin 2
  attachInterrupt(1, WiFi_Reset, RISING);
  McuStatusInit();
}
/*******************************************************************************
* Function Name  : exchangeBytes
* Description    : like htonl
* Input          : value
* Output         : None
* Return         : 转换之后的数据
* Attention      : None
*******************************************************************************/
unsigned short exchangeBytes(unsigned short value)
{
  short     tmp_value;
  unsigned char   *index_1, *index_2;
  
  index_1 = (unsigned char *)&tmp_value;
  index_2 = (unsigned char *)&value;
  
  *index_1 = *(index_2+1);
  *(index_1+1) = *index_2;
  
  return tmp_value;
}
/*******************************************************************************
* Function Name  : SendToUart
* Description    : send data to uart 
* *buf           : the pointer of data to uart 
* packLen        : the data length 
* Return         : None
*******************************************************************************/
void SendToUart(unsigned char  *buf, unsigned short packLen, unsigned char  tag)
{
  unsigned short i=0;
  int             Send_num;
  pro_headPart    send_headPart;  
  pro_commonCmd   recv_commonCmd;  
  unsigned char m_55 = 0x55;
  unsigned long last_time=0;
  for(i=0;i<packLen;i++)
  {
    Serial.write(buf[i]);
    if(i >=2 && buf[i] == 0xFF) Serial.write(m_55);    // add 0x55 while across 0xff except head. 
  }
  if( tag == 0 ) return ;

  memcpy(&send_headPart, buf, sizeof(pro_headPart));
  memset(&recv_commonCmd, 0, sizeof(pro_commonCmd));
  last_time  = gokit_time_ms();  
  Send_num = 1;
  mySerial.print("restart send  timems");
  mySerial.println(gokit_time_ms());  
  while(Send_num <MAX_SEND_NUM )
  {
    if( (gokit_time_ms()-last_time)<MAX_SEND_TIME )
    {
        if(get_onepackage( uart_buf )>0)
        {
          memcpy(&recv_commonCmd, uart_buf, sizeof(pro_commonCmd));
          if((send_headPart.cmd == CMD_SEND_MODULE_P0 && recv_commonCmd.head_part.cmd == CMD_SEND_MODULE_P0_ACK) &&
          (send_headPart.sn == recv_commonCmd.head_part.sn)) break;
        
          if((send_headPart.cmd == CMD_SET_MODULE_WORKMODE && recv_commonCmd.head_part.cmd == CMD_SET_MODULE_WORKMODE_ACK) &&
          (send_headPart.sn == recv_commonCmd.head_part.sn)) break;
        
          if((send_headPart.cmd == CMD_RESET_MODULE && recv_commonCmd.head_part.cmd == CMD_RESET_MODULE_ACK) &&
          (send_headPart.sn == recv_commonCmd.head_part.sn)) break; 
        }
    }
    else
    {
        last_time  = gokit_time_ms();
        Send_num++;
        for(i=0;i<packLen;i++)
        {
          Serial.write(buf[i]);
          if(i >=2 && buf[i] == 0xFF) Serial.write(m_55);    // add 0x55 while across 0xff except head. 
        }
        mySerial.print("restart send  timems");
        mySerial.println(gokit_time_ms());
    } 
  }
}
/*******************************************************************************
* Function Name  : SendCommonCmd
* Description    : send common cmd to WiFi
* cmd            : CMD_SEND_MCU_P0_ACK  CMD_REPORT_MODULE_STATUS_ACK CMD_SEND_HEARTBEAT_ACK
*                   or CMD_REBOOT_MCU_ACK
* packLen        : sn 
* Return         : None
*******************************************************************************/
void  SendCommonCmd(uint8_t cmd, uint8_t sn)
{
  memset(&m_pro_commonCmd, 0, sizeof(pro_commonCmd));
  
  m_pro_commonCmd.head_part.head[0] = 0xFF;
  m_pro_commonCmd.head_part.head[1] = 0xFF;
  m_pro_commonCmd.head_part.len = exchangeBytes(5);
  m_pro_commonCmd.head_part.cmd = cmd;
  m_pro_commonCmd.head_part.sn = sn;
  m_pro_commonCmd.sum = CheckSum((uint8_t *)&m_pro_commonCmd, sizeof(pro_commonCmd));
  
  SendToUart((uint8_t *)&m_pro_commonCmd, sizeof(pro_commonCmd), 0);    
}

void  SendErrorCmd(uint8_t error_no, uint8_t sn)
{
  m_pro_errorCmd.head_part.sn = sn;
  m_pro_errorCmd.error = error_no;
  m_pro_errorCmd.sum = CheckSum((uint8_t *)&m_pro_errorCmd, sizeof(pro_errorCmd));
  
  SendToUart((uint8_t *)&m_pro_errorCmd, sizeof(pro_errorCmd), 0);    
}

int McuStatusInit()
{
  //common package init
  memset(&m_pro_commonCmd, 0, sizeof(pro_commonCmd));
  m_pro_commonCmd.head_part.head[0] = 0xFF;
  m_pro_commonCmd.head_part.head[1] = 0xFF;
  m_pro_commonCmd.head_part.len = exchangeBytes(sizeof(pro_commonCmd) - 4);

  // mcu info package init
  memset(&m_m2w_returnMcuInfo, 0, sizeof(m2w_returnMcuInfo));
  m_m2w_returnMcuInfo.head_part.head[0] = 0xFF;
  m_m2w_returnMcuInfo.head_part.head[1] = 0xFF;
  m_m2w_returnMcuInfo.head_part.len = exchangeBytes(sizeof(m2w_returnMcuInfo) - 4);     //except FF FF XX XX(XX XX is the len)
  m_m2w_returnMcuInfo.head_part.cmd = CMD_GET_MCU_INFO_ACK;
  memcpy(m_m2w_returnMcuInfo.pro_ver, PRO_VER, 8);
  memcpy(m_m2w_returnMcuInfo.p0_ver, P0_VER, 8);
  memcpy(m_m2w_returnMcuInfo.hard_ver, HARD_VER, 8);
  memcpy(m_m2w_returnMcuInfo.soft_ver, SOFT_VER, 8);
  memcpy(m_m2w_returnMcuInfo.product_key, PRODUCT_KEY, 32);
  m_m2w_returnMcuInfo.binable_time = exchangeBytes(PASSCODE_TIME);  //can be binded any time
    
  //
  memset(&m_m2w_mcuStatus, 0, sizeof(m2w_mcuStatus));
  m_m2w_mcuStatus.head_part.head[0] = 0xFF;
  m_m2w_mcuStatus.head_part.head[1] = 0xFF;
  m_m2w_mcuStatus.head_part.len = exchangeBytes(sizeof(m2w_mcuStatus) - 4);
  //DHT11_Read_Data((uint8_t *)&(m_m2w_mcuStatus.status_r.temputure), (uint8_t *)&(m_m2w_mcuStatus.status_r.humidity));
  m_m2w_mcuStatus.status_w.motor_speed = 5;
  
  //
  memset(&m_m2w_setModule, 0, sizeof(m2w_setModule));
  m_m2w_setModule.head_part.head[0] = 0xFF;
  m_m2w_setModule.head_part.head[1] = 0xFF;
  m_m2w_setModule.head_part.cmd = CMD_SET_MODULE_WORKMODE;
  m_m2w_setModule.head_part.len = exchangeBytes(sizeof(m2w_setModule) - 4);

  //
  memset(&m_pro_errorCmd, 0, sizeof(pro_errorCmd));
  m_pro_errorCmd.head_part.head[0] = 0xFF;
  m_pro_errorCmd.head_part.head[1] = 0xFF;
  m_pro_errorCmd.head_part.cmd = CMD_MODULE_CMD_ERROR_ACK;
  m_pro_errorCmd.head_part.len = exchangeBytes(sizeof(pro_errorCmd) - 4);

}
void CmdGetMcuInfo(uint8_t sn)
{
  m_m2w_returnMcuInfo.head_part.sn = sn;
  m_m2w_returnMcuInfo.sum = CheckSum((uint8_t *)&m_m2w_returnMcuInfo, sizeof(m2w_returnMcuInfo));
  SendToUart((uint8_t *)&m_m2w_returnMcuInfo, sizeof(m2w_returnMcuInfo), 0);  
}
/******************************************************
 *    function : get one gagetn uart package  
 *    buf      : uart receive buf. 
 *    return   : uart data length.
 *    Add by Alex.lin    --2014-12-24
******************************************************/
int get_onepackage(unsigned char *buf)
{
      unsigned char UartC;
      unsigned char tempbuf[2];
      int ind=0,i=0;
      int remainlen=0;
      int packlen=0;      
     while(Serial.available()>0)
    {           
      UartC = (unsigned char)Serial.read();
      
      if(ind<2)
      {
        mySerial.println("i<2");
        if( UartC!=0XFF ) continue;
      }
      if(ind==2) tempbuf[0] = UartC;
      if(ind==3) tempbuf[1] = UartC;
      if(ind>3) remainlen = (tempbuf[0]*256)+tempbuf[1];
      
      /* too long */
      if(remainlen>1000) return 0;
      /**** remove 0X55 ****/
      
      if(UartC==0X55 && buf[ind-1]==0XFF)
       {
         mySerial.println("UartC==0X55 remove it");
         continue;
       }
       buf[ind++] = UartC;
      packlen++;
      
    }
    if(remainlen>0)
    {
      mySerial.println("receive data is:");
      for ( i=0;i<ind; i++ )
      {
        mySerial.print(buf[i],HEX);
        mySerial.print(" ");
      }
      mySerial.println("");
    }
    /*error uart data*/
    if((packlen-4)!=remainlen) return 0;
    return packlen;

}
/*******************************************************
 *    function      : WiFi_Reset
 *    Description   : gokit reset the wifi. 
 *    return        : None.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void WiFi_Reset()
{

}
/*******************************************************
 *    function      : WiFi_Config
 *    Description   : Gonfig wifi into airlink
 *    return        : None.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void WiFi_Config()
{

}
/*******************************************************
 *    function      : gokit_time_ms
 *    Description   : gokit running time form power on 
 *    return        : gokit running time(ms).
 *    Add by Alex.lin    --2014-12-25
******************************************************/
unsigned long gokit_time_ms()
{
  return millis();
}
/*******************************************************
 *    function      : gokit_time_m
 *    Description   : gokit running time form power on 
 *    return        : gokit running time(m).
 *    Add by Alex.lin    --2014-12-25
******************************************************/
unsigned long gokit_time_m()
{
  return millis()/1000;
}
