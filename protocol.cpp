#include <MemoryFree.h>
#include <SSD1306.h>
#include "protocol.h"

#include "GoKit.h"
extern SSD1306 oled;

uint8_t                   check_status_time;
uint8_t                   report_status_idle_time;

unsigned char CheckSum( unsigned char *buf, int packLen )
{
  int       i;
  unsigned char  sum;
  
  if(buf == NULL || packLen <= 0) return 0;

  sum = 0;
  for(i=2; i<packLen-1; i++) sum += buf[i];

  return sum;
}
void Handle_uartdata(unsigned char *buf,int len)
{
    if( len < 4 ) return ;

    pro_headPart  tmp_headPart;   
    memset(&tmp_headPart, 0, sizeof(pro_headPart));
    memcpy(&tmp_headPart, buf, sizeof(pro_headPart));
    if(CheckSum(buf,len)!=buf[len-1])
    {
        SendErrorCmd(ERROR_CHECKSUM, tmp_headPart.sn);
        #if(DEBUG==1)
        Serial.println("CheckSum error!");
        #endif
        return ;
    }    
    #if(DEBUG==1)
    Serial.print("sn = ");
    Serial.println(tmp_headPart.sn);
    #endif
    switch( tmp_headPart.cmd )
    {
      case CMD_GET_MCU_INFO :
          #if(DEBUG==1)
          Serial.println("CMD_GET_MCU_INFO");
          #endif
          CmdGetMcuInfo(tmp_headPart.sn);
      
      break;
      case CMD_SEND_MCU_P0 :
            #if(DEBUG==1)
            Serial.println("CMD_SEND_MCU_P0");
            #endif
      break;
      case CMD_SEND_HEARTBEAT:
            #if(DEBUG==1)
            Serial.println("CMD_SEND_HEARTBEAT");
            #endif
            SendCommonCmd(CMD_SEND_HEARTBEAT_ACK,tmp_headPart.sn);
      break;
      case CMD_REPORT_MODULE_STATUS:
            #if(DEBUG==1)
            Serial.println("CMD_REPORT_MODULE_STATUS");
            #endif
      break;
      default:
            #if(DEBUG==1)
            Serial.println("default");
            #endif
            SendErrorCmd(ERROR_CMD, tmp_headPart.sn);
      break;
    }
}
void Handle_keyeven()
{
  /*  长按是指按住按键3s以上   */
  switch(gokit_keydown())
  {
    case KEY1_SHORT_PRESS:
       #if (DEBUG==1) 
        Serial.println("KEY1_SHORT_PRESS");
       #endif

    break;
    case KEY1_LONG_PRESS:
      #if (DEBUG==1) 
        Serial.println("KEY1_LONG_PRESS");
       #endif
        gokit_ResetWiFi();
    break;
    case KEY2_SHORT_PRESS:
       #if (DEBUG==1) 
        Serial.println("KEY2_SHORT_PRESS");
       #endif
          gokit_sendAirlink();
    break;
    case KEY2_LONG_PRESS:
       #if (DEBUG==1) 
        Serial.println("KEY2_LONG_PRESS");
       #endif
       gokit_sendApCmd();
    break;
    default: break;
  }
  //KEY1 : D6
  //KEY2 : D7
}
void Check_Status()
{
  int         i, diff;
  uint8_t     *index_new, *index_old;
  
  diff = 0;
  gokit_DHT11_Read_Data(&m_m2w_mcuStatus.status_r.temputure, &m_m2w_mcuStatus.status_r.humidity);
  
  if(check_status_time < 200) return ;
    
  check_status_time = 0;
  index_new = (uint8_t *)&(m_m2w_mcuStatus.status_w);
  index_old = (uint8_t *)&(m_m2w_mcuStatus_reported.status_w);
    
  for(i=0; i<sizeof(status_writable); i++)
  {
    if(*(index_new+i) != *(index_old+i)) diff += 1;
  }
    
  if(diff == 0)
  {
    index_new = (uint8_t *)&(m_m2w_mcuStatus.status_r);
    index_old = (uint8_t *)&(m_m2w_mcuStatus_reported.status_r);
      
    for(i=0; i<sizeof(status_readonly); i++)
    {
      if(*(index_new+i) != *(index_old+i)) diff += 1;
    }
  }
  
  //Èç¹û×´Ì¬10·ÖÖÓÃ»ÓÐ±ä»¯£¬Ç¿ÖÆÉÏ±¨Ò»´Î£»
  if(diff > 0 || report_status_idle_time > 60000) ReportStatus(REPORT_STATUS);
  if(report_status_idle_time > 60000) report_status_idle_time = 0; 
  /*
  unsigned char tem,hum;
  
  //Temp&Humidity :D3
  //Motor :D4-D3
  //RGB-IIC
  //Arduino_SDA
  //Arduino_SCL
  if((gokit_time_s()-last_time)>2)
  {
    last_time = gokit_time_s();
    gokit_DHT11_Read_Data(&tem,&hum);
    #if (DEBUG==1)
    Serial.print("temp:");Serial.println(tem,DEC);
    Serial.print("hum:");Serial.println(hum,DEC);
    Serial.print("freeMemory()= ");
    Serial.println(freeMemory());
    #endif
  }
  */
}
void GoKit_Handle()
{
  Handle_uartdata(  uart_buf,get_onepackage(uart_buf));
  Handle_keyeven();
  Check_Status();
}

