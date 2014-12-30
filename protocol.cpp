#include <MemoryFree.h>
#include <SSD1306.h>
#include "protocol.h"

#include "GoKit.h"
extern SSD1306 oled;

unsigned long                   check_status_time=0;
unsigned long                   report_status_idle_time=0;

unsigned char CheckSum( unsigned char *buf, int packLen )
{
  int       i;
  unsigned char  sum;
  
  if(buf == NULL || packLen <= 0) return 0;

  sum = 0;
  for(i=2; i<packLen-1; i++) sum += buf[i];

  return sum;
}
void  CmdSendMcuP0(uint8_t *buf)
{
  uint8_t   tmp_cmd_buf;
  
  if(buf == NULL) return ;
  
  memcpy(&m_w2m_controlMcu, buf, sizeof(w2m_controlMcu));
  m_w2m_controlMcu.status_w.motor_speed = exchangeBytes(m_w2m_controlMcu.status_w.motor_speed);
  
  //
  if(m_w2m_controlMcu.sub_cmd == SUB_CMD_REQUIRE_STATUS) gokit_ReportStatus(REQUEST_STATUS);
  
  //
  if(m_w2m_controlMcu.sub_cmd == SUB_CMD_CONTROL_MCU){
    //
    SendCommonCmd(CMD_SEND_MCU_P0_ACK, m_w2m_controlMcu.head_part.sn);
    
    //want to control LED R 
    if((m_w2m_controlMcu.cmd_tag & 0x01) == 0x01)
    {
      //0 bit, 1: R on, 0: R off;
      if((m_w2m_controlMcu.status_w.cmd_byte & 0x01) == 0x01)
      {
        gokit_setColorRGB(254, 0, 0);
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte | 0x01);
      }
      else
      {
        gokit_setColorRGB(0, 0, 0);
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte & 0xFE);
      }
    }

    //¿ØÖÆLED×éºÏÑÕÉ«
    if((m_w2m_controlMcu.cmd_tag & 0x02) == 0x02)
    {
      tmp_cmd_buf = (m_w2m_controlMcu.status_w.cmd_byte & 0x06) >> 1;
      // Ê¹ÓÃcmd_byteµÄµÚ2ºÍ3Î»£¬00:user define, 01: yellow, 10: purple, 11: pink
      
      if(tmp_cmd_buf == 0x00)
      {
        gokit_setColorRGB(m_w2m_controlMcu.status_w.led_r, m_m2w_mcuStatus.status_w.led_g, m_m2w_mcuStatus.status_w.led_b); 
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte & 0xF9);
      }
      else if(tmp_cmd_buf == 0x01)
      {   
        gokit_setColorRGB(254, 70, 0);
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte | 0x02);
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte & 0xFB);
      }
      else if(tmp_cmd_buf == 0x02)
      {
        gokit_setColorRGB(254, 0, 70);  
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte | 0x04);
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte & 0xFD);
      }
      else if(tmp_cmd_buf == 0x03)
      {
        gokit_setColorRGB(238, 30, 30);
        m_m2w_mcuStatus.status_w.cmd_byte = (m_m2w_mcuStatus.status_w.cmd_byte | 0x06);
      }
    }
    
    tmp_cmd_buf = (m_m2w_mcuStatus.status_w.cmd_byte & 0x06) >> 1;

    if((m_w2m_controlMcu.cmd_tag & 0x04) == 0x04)
    {

      if(tmp_cmd_buf == 0x00){
        gokit_setColorRGB(m_w2m_controlMcu.status_w.led_r, m_m2w_mcuStatus.status_w.led_g, m_m2w_mcuStatus.status_w.led_b);     
        m_m2w_mcuStatus.status_w.led_r = m_w2m_controlMcu.status_w.led_r;
      }
    }
    
    if((m_w2m_controlMcu.cmd_tag & 0x08) == 0x08)
    {
      //µ±LED×éºÏÑÕÉ«ÎªÓÃ»§×Ô¶¨ÒåÊ±ÉúÐ§
      if(tmp_cmd_buf == 0x00){
        gokit_setColorRGB(m_m2w_mcuStatus.status_w.led_r, m_w2m_controlMcu.status_w.led_g, m_m2w_mcuStatus.status_w.led_b);     
        m_m2w_mcuStatus.status_w.led_g = m_w2m_controlMcu.status_w.led_g;
      }
    }

    //¿ØÖÆ LED B
    if((m_w2m_controlMcu.cmd_tag & 0x10) == 0x10)
    {
      //µ±LED×éºÏÑÕÉ«ÎªÓÃ»§×Ô¶¨ÒåÊ±ÉúÐ§
      if(tmp_cmd_buf == 0x00){
        gokit_setColorRGB(m_m2w_mcuStatus.status_w.led_r, m_m2w_mcuStatus.status_w.led_g, m_w2m_controlMcu.status_w.led_b);     
        m_m2w_mcuStatus.status_w.led_b = m_w2m_controlMcu.status_w.led_b;
      }
    }
    
    //¿ØÖÆµç»ú
    if((m_w2m_controlMcu.cmd_tag & 0x20) == 0x20)
    {
      gokit_motorstatus(m_w2m_controlMcu.status_w.motor_speed);
      m_m2w_mcuStatus.status_w.motor_speed = m_w2m_controlMcu.status_w.motor_speed;
    }
    
    gokit_ReportStatus(REPORT_STATUS);
  }
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
        mySerial.println("CheckSum error!");
        #endif
        return ;
    }    
    #if(DEBUG==1)
    Serial.print("sn = ");
    Serial.println(tmp_headPart.sn);
   
    mySerial.println("receive:");
    for (int i = 0; i < len; ++i)
    {
      mySerial.print(" "); mySerial.print(buf[i],HEX);
    }
    mySerial.println("");
    #endif
    switch( tmp_headPart.cmd )
    {
      
      case CMD_GET_MCU_INFO :
          #if(DEBUG==1)
          Serial.println("CMD_GET_MCU_INFO");
          mySerial.println("CMD_GET_MCU_INFO");
          #endif
          CmdGetMcuInfo(tmp_headPart.sn);
      
      break;
      
      case CMD_SEND_MCU_P0 :
            #if(DEBUG==1)
            Serial.println("CMD_SEND_MCU_P0");
            mySerial.println("CMD_SEND_MCU_P0");
            #endif
            CmdSendMcuP0(buf);
      break;
      case CMD_SEND_HEARTBEAT:
            #if(DEBUG==1)
            Serial.println("CMD_SEND_HEARTBEAT");
            mySerial.println("CMD_SEND_HEARTBEAT");
            #endif
            SendCommonCmd(CMD_SEND_HEARTBEAT_ACK,tmp_headPart.sn);
      break;
      /*
      case CMD_REPORT_MODULE_STATUS:
            #if(DEBUG==1)
            Serial.println("CMD_REPORT_MODULE_STATUS");
            #endif
      break;
      */
      default:
            #if(DEBUG==1)
            //Serial.println("default");
            #endif
            //SendErrorCmd(ERROR_CMD, tmp_headPart.sn);
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
  
  if(gokit_time_s()-check_status_time < 10 ) return ;
    
  check_status_time = gokit_time_s();
  index_new = (uint8_t *)&(m_m2w_mcuStatus.status_w);
  index_old = (uint8_t *)&(m_m2w_mcuStatus_reported.status_w);
    
  for(i=0; i<sizeof(status_writable); i++)
  {
    if(*(index_new+i) != *(index_old+i)) 
      {
         diff += 1;
         #if(DEBUG==1)
         Serial.print("status_w ");Serial.println(i,DEC);
         #endif
      }

  }
    
  if(diff == 0)
  {
    index_new = (uint8_t *)&(m_m2w_mcuStatus.status_r);
    index_old = (uint8_t *)&(m_m2w_mcuStatus_reported.status_r);
      
    for(i=0; i<sizeof(status_readonly); i++)
    {
      if(*(index_new+i) != *(index_old+i))
      {
        diff += 1;
       #if(DEBUG==1) 
        Serial.print("status_r ");Serial.println(i,DEC);
        Serial.print("old: ");Serial.println(index_old[i]);
        Serial.print("new: ");Serial.println(index_new[i]);
        Serial.print("temp:");Serial.println(m_m2w_mcuStatus.status_r.temputure,DEC);
        Serial.print("hum: ");Serial.println(m_m2w_mcuStatus.status_r.humidity,DEC);
      #endif
      }
    }
  }
  if(diff > 0 || gokit_time_s()-report_status_idle_time > GOKIT_REPORT_TIME)
  {
   gokit_ReportStatus(REPORT_STATUS);
   report_status_idle_time = gokit_time_s(); 
  }

}
void GoKit_Handle()
{
  Handle_uartdata(  uart_buf,get_onepackage(uart_buf));
  Handle_keyeven();
  Check_Status();
}

