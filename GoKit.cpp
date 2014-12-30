
#include <Arduino.h>
#include <DHT.h>
#include <MsTimer2.h>
#include <ChainableLED.h>
#include <SoftwareSerial.h>
#include "GoKit.h"

DHT dht(DHTPIN, DHTTYPE);
ChainableLED leds(A5, A4, 1);
#if(DEBUG==1)
SoftwareSerial mySerial(8, 9);
#endif
void motortime();
unsigned char uart_buf[MAX_UART_LEN]={0};
m2w_returnMcuInfo         m_m2w_returnMcuInfo;
pro_commonCmd             m_pro_commonCmd;              //通用命令 心跳 ack等待复用
m2w_setModule             m_m2w_setModule;              //配置模块
w2m_controlMcu            m_w2m_controlMcu;             //控制MCU
m2w_mcuStatus             m_m2w_mcuStatus;              //MCU当前状态
m2w_mcuStatus             m_m2w_mcuStatus_reported;     //上次MCU的状态
w2m_reportModuleStatus    m_w2m_reportModuleStatus;     //WIFI模组状态
pro_errorCmd              m_pro_errorCmd;               //错误命令帧
unsigned char SN = 0;
unsigned long last_time=0;
unsigned char hal_UartRxBuffer[UART_RX_BUF_SIZE];

int hal_UartRxLen;
int hal_dmatxhead, hal_dmatxtail;
int hal_UartRxTail;
int hal_UartRxHead;
void GoKit_Init()
{
  Serial.begin(9600);
  #if(DEBUG==1)
  mySerial.begin(9600);
  #endif
  dht.begin();
  leds.init();
  
  pinMode(KEY1,INPUT_PULLUP); //KEY1 上拉输入
  pinMode(KEY2,INPUT_PULLUP); //KEY2 上拉输入
  attachInterrupt(0, gokit_IR_event, CHANGE);//当引脚电平发生变化,触发中断函数gokit_IR_event pin 2
  MsTimer2::set(1000, gokit_timer); // 1s period
  MsTimer2::start();
  gokit_motor_init();
  gokit_setColorRGB(0,0,0);
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
  #if(DEBUG==1)
  Serial.print("restart send  timems");
  Serial.println(gokit_time_ms());  
  #endif
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
        #if(DEBUG==1)
        Serial.print("restart send  timems");
        Serial.println(gokit_time_ms());
        #endif
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
  gokit_DHT11_Read_Data((uint8_t *)&(m_m2w_mcuStatus.status_r.temputure), (uint8_t *)&(m_m2w_mcuStatus.status_r.humidity));
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
 *    function    : serialEvent
 *    Description : arduino serial uart receive  interrupt 
 *                  function
 *   
 *    return      : none.
 *    Add by Alex.lin    --2014-12-24
******************************************************/
void serialEvent()
{
   hal_UartRxBuffer[hal_UartRxTail] = (unsigned char)Serial.read();
      hal_UartRxTail++;
   if (hal_UartRxTail == UART_RX_BUF_SIZE)
   {
        hal_UartRxTail = 0;
  }
    hal_UartRxLen++;
#if(DEBUG==1)  
  Serial.println(hal_UartRxBuffer[hal_UartRxTail-1],HEX);
#endif
}
/******************************************************
 *    function : get one gagetn uart package  
 *    buf      : uart receive buf. 
 *    return   : uart data length.
 *    Add by Alex.lin    --2014-12-24
******************************************************/
int get_onepackage(unsigned char *buf)
{
  int ret = 0;
  int len = 0;
  int i, head=0;
  unsigned long receive_time=0;
  if (hal_UartRxLen<4) // not have enough data yet.
    goto done;

    for (i=0, head=hal_UartRxHead; i+1<(int)hal_UartRxLen; i++) 
    {

      if ((hal_UartRxBuffer[head] != 0xFF) || (hal_UartRxBuffer[(head+1)%UART_RX_BUF_SIZE] != 0xFF) )   
      {
        head++;
        if (head == UART_RX_BUF_SIZE) {
          head = 0;
        }
      } else {
          break;
      } 
    }
  hal_UartRxLen -= i; 
  hal_UartRxHead = head; // remove invalid data.

  if (hal_UartRxLen<4) // not have enough data yet.
    goto done;

  // copy first 4 bytes to buf
  head = hal_UartRxHead;
  for (i = 0; i < 4; i++) 
  {
    buf[i] = hal_UartRxBuffer[head++];
    if (head == UART_RX_BUF_SIZE) {
      head = 0;
    }
  }

    // total data length
  len = (buf[2]<<8 )+ buf[3] + 4;
  
  if (len > UART_RX_BUF_SIZE) { // length to long, must wrong format
    goto WRONG;
  }
    
  if (len > (int)hal_UartRxLen)//
    goto done; // not have enough data yet.

  receive_time = gokit_time_s();
    // copy data & checksum to buf
  for (; i<len; i++) 
  {
    if((gokit_time_s()-receive_time) >1)
    {
      goto WRONG;//1S内没接收到全数据，跳出。
    }
    buf[i] = hal_UartRxBuffer[head];
    head++;
    if (head == UART_RX_BUF_SIZE) {
      head = 0;
    }
  }
  
  ret = len;
  hal_UartRxLen -= len;
  hal_UartRxHead = head;
  goto done;
    
WRONG: // wrong format, remove one byte
  hal_UartRxLen--;
  hal_UartRxHead++;
  if (hal_UartRxHead == UART_RX_BUF_SIZE) {
    hal_UartRxHead = 0;
  }
done:
  return ret;
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
unsigned long gokit_time_s()
{
  return millis()/1000;
}

/*******************************************************
 *    function      : gokit_DHT11_Read_Data
 *    Description   : gokit read temperature and hum 
 *    return        : none
 *    Add by Alex.lin    --2014-12-25
******************************************************/

void gokit_DHT11_Read_Data( unsigned char *temperature,unsigned char *humidity)
{
  *temperature = (unsigned char)dht.readTemperature();
  *humidity = (unsigned char)dht.readHumidity();
  return ;
}
/*******************************************************
 *    function      : gokit_key1down
 *    Description   : check the gokit key1 event 
 *    return        : KEY1_LONG_PRESS  KEY1_SHORT_PRESS  
 *                     0-no keydown event.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
 char gokit_key1down()
{
  int unsigned long keep_time=0;
  if( digitalRead(KEY1)==LOW)
  {
    delay(100);
    if(digitalRead(KEY1)==LOW)
    {
      keep_time = gokit_time_s();
      while(digitalRead(KEY1)==LOW)
      {
        if( (gokit_time_s()-keep_time)> KEY_LONG_TIMER)
        {
          return KEY1_LONG_PRESS;
         }        
      }//until open the key 
      return KEY1_SHORT_PRESS ;
    }
    return 0;
  }
  return 0;
}
/*******************************************************
 *    function      : gokit_key2down
 *    Description   : check the gokit key2 event 
 *    return        : KEY2_LONG_PRESS  KEY2_SHORT_PRESS  
 *                     0-no keydown event.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
char gokit_key2down()
{
  int unsigned long keep_time=0;
  if( digitalRead(KEY2)==LOW)
  {
    delay(100);
    if(digitalRead(KEY2)==LOW)
    {
      keep_time = gokit_time_s();
      while(digitalRead(KEY2)==LOW)//until open the key 
      {

        if( (gokit_time_s()-keep_time)> KEY_LONG_TIMER)
        {
          return KEY2_LONG_PRESS;
         }  
      }
 
      return KEY2_SHORT_PRESS;
    }
    return 0;
  }
  return 0;
}
/*******************************************************
 *    function      : gokit_keydown
 *    Description   : check the gokit key1 or key2 event 
 *    return        : KEY1_LONG_PRESS  KEY1_SHORT_PRESS 
 *                    KEY2_LONG_PRESS  KEY2_SHORT_PRESS   
 *                     0-no keydown event.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
char gokit_keydown()
{
  char ret=0;
  ret |= gokit_key2down();
  ret |= gokit_key1down();
  return ret;

}
/*******************************************************
 *    function      : gokit_IR_event
 *    Description   : check the gokit infrared event 
 *    return        : none 
 *                   
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void gokit_IR_event()
{
  noInterrupts();
  if(digitalRead(2))
  {
    #if (DEBUG==1)
    Serial.println("gokit_IR_event! no happen. ");
    #endif
    m_m2w_mcuStatus.status_r.ir_status &= ~(1<<0);
  }
  else
  {
    #if (DEBUG==1)
    Serial.println("gokit_IR_event! happen. ");
    #endif
    m_m2w_mcuStatus.status_r.ir_status |= (1<<0);
  }
  interrupts();
}
/*******************************************************
 *    function      : gokit_setColorRGB
 *    Description   : set gokit colorrgb led
 *    return        : none 
 *                   
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void gokit_setColorRGB(byte red, byte green, byte blue)
{
 leds.setColorRGB(0, red, green, blue); 
}
/*******************************************************
 *    function      : gokit_motor_init
 *    Description   : init gokit motor.
 *    return        : none 
 *                   
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void gokit_motor_init()
{
  pinMode(MOTOR_PINA,OUTPUT);
  pinMode(MOTOR_PINB,OUTPUT);
  digitalWrite(MOTOR_PINB,LOW);
  digitalWrite(MOTOR_PINA,LOW);
  gokit_motorstatus(5);
}
/*******************************************************
 *    function      : gokit_motorstatus
 *    Description   : set gokit motor speed.
 *    return        : none 
 *                   
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void gokit_motorstatus( char motor_speed )
{

  unsigned char Temp_motor_speed=0;
  if(motor_speed==5) //停止
  {
    digitalWrite(MOTOR_PINA,LOW);
    digitalWrite(MOTOR_PINB,LOW);
  }
  if(motor_speed>5)//正转
  {
    Temp_motor_speed = (motor_speed-5)*51;
    if(Temp_motor_speed>255) Temp_motor_speed=255;
    digitalWrite(MOTOR_PINA,LOW);
    analogWrite( MOTOR_PINB, Temp_motor_speed);
  }
  if(motor_speed<5)//反转
  {
    Temp_motor_speed = (255-(5+motor_speed))*51;
    if(Temp_motor_speed>255) Temp_motor_speed =255;
    digitalWrite(MOTOR_PINA,HIGH);
    analogWrite( MOTOR_PINB,Temp_motor_speed );
  }
}

void gokit_timer()
{
}
/*******************************************************
 *    function      : gokit_ResetWiFi
 *    Description   : gokit reset the wifi. 
 *    return        : None.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void gokit_ResetWiFi()
{
    gokit_setColorRGB(0,0,50);
    delay(500);
    gokit_setColorRGB(0,0,0);  

    m_pro_commonCmd.head_part.cmd = CMD_RESET_MODULE;
    m_pro_commonCmd.head_part.sn = ++SN;
    m_pro_commonCmd.sum = CheckSum((uint8_t *)&m_pro_commonCmd, sizeof(pro_commonCmd));
    SendToUart((uint8_t *)&m_pro_commonCmd, sizeof(pro_commonCmd), 1);
}
/*******************************************************
 *    function      : WiFi_Config
 *    Description   : Gonfig wifi into airlink
 *    return        : None.
 *    Add by Alex.lin    --2014-12-25
******************************************************/
void gokit_sendAirlink()
{
    gokit_setColorRGB(0,50,0);
    m_m2w_setModule.config_info = 0x02;   //air link
    m_m2w_setModule.head_part.sn = ++SN;
    m_m2w_setModule.sum = CheckSum((uint8_t *)&m_m2w_setModule, sizeof(m2w_setModule));
    SendToUart((uint8_t *)&m_m2w_setModule, sizeof(m2w_setModule), 1);
}
void gokit_sendApCmd()
{
    gokit_setColorRGB(50,0,0);
    delay(500);
    gokit_setColorRGB(0,0,0);   
     
    m_m2w_setModule.config_info = 0x01;   //air link
    m_m2w_setModule.head_part.sn = ++SN;
    m_m2w_setModule.sum = CheckSum((uint8_t *)&m_m2w_setModule, sizeof(m2w_setModule));
    SendToUart((uint8_t *)&m_m2w_setModule, sizeof(m2w_setModule), 1);

}
void gokit_ReportStatus(uint8_t tag)
{
  if(tag == REPORT_STATUS)
  {
    m_m2w_mcuStatus.head_part.cmd = CMD_SEND_MODULE_P0;
    m_m2w_mcuStatus.head_part.sn = ++SN;
    m_m2w_mcuStatus.sub_cmd = SUB_CMD_REPORT_MCU_STATUS;
    m_m2w_mcuStatus.status_w.motor_speed = exchangeBytes(m_m2w_mcuStatus.status_w.motor_speed);
    m_m2w_mcuStatus.sum = CheckSum((uint8_t *)&m_m2w_mcuStatus, sizeof(m2w_mcuStatus));
    SendToUart((uint8_t *)&m_m2w_mcuStatus, sizeof(m2w_mcuStatus), 0);
  }
  else if(tag == REQUEST_STATUS)
  {
    m_m2w_mcuStatus.head_part.cmd = CMD_SEND_MCU_P0_ACK;
    m_m2w_mcuStatus.head_part.sn = m_w2m_controlMcu.head_part.sn;
    m_m2w_mcuStatus.sub_cmd = SUB_CMD_REQUIRE_STATUS_ACK;
    m_m2w_mcuStatus.status_w.motor_speed = exchangeBytes(m_m2w_mcuStatus.status_w.motor_speed);
    m_m2w_mcuStatus.sum = CheckSum((uint8_t *)&m_m2w_mcuStatus, sizeof(m2w_mcuStatus));
    SendToUart((uint8_t *)&m_m2w_mcuStatus, sizeof(m2w_mcuStatus), 0);
  }
    

  m_m2w_mcuStatus.status_w.motor_speed = exchangeBytes(m_m2w_mcuStatus.status_w.motor_speed);
  memcpy(&m_m2w_mcuStatus_reported, &m_m2w_mcuStatus, sizeof(m2w_mcuStatus));
}
