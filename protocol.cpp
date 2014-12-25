#include "protocol.h"
#include "GoKit.h"
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
        mySerial.println("CheckSum error!");
        return ;
    }    
    mySerial.print("sn = ");
    mySerial.println(tmp_headPart.sn);
    switch( tmp_headPart.cmd )
    {
      case CMD_GET_MCU_INFO :
          mySerial.println("CMD_GET_MCU_INFO");
          CmdGetMcuInfo(tmp_headPart.sn);
      
      break;
      case CMD_SEND_MCU_P0 :
            mySerial.println("CMD_SEND_MCU_P0");
      break;
      case CMD_SEND_HEARTBEAT:
            mySerial.println("CMD_SEND_HEARTBEAT");
            SendCommonCmd(CMD_SEND_HEARTBEAT_ACK,tmp_headPart.sn);
      break;
      case CMD_REPORT_MODULE_STATUS:
            mySerial.println("CMD_REPORT_MODULE_STATUS");
      break;
      default:
            mySerial.println("default");
            SendErrorCmd(ERROR_CMD, tmp_headPart.sn);
      break;
    }
}
void Handle_keyeven()
{

}
void Check_Status()
{

}
void GoKit_Handle()
{
  Handle_uartdata( uart_buf,get_onepackage( uart_buf ));
  Handle_keyeven();
  Check_Status();
}

