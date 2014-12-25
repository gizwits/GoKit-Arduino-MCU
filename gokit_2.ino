#include <SSD1306.h>
#include <MsTimer2.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "GoKit.h"

#define OLED_DC 9
#define OLED_CS 10
#define OLED_CLK 13
#define OLED_MOSI 11
#define OLED_RESET 8
unsigned char uart_buf[1024]={0};
SoftwareSerial mySerial(2, 3); // RX, TX
//SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
void setup()
{

  GoKit_Init();
//  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
//  oled.drawstring(0,4,"   ---- GoKit ----   ");
 // oled.display();
  mySerial.println("GoKit init  OK!");

  
}
int len=0,i=0;
void loop()
{
 GoKit_Handle();
}

