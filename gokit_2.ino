#include <DHT.h>
#include <I2Cdev.h>
#include <MemoryFree.h>
#include <MsTimer2.h>
#include <SSD1306.h>
#include <ChainableLED.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Wire.h"
#include "GoKit.h"


#define OLED_DC 9
#define OLED_CS 10
#define OLED_CLK 13
#define OLED_MOSI 11
#define OLED_RESET 8

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void setup()
{

  GoKit_Init();
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.drawstring(0,3,"   ---- GoKit ----   ");
  oled.display();
  #if (DEBUG==1)
  Serial.println("GoKit init  OK!");
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
  #endif
}
void loop()
{  
  GoKit_Handle();
}

