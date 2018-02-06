/* 
  Check the new incoming messages, and print via serialin 115200 baud rate.
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/

#include <SPI.h>
#include <LoRa.h>


#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16

 SSD1306  display(0x3c, 4, 15);


// Pin definetion of WIFI LoRa 32
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define BAND    868.10E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true

//app defines
//resolution1000msec -> 30sec
#define TIME_OUT_RCV_PACKET  50
//#define TIME_OUT_RCV_PACKET  10

void init_diplay()
{
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  display.init();

  // display.flipScreenVertically();

  display.setContrast(255);
}


void setup() {
  init_diplay();
  display.clear();
  //display.setTextSize(1);
  //display.setColor(WHITE);
  //display.setCursor(0,0);
  display.setLogBuffer(5, 30);
  display.println("LoRa Receiver");
  display.drawLogBuffer(0, 0);
  display.display();
#if 0
  // Initialize the log buffer
  // allocate memory to store 8 lines of text and 30 chars per line.
  display.setLogBuffer(5, 30);
  // Print to the screen
  display.println("LoRa Receiver:");
  // Draw it to the internal screen buffer
  display.drawLogBuffer(0, 0);
  // Display it on the screen
  display.display();
#endif
  Serial.begin(9600);
  while (!Serial); //test this program,you must connect this board to a computer
  Serial.println("LoRa Receiver");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND,PABOOST )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSignalBandwidth(125e3);
  LoRa.setSpreadingFactor(11);
}

int time_out =0;

void loop() {
  // try to parse packet
  char ch;
  delay(1000);
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    time_out =0;
    // received a packet
    display.clear();
    display.print("Rcv:");
    Serial.print("Received packet '");
    int hTab=0;
    // read packet
    while (LoRa.available()) {
      ch=(char)LoRa.read();
      display.print((char)ch);
      Serial.print(ch);
      hTab++;
      //add new line <CR>
      if (hTab>=21) //accepts 23 also .Lowered to show tes message beter
      {
        display.println(" ");
        hTab=0;
      }
    }
    // print RSSI of packet
    display.print("' with RSSI ");
    display.println(LoRa.packetRssi());
    display.drawLogBuffer(0, 0);
    display.display();
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  else
  {
    time_out++;
    if (time_out> TIME_OUT_RCV_PACKET)
    {
      display.clear();
      display.println("RCV_ TIMEOUT !!!");
      display.drawLogBuffer(0, 0);
      display.display();
    }
    //Serial.println("no lora packet");
  }
}
