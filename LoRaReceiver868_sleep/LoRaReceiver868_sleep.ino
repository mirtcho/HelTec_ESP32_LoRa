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
#include <esp_deep_sleep.h>

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
//resolution5000msec -> 50sec
#define TIME_OUT_RCV_PACKET  10

/* deepsleep definitions */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int time_out =0;

/*
Method to print the reason by which ESP32
has been awaken from deep sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}


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

void setup_deep_sleep()
{
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
}

void do_setup(int boot_cnt)
{
  init_diplay();
  display.clear();
  display.setLogBuffer(5, 30);
  display.println("LoRa Receiver");
  display.drawLogBuffer(0, 0);
  display.display();
  
  Serial.begin(9600);
  while (!Serial); //test this program,you must connect this board to a computer
  Serial.println("LoRa Receiver");
  
  pinMode(SS, OUTPUT);
  //pinMode(RST, OUTPUT);
  pinMode(DI0, INPUT);
  //digitalWrite(RST, HIGH);
  delay(50);
  // set SS high
  digitalWrite(SS, HIGH);
  SPI.begin(SCK,MISO,MOSI,SS);
  //Serial.print("after boot REG_OP_MODE="); Serial.println(LoRa.readRegister(0x1));
  
  LoRa.setPins(SS,RST,DI0);
  if (boot_cnt<=2)
  {
    if (!LoRa.begin(BAND,PABOOST )) {
      Serial.println("Starting LoRa failed!");
      while (1);
    }
    LoRa.setSignalBandwidth(125e3);
    LoRa.setSpreadingFactor(11);
  }
  if (boot_cnt==2){
    Serial.println("Dump after setup boot_cnt==2");
    LoRa.dumpRegisters(Serial);
  }
  //if (boot_cnt==20){
  //  LoRa.dumpRegisters(Serial);
  //  while (1){
  //  }
  //}
}

void do_loop ()
{
  // try to parse packet
  char ch;
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
    delay(10000);
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

void setup() {
  do_setup (bootCount);
  if (bootCount!=0)
  {
    do_loop();
  }
  /* Initialize deep sleep periphery */
  setup_deep_sleep();

  bootCount++;
  //Serial.print("before sleep REG_OP_MODE="); Serial.println(LoRa.readRegister(0x1));
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {

}
