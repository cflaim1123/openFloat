////////// sensors and electronics on board ////////// 
// board 1
  // - pcf8523 RTC
  // - microsd reader
  // - BNO055 9-axis abosute postioning IMU
  // - BMP180 temperature and pressure
// board 2
  // - adafruit ultimate gps
// board 3
  // 915 mhz LoRa module
// board 4
  // - DHT22 temperature and humidity
  // - DS18b20 temperature probe (x3)
  // - NTC 10K Thermistor
  // - MS8503-05 BA
  // - TSl2591 lux sensor
// board 5
  // - INA219 current meter
  // - pololu 5V 5A step-down converter


#include <Wire.h>
#include "SPI.h"

#include <Adafruit_Sensor.h>

// board 1
// internal humidity
#include <DHT.h>
#include <DHT_U.h>
// IMU
#include <Adafruit_9DOF.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// internal pressure 
#include <SFE_BMP180.h>
// sd card reader 
#include "RTClib.h"
#include "FS.h"
#include "SD.h"

// External pressure
#include <MS5803_05.h> 

////////// sensors pins ////////// 
// External digital temp
#define DS18B20_DATA_PIN 37
// Internal temp and humidity
#define DHT22_DATA_PIN 15
// Internal pressure
#define BMP180_power 32 // need to make sure sensor is grounded to controller
// SD reader
#define SD_CS 33
// Radio
#define RADIO_CS 13
#define RADIO_EN 25 // A1
#define RADIO_GO 12
#define RADIO_RST 27
// GPS
#define GPS_RST 26 // A0
// Thermistor
#define THERMISTOR_ADC 36 // A4

////////// create sensor suit ////////// 
#define DHTTYPE DHT22

// make sensor objects
// board 1
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SFE_BMP180 bmp180;
RTC_PCF8523 rtc;

DHT_Unified dht(DHT22_DATA_PIN, DHTTYPE);

////////// inits ////////// 
void initSdReader(){
  if(!SD.begin(SD_CS)){
        Serial.println("ERROR: Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("ERROR: No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
}

void initPcf8523(){
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("ERROR: RTC is NOT initialized -- setting the time");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtc.start();
}

void initBNO055(){
  if(!bno.begin()){
    Serial.print("ERROR: BNO055 not detected");
    while(1);
  }
  delay(1000); 
  bno.setExtCrystalUse(true);
}

void initBmp180(){
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("ERROR: BMP180 not detected");
    while(1); 
  }
}

void init(){
  initBNO055();
  initBmp180();
  initPcf8523();
  initSDReader();

}

////////// read sensors ////////// 
void sample(){

}
////////// write data to sd card ////////// 

////////// main loop ////////// 
void setup() {
  Serial.begin(115200);
  init();

}

void loop() {
  // put your main code here, to run repeatedly:

}
