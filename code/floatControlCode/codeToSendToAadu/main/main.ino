#include <Wire.h>
#include "RTClib.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

////////// May not nedd ////////// 
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MS5803_05.h> 
#include <Adafruit_9DOF.h>
////////// May not nedd ////////// 

#include "DHT22_humidity.h"
#include "SensorSuit.h"

////////// sensors on board ////////// 
// - pcf8523 RTC
// - DHT22 temperature and humidity
// - DS18b20 temperature probe (x3)
// - INA219 current meter
// - BNO055 9-axis abosute postioning IMU
// - BMP180 temperature and pressure
// - NTC 10K Thermistor
// - adafruit ultimate gps
// - MS8503-05 BA
// - TSl2591 lux sensor


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

DHT_Unified dht(DHT22_DATA_PIN, DHTTYPE);
DHT22Humidity dht22Humidity(&dht);

SensorSuit sensorSuit(&dht22Humidity);


const int baud = 9600;

void setup() {
  Serial.begin(baud);

  sensorSuit.init();
  sensorSuit.selfTestAllSensors();
  Serial.println(sensorSuit.sampleEngSensors());

}

void loop() {
  // put your main code here, to run repeatedly:

}




