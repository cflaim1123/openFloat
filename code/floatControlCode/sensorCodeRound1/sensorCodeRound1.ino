#include <Wire.h>
#include "RTClib.h"
#include <MS5803_05.h> 
#include "FS.h"
#include "SD.h"
#include "SPI.h"

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

const int baud = 9600;

void setup() {
  Serial.begin(baud);

}

void loop() {
  // put your main code here, to run repeatedly:

}
