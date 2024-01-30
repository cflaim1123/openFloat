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

// internal humidity
// #include <DHT.h>
// #include <DHT_U.h>
// IMU
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// internal pressure 
#include <SFE_BMP180.h>
// sd card reader and clock backpack
#include "RTClib.h"
#include "FS.h"
#include <SD.h>
// GPS
#include <Adafruit_GPS.h>
// External pressure
// #include "MS5803_05.h" 

////////// GPS serial port ////////// 
#define GPSSerial Serial1
////////// sensors pins ////////// 
// External digital temp
#define DS18B20_DATA_PIN 37
// Internal temp and humidity
#define DHT22_DATA_PIN 15
// Internal pressure
#define BMP180_power 32 // need to make sure sensor is grounded to controller
// SD reader
#define SD_CS 33 //const int8_t SD_CS = 33;
// Radio
#define RADIO_CS 13
#define RADIO_EN 25 // A1
#define RADIO_GO 12
#define RADIO_RST 27
// GPS
#define GPS_EN 26 // A0
// Thermistor
#define THERMISTOR_ADC 36 // A4

uint64_t  floatStart = millis();

// Make data structure to hold float sensor values
struct FloatData{
  uint64_t codeStartTime = floatStart;
  DateTime turnOnTime;

  struct Bmp180Reading{
    float temp;
    float press;
  } bmp180Reading;

  struct Time{
    DateTime dateAndTime;
  } time;

  struct AccelerometerReading{
    int8_t boardTemp;

    struct Orientation{
      float X;
      float Y;
      float Z;
    } orientation;

    struct Gyro{
      float X;
      float Y;
      float Z;
    } gyro;

    struct Linear{
      float X;
      float Y;
      float Z;
    } linear; 

    struct Mag{
      float X;
      float Y;
      float Z;
    } mag; 

    struct Accl{
      float X;
      float Y;
      float Z;
    } accl;

    struct Gravity{
      float X;
      float Y;
      float Z;
    } gravity;


  } accelerometerReading;


  struct GpsReading {
    DateTime time;
    // uint8_t hour, minute, seconds, day, month;
    // uint16_t year;
    float latitude, longitude, speed, altitude;
    // float angle;
    uint8_t fixquality;
    // unint8_t fix, satellites, antenna;
  } gpsReading;
} floatData;

////////// create sensor suit ////////// 
// #define DHTTYPE DHT22

// make sensor objects
// board 1
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
SFE_BMP180 bmp180;
RTC_PCF8523 rtc;
Adafruit_GPS GPS(&GPSSerial);

DateTime now;
int missionNum = 0;
char globalTime[] = "%02d/%02d/%02d %02hhu:%02hhu:%02hhu";
char dataSavePath[64];
String tempHolder;
// DHT_Unified dht(DHT22_DATA_PIN, DHTTYPE);

/*******************************************************************************/
////////// main loop ////////// 
void setup() {
  Serial.begin(115200);
  SPI.begin(SCK, MISO, MOSI, SD_CS);
  Wire.begin();
  init();
}

void loop() {
  sample();
  writeToFile(dataSavePath);
  printEverything();
  delay(1000);
}


/*******************************************************************************/


////////// inits ////////// 

void init(){
  initBoard1();
  initBoard2();
}

void initBoard1(){
  // initBNO055();
  initSdReader();
  delay(50);
  initPcf8523();
  delay(50);
  initBmp180();
  delay(50);
  initBNO055();
}

void initBoard2(){
  initGps();
}

void initSdReader(){
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.println("SD card init success");
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

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  sprintf(dataSavePath, "/mission%ddata.csv", missionNum);
  if(!SD.exists(dataSavePath)){
    char fileTop[64];
    char equippedSensors[] = "openFloat is outfitted with bmp180, pcf8523, microsd reader,\nBNO055, adafruit ultimate gps, 915 mhz LoRa module, DHT22, DS18b20(x3),\nthermistor, MS8503-05 BA, TSl2591, INA219\n";
    char fileHead[] = "Time, bmpInternalTemp, bmpInternalPress, orientX, orientY, orientZ, gyroX, gyroY, gyroZ, linearX, linearY, linarZ, magX, magY, magZ, acclX, accY, accZ, gravX, gravY, gravZ, latitude, longitude, gpsAltitude, gpsFixQuality, gpsSpeed, gpsDateTime\n";
  
    sprintf(fileTop, "Mission %d on %02d/%02d/%02d\n", missionNum, 
            floatData.turnOnTime.year(), floatData.turnOnTime.month(), floatData.turnOnTime.day());
    writeGeneralInfoToCard(dataSavePath, fileTop, equippedSensors, fileHead);
  }
}

void initPcf8523(){
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("ERROR: RTC is NOT initialized -- setting the time");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtc.start();
    Serial.println("RTC init success");
  }
  now = rtc.now();
  floatData.turnOnTime = now;
  char time[36];
  // char[64] time;
  sprintf(time,"%02d/%02d/%02d %02hhu:%02hhu:%02hhu", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  Serial.println("Turn on time: ");
  Serial.println(time);
  strcpy(globalTime, time);
}

void initBNO055(){
  if(!bno.begin()){
    Serial.print("ERROR: BNO055 not detected");
    while(1);
  }
  delay(100); 
  Serial.println("BNO055 init success");
}

void initBmp180(){
  digitalWrite(BMP180_power, HIGH);
  if (bmp180.begin()){
    Serial.println("BMP180 init success");
  } else{
    Serial.println("ERROR: BMP180 not detected");
    while(1); 
  }
  digitalWrite(BMP180_power, LOW);
}

void initGps(){
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  Serial.println("Adafruit GPS init success");
}


/*******************************************************************************/


////////// read sensors ////////// 
void sample(){
  sampleBmp180();
  sampleTime();
  sampleBNO055();
  sampleGps();

}

void sampleBmp180(){
  digitalWrite(BMP180_power, HIGH);
  char status;
  double T,P,p0,a;  

  status = bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp180.getTemperature(T);
    if (status != 0)
    {
      floatData.bmp180Reading.temp = T;
      // Print out the measurement:
      // Serial.print("temperature: ");
      // Serial.print(T,2);
      // Serial.print(" deg C, ");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp180.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp180.getPressure(P,T);
        if (status != 0)
        {
          floatData.bmp180Reading.press = P;
          // Print out the measurement:
          // Serial.print("absolute pressure: ");
          // Serial.print(P,2);
          // Serial.print(" mb, ");

        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
  digitalWrite(BMP180_power, LOW);
}

void sampleTime(){
  floatData.time.dateAndTime = rtc.now();
}

void sampleBNO055(){
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  floatData.accelerometerReading.orientation.X = orientationData.orientation.x;
  floatData.accelerometerReading.orientation.Y = orientationData.orientation.y;
  floatData.accelerometerReading.orientation.Z = orientationData.orientation.z;
  
  floatData.accelerometerReading.gyro.X = angVelocityData.gyro.x;
  floatData.accelerometerReading.gyro.Y = angVelocityData.gyro.y;
  floatData.accelerometerReading.gyro.Z = angVelocityData.gyro.z;

  floatData.accelerometerReading.linear.X = linearAccelData.acceleration.x;
  floatData.accelerometerReading.linear.Y = linearAccelData.acceleration.y;
  floatData.accelerometerReading.linear.Z = linearAccelData.acceleration.z;

  floatData.accelerometerReading.mag.X = magnetometerData.magnetic.x;
  floatData.accelerometerReading.mag.Y = magnetometerData.magnetic.y;
  floatData.accelerometerReading.mag.Z = magnetometerData.magnetic.z;

  floatData.accelerometerReading.accl.X = accelerometerData.acceleration.x;
  floatData.accelerometerReading.accl.Y = accelerometerData.acceleration.y;
  floatData.accelerometerReading.accl.Z = accelerometerData.acceleration.z;

  floatData.accelerometerReading.gravity.X = gravityData.acceleration.x;
  floatData.accelerometerReading.gravity.Y = gravityData.acceleration.y;
  floatData.accelerometerReading.gravity.Z = gravityData.acceleration.z;

  int8_t boardTemp = bno.getTemp();
  floatData.accelerometerReading.boardTemp = boardTemp;

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  // Serial.println("--");
}

void sampleGps(){
  while (!GPS.available());
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    // Parse the received NMEA sentence
    if (GPS.parse(GPS.lastNMEA())) {
      // Extract data from the GPS object
      // floatData.gpsReading.hour = GPS.hour;
      // floatData.gpsReading.minute = GPS.minute;
      // floatData.gpsReading.seconds = GPS.seconds;
      // floatData.gpsReading.day = GPS.day;
      // floatData.gpsReading.month = GPS.month;
      // floatData.gpsReading.year = GPS.year;
      floatData.gpsReading.latitude = GPS.latitude;
      floatData.gpsReading.longitude = GPS.longitude;
      floatData.gpsReading.speed = GPS.speed;
      // floatData.gpsReading.angle = GPS.angle;
      floatData.gpsReading.altitude = GPS.altitude;
      // floatData.gpsReading.fix = GPS.fix;
      floatData.gpsReading.fixquality = GPS.fixquality;
      // floatData.gpsReading.satellites = GPS.satellites;
      // floatData.gpsReading.antenna = GPS.antenna;

      int year = GPS.year + 2000;
      floatData.gpsReading.time = DateTime(year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);

      // Note: SHOULD MAKE STRUCT HOLD CHAR LIST AS TIME INSTEAD OF DATETIME SO IT CAN BE PRINTED
    }
  }
}

////////// write data to sd card ////////// 
void writeGeneralInfoToCard(const char *dataSavePath, const char *fileTop, const char*equippedSensors, const char *fileHead){
  // if SD.eists()
  appendFile(SD, dataSavePath, fileTop);
  // Serial.println(dataSavePath);
  Serial.println(fileTop);
  appendFile(SD, dataSavePath, equippedSensors);
  Serial.println(equippedSensors);
  appendFile(SD, dataSavePath, fileHead);
  Serial.println(fileHead);
}

void writeToFile(const char *dataSavePath){
  char dataLine[1000];
  // "Time, bmpInternalTemp, bmpInternalPress, orientX, orientY, orientZ, gyroX, gyroY, gyroZ, linearX, linearY, linarZ, magX, magY, magZ, acclX, accY, accZ, gravX, gravY, gravZ, latitude, longitude, gpsAltitude, gpsFixQuality, gpsSpeed, gpsDateTime\n";
  sprintf(dataLine, "%02d/%02d/%02d %02hhu:%02hhu:%02hhu, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %f, %f, %f, %.2d, %.2f, %02d/%02d/%02d %02hhu:%02hhu:%02hhu\n", floatData.time.dateAndTime.year(), 
          floatData.time.dateAndTime.month(), floatData.time.dateAndTime.day(), 
          floatData.time.dateAndTime.hour(), floatData.time.dateAndTime.minute(), 
          floatData.time.dateAndTime.second(), floatData.bmp180Reading.temp, floatData.bmp180Reading.press,
          floatData.accelerometerReading.orientation.X, floatData.accelerometerReading.orientation.Y, 
          floatData.accelerometerReading.orientation.Z, floatData.accelerometerReading.gyro.X,
          floatData.accelerometerReading.gyro.Y, floatData.accelerometerReading.gyro.Z,
          floatData.accelerometerReading.linear.X, floatData.accelerometerReading.linear.Y,
          floatData.accelerometerReading.linear.Z, floatData.accelerometerReading.mag.X,
          floatData.accelerometerReading.mag.Y, floatData.accelerometerReading.mag.Z,
          floatData.accelerometerReading.accl.X, floatData.accelerometerReading.accl.Y,
          floatData.accelerometerReading.accl.Z, floatData.accelerometerReading.gravity.X,
          floatData.accelerometerReading.gravity.Y, floatData.accelerometerReading.gravity.Z,
          floatData.gpsReading.latitude, floatData.gpsReading.longitude, floatData.gpsReading.altitude,
          floatData.gpsReading.fixquality, floatData.gpsReading.speed, floatData.gpsReading.time);

  appendFile(SD, dataSavePath, dataLine);
}



void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.print (file.name());
            time_t t= file.getLastWrite();
            struct tm * tmstruct = localtime(&t);
            Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour , tmstruct->tm_min, tmstruct->tm_sec);
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.print(file.size());
            time_t t= file.getLastWrite();
            struct tm * tmstruct = localtime(&t);
            Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour , tmstruct->tm_min, tmstruct->tm_sec);
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

/****************************************************************************/

void printEverything(){
  Serial.print("Internal temp: ");
  Serial.println(floatData.bmp180Reading.temp);
  Serial.print("Internal pressure: ");
  Serial.println(floatData.bmp180Reading.press);
  sprintf(globalTime,"%02d/%02d/%02d %02hhu:%02hhu:%02hhu", floatData.time.dateAndTime.year(), 
          floatData.time.dateAndTime.month(), floatData.time.dateAndTime.day(), 
          floatData.time.dateAndTime.hour(), floatData.time.dateAndTime.minute(), 
          floatData.time.dateAndTime.second());

  Serial.print("Time: ");
  Serial.println(globalTime);
  Serial.print("Accel board temp: ");
  Serial.println(floatData.accelerometerReading.boardTemp);
  Serial.print("Orientation: X: ");
  Serial.print(floatData.accelerometerReading.orientation.X);
  Serial.print(" Y: ");
  Serial.print(floatData.accelerometerReading.orientation.Y);
  Serial.print(" Z: ");
  Serial.println(floatData.accelerometerReading.orientation.Z);
  Serial.print("Angular vel: X: ");
  Serial.print(floatData.accelerometerReading.gyro.X);
  Serial.print(" Y: ");
  Serial.print(floatData.accelerometerReading.gyro.Y);
  Serial.print(" Z: ");
  Serial.println(floatData.accelerometerReading.gyro.Z);
  Serial.print("Linear accel: X: ");
  Serial.print(floatData.accelerometerReading.linear.X);
  Serial.print(" Y: ");
  Serial.print(floatData.accelerometerReading.linear.Y);
  Serial.print(" Z: ");
  Serial.println(floatData.accelerometerReading.linear.Z);
  Serial.print("Magnetometer: X: ");
  Serial.print(floatData.accelerometerReading.mag.X);
  Serial.print(" Y: ");
  Serial.print(floatData.accelerometerReading.mag.Y);
  Serial.print(" Z: ");
  Serial.println(floatData.accelerometerReading.mag.Z);
  Serial.print("Acceleration: X: ");
  Serial.print(floatData.accelerometerReading.accl.X);
  Serial.print(" Y: ");
  Serial.print(floatData.accelerometerReading.accl.Y);
  Serial.print(" Z: ");
  Serial.println(floatData.accelerometerReading.accl.Z);
  Serial.print("Gravity: X: ");
  Serial.print(floatData.accelerometerReading.gravity.X);
  Serial.print(" Y: ");
  Serial.print(floatData.accelerometerReading.gravity.Y);
  Serial.print(" Z: ");
  Serial.println(floatData.accelerometerReading.gravity.Z);
  Serial.print("Lat: ");
  Serial.println(floatData.gpsReading.latitude);
  Serial.print("Long: ");
  Serial.println(floatData.gpsReading.longitude);
  Serial.print("GPS fix quality: ");
  Serial.println(floatData.gpsReading.fixquality);
  Serial.print("GPS altitude: ");
  Serial.println(floatData.gpsReading.altitude);
  Serial.print("GPS speed: ");
  Serial.print(floatData.gpsReading.speed);
  // Serial.print("GPS time: ");
  // Serial.println(floatData.gpsReading.time);
  Serial.println();
}



