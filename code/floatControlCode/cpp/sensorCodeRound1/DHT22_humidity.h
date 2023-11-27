#ifndef DHT22_HUMIDITY_H
#define DHT22_HUMIDITY_H

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "SensorData.h"
#include "Sensor.h"

class DHT22Humidity : public Sensor{
public:
    DHT22Humidity()
    void init();
    SensorReadings sample();
    bool selfTest();

private:
    static const int dataPin = 15;


};

#endif // DHT22_HUMIDITY_H