#ifndef DHT22_HUMIDITY_H
#define DHT22_HUMIDITY_H

#include "sensorReadings.h"

class SensorSuit; // Forward declarationt to avoid circular dependency

class DHT22Humidity{
public:

    void init();
    SensorReadings sample();
    bool selfTest();

private:
    const int dataPin = 15;
    SensorSuit* sensorSuit;

};

#endif // DHT22_HUMIDITY_H