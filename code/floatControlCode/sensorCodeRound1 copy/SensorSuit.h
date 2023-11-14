#ifndef SENSOR_SUIT_H
#define SENSOR_SUIT_H

#include "DHT22_humidity.h"
#include "sensorReadings.h"

class SensorSuit{
public:
    void initAllSensors();
    void selfTestAllSensors();
    SensorReadings sampleScienceSensors();
    SensorReadings sampleEngSensors();

private:
    DHT22Humidity dht22Humidity;

};

#endif // SENSOR_SUIT_H