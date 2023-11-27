#ifndef SENSOR_SUIT_H
#define SENSOR_SUIT_H

#include <Arduino>
#include "DHT22_humidity.h"
#include "SensorReadings.h"

class SensorSuit{
public:
    SensorSuit(DHT22Humidity*);

    void initAllSensors();
    bool selfTestAllSensors();
    SensorReadings sampleScienceSensors();
    SensorReadings sampleEngSensors();
    SensorReadings sampleDepth();
    SensorReadings sampleTemp();

private:
    DHT22Humidity dht22Humidity;

};

#endif // SENSOR_SUIT_H