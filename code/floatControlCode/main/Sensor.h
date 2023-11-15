#ifndef SENSOR_H
#define SENSOR_H

#include "SensorData.h"

class Sensor {
public:
    virtual void init() = 0;
    virtual SensorReadings sample() = 0;
    virtual bool selfTest() = 0;
};

#endif  // SENSOR_H