#ifndef SENSOR_READINGS_H
#define SENSOR_READINGS_H

#include <vector>

struct SensorReadings {
    float dht22Humid;
    float vdht22Temp;
    float bmp180Press;
    float bmp180Temp;
};

struct AllData {
    std::vector<SensorReadings> sensorReadings;
};

#endif  // SENSOR_READINGS_H