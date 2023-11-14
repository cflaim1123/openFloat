#ifndef SENSOR_READINGS_H
#define SENSOR_READINGS_H

struct SensorReadings {
    float dht22Humid;
    float vdht22Temp;
    float bmp180Press;
    float bmp180Temp;
};

#endif  // SENSOR_READINGS_H