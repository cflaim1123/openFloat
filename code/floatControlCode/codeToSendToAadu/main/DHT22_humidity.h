#ifndef DHT22_HUMIDITY_H
#define DHT22_HUMIDITY_H

#include <Arduino.h>
#include <DHT_U.h>
#include "SensorReadings.h"
#include "Sensor.h"

class DHT22Humidity : public Sensor{
public:
    DHT22Humidity(DHT_Unified*);
    
    void init() override;
    SensorReadings sample() override;
    bool selfTest() override;

private:
    DHT_Unified* dht;

};

#endif // DHT22_HUMIDITY_H