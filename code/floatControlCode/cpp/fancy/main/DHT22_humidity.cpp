
#include <DHT_U.h>

#include "SensorSuit.h"
#include "SensorReadings.h"
#include "DHT22_humidity.h"

DHT22Humidity::DHT22Humidity(DHT_Unified* dht) : dht(dht){}

void DHT22Humidity::init(){dht->begin();}

SensorReadings DHT22Humidity::sample(){
    SensorReadings sensorReadings;
    sensors_event_t event;

    dht->humidity().getEvent(&event);
    if (!isnan(event.temperature)) {
        sensorReadings.temperature = event.temperature;
    } else {
        // Handle error or no reading
    }

    dht->temperature().getEvent(&event);
    if (!isnan(event.relative_humidity)) {
        sensorReadings.humidity = event.relative_humidity;
    } else {
        // Handle error or no reading
    }

    return sensorReadings;
}

bool DHT22Humidity::selfTest(){
    dht->humidity().getEvent(&event);
    if (!isnan(event.temperature) && !isnan(event.relative_humidity)) {
        return true;

    } else {
        return false;
    }
}


