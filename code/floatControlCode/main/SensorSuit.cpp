#include "SensorSuit.h"
#include "DHT22_humidity.h"

SensorSuit::SensorSuit() : dht22Humidity(&dht){}

void SensorSuite::initializeAll() {
    dht22Humidity.init();

}

void SensorSuite::sampleAll() {
    SensorReadings dht22Readings = dht22Humidity.sample();
    // Sample other sensors as needed
    // You might want to store or process the sampled values here
}

bool SensorSuite::selfTestAllSensors() {
    if(dht22Humidity.selfTest()){
        return true;
    }else{
        return false;
        }

}

SensorReadings SensorSuite::sampleScienceSensors(){
    return NULL;
}

SensorReadings SensorSuite::sampleEngSensors(){
    dht22Reading = dht22Humidity.sample();

    return dht22Reading;
}
// SensorReadings sampleScienceSensors();
// SensorReadings sampleEngSensors();
// SensorReadings sampleDepth();
// SensorReadings sampleTemp();
