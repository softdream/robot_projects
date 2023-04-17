#include "dht11.h"

DFRobot_DHT11 DHT;

void getHumidityAndTemperature( int& humidity, int& temperature )
{
    DHT.read( DHT11_PIN );

    humidity = DHT.humidity;
    temperature = DHT.temperature;
}
