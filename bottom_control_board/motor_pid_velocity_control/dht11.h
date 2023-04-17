#ifndef __DHT11_H
#define __DHT11_H

#include <DFRobot_DHT11.h>

#define DHT11_PIN 16

void getHumidityAndTemperature( int& humidity, int& temperature );

#endif
