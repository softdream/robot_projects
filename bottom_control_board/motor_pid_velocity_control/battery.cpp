#include "battery.h"

float getBatteryVoltage()
{
    int adc_value = analogRead( Battery_Pin );
    float voltage = ( ( (float)adc_value ) / 4095 ) * 16.5; // * 3.3 * 5;

    return voltage;
}
