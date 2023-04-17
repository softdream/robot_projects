#include "ultrasonic.h"

void ultraSonicInit()
{
    pinMode( Trig, OUTPUT );
    pinMode( Echo, INPUT );

    digitalWrite( Trig, LOW );
}

float getDistance()
{
    digitalWrite( Trig, HIGH );
    delayMicroseconds(10);
    digitalWrite( Trig, LOW );

    float dist = pulseIn( Echo, HIGH ) * 0.034 / 2;

    return dist;
}
