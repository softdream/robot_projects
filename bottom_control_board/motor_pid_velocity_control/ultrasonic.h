#ifndef __ULTRA_SONIC_H
#define __ULTRA_SONIC_H

#include <Arduino.h>

#define Trig 22
#define Echo 23

void ultraSonicInit();
float getDistance();

#endif
