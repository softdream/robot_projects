#ifndef __IMU_H
#define __IMU_H

#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define IMU_SDA 5
#define IMU_SCL 18

typedef struct ImuData_
{
    float ax;
    float ay;
    float gz;
}ImuData;

typedef struct ImuStatus_
{
    
}ImuStatus;

void imuInit();
ImuData getImuData();

#endif
