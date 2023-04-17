#ifndef __SERVER_H
#define __SERVER_H

#include <Arduino.h>


void initMotor();
int calculatePWM(int degree);

void motor1Run( int degree );
void motor2Run( int degree );

void motor3Run( int degree );
void motor4Run( int degree );

#endif
