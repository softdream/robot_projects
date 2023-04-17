#ifndef __PID_H
#define __PID_H

#include <Arduino.h>

class PID
{
public:
    PID();

    PID( const float dt, const float max, const float min, const float kp, const float ki, const float kd ); 

    ~PID();

    const float caculate( const float& target, const float& current );

private:
    float dt_;
    float max_;
    float min_;
    float kp_;
    float ki_;
    float kd_;
    float pre_error_ = 0;
    float integral_ = 0;
};

#endif
