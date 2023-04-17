#include "pid.h"

PID::PID() : dt_( 0.05 ), max_( 255 ), min_(-255), kp_(10), kd_(0), ki_(1.1)
{

}

PID::PID( const float dt, const float max, const float min, const float kp, const float ki, const float kd )
:   dt_(dt), max_(max), min_(min), kp_(kp_), ki_(ki), kd_(kd)
{

}

PID::~PID()
{

}

const float PID::caculate( const float& target, const float& current )
{
    // error
    float error = target - current;
    
    // Proportional
    float p_out = kp_ * error;

    // Integral
    integral_ += error;
    if( target == 0 && error == 0 ){
      integral_ = 0;
    }
    
    float i_out = ki_ * integral_;

    // Derivative
    float derivative = ( error - pre_error_ );
    float d_out = kd_ * derivative;

    // total output 
    float out = p_out + i_out + d_out;

    // limit
    if( out > max_ ){
            out = max_;
    }
    else if( out < min_ ){
            out = min_;
    }

    // update the pre_error
    pre_error_ = error;

    return out;
}
