#ifndef __MOTOR_H
#define __MOTOR_H

#include <Arduino.h>
#include <Ticker.h>

#include "pid.h"

#ifndef M_PI
#define M_PI 3.141592653
#endif

// 左轮电机
#define IN1 26  // 电机1控制引脚1
#define IN2 27  // 电机1控制引脚2
#define PWMA 14 // 电机1 pwm引脚

// 右轮电机
#define IN3 25 // 电机2控制引脚1
#define IN4 4  // 电机2控制引脚2
#define PWMB 0  // 电机2 pwm引脚

#define FREQ 50000 // pwm频率

#define pwm_Channel_A  0 //电机1使用PWM的通道0
#define pwm_Channel_B  1 //电机1使用PWM的通道1

#define resolution  8    //使用PWM占空比的分辨率
#define interrupt_time_control 50//定时器50ms中断控制时间

#define LA 13 // 电机1编码器 A 相位
#define LB 12 // 电机1编码器 B 相位
#define RA 2  // 电机2编码器 A 相位
#define RB 15 // 电机2编码器 B 相位

// --------------------- 车身参数 --------------------- //
#define circumference 0.132889 // 轮子的周长，单位m
#define base_width 0.166 // 车身宽度，单位m
#define MAX_RPM 300 // 电机最大转数
#define pulse_number 468 // 电机转一圈编码器的脉冲数


// ------------------- 定时器 & 电机 ------------------ //

extern float required_rpm_left;
extern float required_rpm_right;

// 控制量
typedef struct Control_
{
  float v;
  float w;
}Control;


void leftEncoder();
void rightEncoder();

void encoderInit();
int rpmToPWM(float rpm);
void setPwm( int motor1_pwm, int motor2_pwm );

float getVelocity( float motor1_rpm, float motor2_rpm );
float getDeltaS( long l_count_c, long r_count_c );
float getDeltaAngle( long l_count_c, long r_count_c );

void motorControl();
void motorGpioInit();
void motorInit();

// motion model
void cacuRPM( float v, float w );

#endif
