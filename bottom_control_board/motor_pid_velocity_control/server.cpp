#include "server.h"

const int freq = 50;      // 频率(20ms周期)
const int channel1 = 8;    // 通道(高速通道（0 ~ 7）由80MHz时钟驱动，低速通道（8 ~ 15）由 1MHz 时钟驱动。)
const int channel2 = 9;
const int channel3 = 10;
const int channel4 = 11;

const int resolution = 8; // 分辨率
const int motor1 = 18;
const int motor2 = 19;
const int motor3 = 5;
const int motor4 = 4;

void initMotor()
{
    ledcSetup(channel1, freq, resolution); // 设置通道
    ledcAttachPin(motor1, channel1);          // 将通道与对应的引脚连接

    ledcSetup(channel2, freq, resolution);
    ledcAttachPin(motor2, channel2);

    ledcSetup(channel3, freq, resolution);
    ledcAttachPin(motor3, channel3);

    ledcSetup(channel4, freq, resolution);
    ledcAttachPin(motor4, channel4);
}

int calculatePWM(int degree)
{   //0-180度
    //20ms周期，高电平0.5-2.5ms，对应0-180度角度
    const float deadZone = 6.4;//对应0.5ms（0.5ms/(20ms/256）)
    const float max = 32;//对应2.5ms

    if (degree < 0)
    degree = 0;
    if (degree > 180)
    degree = 180;
    return (int)(((max - deadZone) / 180) * degree + deadZone);
}

void motor1Run( int degree )
{
    ledcWrite(channel1, calculatePWM(degree)); // 输出PWM
}

void motor2Run( int degree )
{
    ledcWrite(channel2, calculatePWM(degree)); // 输出PWM
}

void motor3Run( int degree )
{
    ledcWrite(channel3, calculatePWM(degree)); // 输出PWM
}

void motor4Run( int degree )
{
    ledcWrite(channel4, calculatePWM(degree)); // 输出PWM
}
