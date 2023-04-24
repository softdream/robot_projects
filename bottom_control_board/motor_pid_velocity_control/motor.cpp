#include "motor.h"
#include "imu.h"
#include "uart.h"

float required_rpm_left = 0;
float required_rpm_right = 0; 

// ---------------------- 编码器 --------------------- //
volatile long l_count = 0; // 左轮脉冲计数
volatile long r_count = 0; // 右轮脉冲计数

Ticker control_timer; // 用于对电机进行控制的定时器

PID pid_l( 0.05, 255, -255, 7, 0.4, 0 );
PID pid_r( 0.05, 255, -255, 7, 0.5, 0 );

// 在外部中断中处理编码器数据
void leftEncoder()
{
  if( digitalRead(LA) && !digitalRead(LB) ){ // A相上升沿并且B相为低电平时为正转
    l_count ++;
  }
  else if( !digitalRead(LA) && digitalRead(LB) ){ // A相下降沿并且B相为高电平时为正转
    l_count ++;
  }
  else { // 其他情况就是反转
    l_count --;   
  }
  //Serial.println(l_count);
}

void rightEncoder()
{
  if( digitalRead(RA) && !digitalRead(RB) ){ // A相上升沿并且B相为低电平时为正转
    r_count ++;
  }
  else if( !digitalRead(RA) && digitalRead(RB) ){ // A相下降沿并且B相为高电平时为正转
    r_count ++;
  }
  else { // 其他情况就是反转
    r_count --;   
  }
  //Serial.println(r_count);
}

void encoderInit()
{
  pinMode(LA, INPUT); 
  pinMode(LB, INPUT); 
  pinMode(RA, INPUT); 
  pinMode(RB, INPUT);
  
  attachInterrupt(LA, leftEncoder, CHANGE);//设置相位边沿触发中断
  attachInterrupt(RA, rightEncoder, CHANGE);

  interrupts(); //打开外部中断 

  uart1.print("encoder");
}

int rpmToPWM(float rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
 return ((rpm / MAX_RPM) * 255);
}

void setPwm( int motor1_pwm, int motor2_pwm )
{
  // 电机1 正方向
  if( motor1_pwm > 0 ){
    digitalWrite( IN1, LOW );
    digitalWrite( IN2, HIGH );
    ledcWrite( pwm_Channel_A, abs(motor1_pwm) );
  }
  else { // 反方向
    digitalWrite( IN1, HIGH );
    digitalWrite( IN2, LOW );
    ledcWrite( pwm_Channel_A, abs(motor1_pwm) );
  }

  // 电机2 正方向
  if( motor2_pwm > 0 ){
    digitalWrite( IN3, LOW );
    digitalWrite( IN4, HIGH );
    ledcWrite( pwm_Channel_B, abs(motor2_pwm) );
  }
  else { // 反方向
    digitalWrite( IN3, HIGH );
    digitalWrite( IN4, LOW );
    ledcWrite( pwm_Channel_B, abs(motor2_pwm) );
  }
}

float getVelocity( float motor1_rpm, float motor2_rpm )
{
  float motor1_rps = motor1_rpm / 60; // rpm -> rps
  float motor2_rps = motor2_rpm / 60; // rpm -> rps

  float l_x = motor1_rps * circumference; // 左轮速度
  float l_y = motor2_rps * circumference; // 右轮速度

  //Serial.printf("velocity : %f\n", (l_x + l_y) / 2);

  return ( l_x + l_y ) * 0.5; // 前进速度
}

float getDeltaS( long l_count_c, long r_count_c )
{
  float l_s = ((float)l_count_c / pulse_number) * circumference;
  float r_s = -((float)r_count_c / pulse_number) * circumference;

  //Serial.printf("delta s : %f\n", (l_s + r_s) / 2);
  return (l_s + r_s) / 2;
}

float getDeltaAngle( long l_count_c, long r_count_c )
{
  float l_s = ((float)l_count_c / pulse_number) * circumference;
  float r_s = -((float)r_count_c / pulse_number) * circumference;

  return (r_s - l_s) / base_width;
}

// 定时器中断函数, 频率20Hz
void motorControl()
{    
  float l_rpm = ( 60 * l_count / ( pulse_number * 0.05 ) ); // 
  float r_rpm = -60 * r_count / ( pulse_number * 0.05 );  // 这里正负号要修改，适应车身的前进方向

  //Serial.printf("left  rpm : %f\n", l_rpm);
  //Serial.printf("right rpm : %f\n", r_rpm);

  // 得到测量值
  float velocity = getVelocity( l_rpm, r_rpm );
  float delta_s = getDeltaS( l_count, r_count );
  float delta_angle = getDeltaAngle( l_count, r_count );
  ImuData imu = getImuData();

  // 发送测量值到上位机
  uart1.printf( "meas %ld %.4f %.4f %.4f %.4f %.4f %.4f\n", millis(), velocity, delta_s, delta_angle, imu.gz, l_rpm, r_rpm );
  //Serial.printf( "meas %ld %.2f %.2f %.2f %.2f %.2f %.2f\n", millis(), velocity, delta_s, delta_angle, imu.gz, l_rpm, r_rpm );
  
  // reset encoder count
  l_count = 0;
  r_count = 0;

  // pid control
  float l_out = pid_l.caculate( required_rpm_left, l_rpm );
  float r_out = pid_r.caculate( required_rpm_right, r_rpm );
 
  //setPwm( rpmToPWM(required_rpm_left), rpmToPWM(required_rpm_right) ); 
  setPwm( l_out, r_out ); 
}

void motorGpioInit()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite( IN1, LOW );
  digitalWrite( IN2, LOW );
  digitalWrite( IN3, LOW );
  digitalWrite( IN4, LOW );
  digitalWrite( PWMA, LOW );
  digitalWrite( PWMB, LOW );
}

void motorInit()
{
  // 电机1
  ledcSetup(pwm_Channel_A, FREQ, resolution);
  ledcAttachPin(PWMA, pwm_Channel_A);
  ledcWrite(pwm_Channel_A, 0);        //PWM通道一占空比设置为零

  // 电机2
  ledcSetup(pwm_Channel_B, FREQ, resolution);
  ledcAttachPin(PWMB, pwm_Channel_B);
  ledcWrite(pwm_Channel_B, 0);

  // 打开定时器中断
  control_timer.attach_ms( interrupt_time_control, motorControl );
  
  uart1.print("motor");
}

void cacuRPM( float v, float w )
{
  //convert m/s to m/min
  float v_mins = v * 60;
  
  //convert rad/s to rad/min
  float w_mins = w * 60;

  //Vt = ω * radius
  float w_vel = w_mins * base_width;

  float x_rpm = v_mins / circumference;
  float tan_rpm = w_vel / circumference;

  required_rpm_left = x_rpm - tan_rpm * 0.5;
  required_rpm_right = x_rpm + tan_rpm * 0.5;
}
