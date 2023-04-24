#include "motor.h"
#include "imu.h"
#include "ultrasonic.h"
#include "dht11.h"
#include "uart.h"
#include "battery.h"
#include <Ticker.h>

char recv_buff[20];

void setup()
{
  Serial.begin(115200);
  uartInit(230400);
  
  motorGpioInit();
  delay(100);
  imuInit();
  delay(100);
  encoderInit();
  delay(100);
  motorInit();
  delay(100);

  // 创建任务：获取各个传感器的值
  xTaskCreate(taskSensors,         // 任务函数 
              "sensors",     //  任务名 
              8*1024,            // 任务栈大小，根据需要自行设置
              NULL,              // 参数，入参为空 
              1,                 // 优先级 
              NULL);  // 任务句柄 
}

void loop()
{
  // 接收串口发过来的指令，主要是线速度和角速度
  int len = uart1.available();
  if( len > 0 ){
    memset(recv_buff, 0, sizeof(recv_buff));
    uart1.readBytes( recv_buff, len );
    if( len == 8 ){
        Control u;
        memcpy( &u, recv_buff, sizeof(u) );
        cacuRPM( u.v, u.w );
        Serial.printf( "control : %.2f %.2f\n", u.v, u.w );
    }
  }
  delay(100);
}

// 用来获取传感器数据的线程
void taskSensors(void *parameter)
{
  ultraSonicInit();
  delay(100);
  
  while(1){
    int humidity = 0;
    int temperature = 0;
    getHumidityAndTemperature( humidity, temperature );

    float distance = getDistance();

    float voltage = getBatteryVoltage();

    uart1.printf( "sens %d %d %.2f %.2f\n", humidity, temperature, distance, voltage );
    
    vTaskDelay(100); // 10Hz频率
  } 

  vTaskDelete(NULL);
}
