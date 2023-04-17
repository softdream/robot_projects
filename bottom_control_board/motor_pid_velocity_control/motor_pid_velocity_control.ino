#include "motor.h"
#include "imu.h"
#include "ultrasonic.h"
#include "dht11.h"
#include <Ticker.h>

char recv_buff[20];

void setup()
{
  Serial.begin(115200);
  /*motorGpioInit();
  delay(100);
  //imuInit();
  delay(100);
  encoderInit();
  delay(100;
  motorInit();
  delay(100);*/
}

void loop()
{
  // 接收串口发过来的指令，主要是线速度和角速度
  int len = Serial.available();
  if( len > 0 ){
    memset(recv_buff, 0, sizeof(recv_buff));
    Serial.readBytes( recv_buff, len );
    if( len == 8 ){
        Control u;
        memcpy( &u, recv_buff, sizeof(u) );
        cacuRPM( u.v, u.w );
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
    Serial.printf( "sens %d %d %.2f\n", humidity, temperature, distance );
    
    vTaskDelay(100); // 10Hz频率
  } 

  vTaskDelete(NULL);
}
