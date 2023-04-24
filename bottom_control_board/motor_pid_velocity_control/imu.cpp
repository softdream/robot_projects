#include "imu.h"
#include "uart.h"

MPU6050 mpu(Wire);

void imuInit()
{
  Wire.begin(IMU_SDA, IMU_SCL); // SDA, SCL
  byte status = mpu.begin();

  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while(status != 0){ }

  Serial.println("calibration");
  uart1.print("calibration");
  delay(100);
  mpu.calcOffsets(true, true); // gyro and accelero
  delay(2000);
  Serial.println("done");
  uart1.print("done");
}

ImuData getImuData()
{
    ImuData imu;
    mpu.update();
    imu.ax = mpu.getAccX();
    imu.ay = mpu.getAccY();
    imu.gz = mpu.getGyroZ();

    return imu;
}
