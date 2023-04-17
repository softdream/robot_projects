#include "imu.h"

MPU6050 mpu(Wire);

void imuInit()
{
  Wire.begin(IMU_SDA, IMU_SCL); // SDA, SCL
  byte status = mpu.begin();

  while(status != 0){ }

  Serial.println("calibration");
  delay(100);
  mpu.calcOffsets(true, true); // gyro and accelero
  delay(2000);
  Serial.println("done");
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
