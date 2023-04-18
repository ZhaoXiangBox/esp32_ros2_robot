#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

struct MPU6050_DATA
{
  // 摄氏度
  double Temperature = 0.0;

  // 沿三轴的线加速度 米每二次方秒 m/s^2
  double Roll = 0.0;
  double Pitch = 0.0;
  double Yaw = 0.0;

  // 沿三轴的线加速度 米每二次方秒 m/s^2
  double Acc_X = 0.0;
  double Acc_Y = 0.0;
  double Acc_Z = 0.0;

  // 三轴角速度 弧度每秒 rad/s
  double Angle_Velocity_R = 0.0;
  double Angle_Velocity_P = 0.0;
  double Angle_Velocity_Y = 0.0;

}mpu6050_data;

void Init_mpu6050()
{
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  /***********************************************
    Available AccelerometerRange:
      MPU6050_RANGE_2_G
      MPU6050_RANGE_4_G
      MPU6050_RANGE_8_G
      MPU6050_RANGE_16_G
  
    API:
      getAccelerometerRange()
      setAccelerometerRange()
  ***********************************************/
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  /***********************************************
    Available GyroRange:
      MPU6050_RANGE_250_DEG
      MPU6050_RANGE_500_DEG
      MPU6050_RANGE_1000_DEG
      MPU6050_RANGE_2000_DEG
  
    API:
      getGyroRange()
      setGyroRange()
  ***********************************************/
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  /***********************************************
    Available Bandwidth:
      MPU6050_BAND_5_HZ
      MPU6050_BAND_10_HZ
      MPU6050_BAND_21_HZ
      MPU6050_BAND_44_HZ
      MPU6050_BAND_94_HZ
      MPU6050_BAND_184_HZ
      MPU6050_BAND_260_HZ

    API:
      getFilterBandwidth()
      setFilterBandwidth()
  ***********************************************/
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}


void ReadMPU6050()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  mpu6050_data.Temperature = temp.temperature;
  
  mpu6050_data.Acc_X = a.acceleration.x;
  mpu6050_data.Acc_Y = a.acceleration.y;
  mpu6050_data.Acc_Z = a.acceleration.z;

  mpu6050_data.Angle_Velocity_R = g.gyro.x; 
  mpu6050_data.Angle_Velocity_P = g.gyro.y; 
  mpu6050_data.Angle_Velocity_Y = g.gyro.z; 
}