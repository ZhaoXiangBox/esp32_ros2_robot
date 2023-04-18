#include "mpu6050.h"

void setup() {
  Serial.begin(115200);
  Init_mpu6050();
}

void loop() {

  ReadMPU6050();
  // 串口绘图仪 可视化线加速度曲线
  // Serial.print("Acc_x:");
  // Serial.print(mpu6050_data.Acc_X);
  // Serial.print(",");
  // Serial.print("Acc_Y:");
  // Serial.print(mpu6050_data.Acc_Y);
  // Serial.print(",");
  // Serial.print("Acc_Z:");
  // Serial.println(mpu6050_data.Acc_Z);

  // 串口绘图仪 可视化角速度曲线
  Serial.print("Angle_velocity_R:");
  Serial.println(mpu6050_data.Angle_Velocity_R);
  Serial.print(",");
  Serial.print("Angle_velocity_P:");
  Serial.print(mpu6050_data.Angle_Velocity_P);
  Serial.print(",");
  Serial.print("Angle_velocity_Y:");
  Serial.println(mpu6050_data.Angle_Velocity_Y);

  delay(50);

}
