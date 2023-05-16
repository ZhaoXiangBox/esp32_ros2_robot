#include "servos_driver.h"
#include "ros_driver.h"

void setup() {
  // Serial.begin(115200);
  Init_Servos_Drivers();
  Init_ROS();
}

void loop() {

  Servo_control(temp_servo_data.Yaw_Servo_Data, temp_servo_data.Pitch_Servo_Data);

  RCCHECK(rclc_executor_spin_some(&Yaw_Servo_executor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&Pitch_Servo_executor, RCL_MS_TO_NS(10)));
}
