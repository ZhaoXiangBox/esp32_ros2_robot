#include "encoder_driver.h"
#include "motor_driver.h"
#include "ros_driver.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Init_Motors();
  init_Servo_motor();
  Init_PID();
  Init_Encoder();
  Init_ROS();
}

void loop() 
{
  //若时间间隔小于规定PID rate，而且已经执行过PID运算了，则会继续进行PID运算
  if (nextmotion <= millis() && moving == 1){
    compute_PID();
  }

  /************    自动超时停止保护,底盘将自动停车     *************/
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setTargetTicksPerFrame(0, 0);
    Servo1_control(MIDDLE_Angle_Value);
  }

  // like ROS1 spin_once
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
