#include "servos_driver.h"
#include "ros_driver.h"

void setup() {
  // Serial.begin(115200);
  Init_Servos_Drivers();
  Init_Encoder();
  Init_Motors();
  init_Servo_motor();
  LED_Blink_Init();
  Init_PID();
  Init_ROS();  
}

void loop() {
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

  // control 2 dof servo.
  Servo_control(esp32_state.Yaw_Servo_Data, esp32_state.Pitch_Servo_Data);
  Control_LED();

  // like ROS1 spin_once，执行一次spin 之后，就在DDS队列中检查新消息或等待计时器准备就绪的的等待配置为1ns，
  // Serial.println(" Init ROS cyc! ");
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
