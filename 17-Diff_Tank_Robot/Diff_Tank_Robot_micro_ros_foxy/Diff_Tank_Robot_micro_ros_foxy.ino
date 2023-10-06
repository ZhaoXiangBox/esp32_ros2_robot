#include "ros_driver.h"

void setup() {
  // Serial.begin(115200);
  Init_Encoder();
  Init_Motors();
  LED_Blink_Init();
  Init_PID();
  Init_ROS();  
}

void loop() 
{
  //Task 1 : PID compute with fixed frequency.
  if (nextmotion <= millis() && moving == 1){
    compute_PID();
  }

  //Task 2 : 自动超时停止保护,底盘将自动停车    
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL){
    setTargetTicksPerFrame(0, 0);
  }

  Control_LED();

  //Task 7 : like ROS1 spin_once, execute ROS message.
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); // 执行一次spin 之后，就在DDS队列中检查新消息或等待计时器准备就绪的的等待配置为1ns，
}
