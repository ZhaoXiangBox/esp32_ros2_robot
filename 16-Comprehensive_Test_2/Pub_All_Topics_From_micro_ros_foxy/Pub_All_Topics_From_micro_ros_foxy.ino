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
  Init_HC_SR04();
  Init_mpu6050();
  GPS_BDS_Init();
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
    Servo1_control(MIDDLE_Angle_Value);
  }

  //Task 3 : Control 2 dof servo.
  Servo_control(esp32_state.Yaw_Servo_Data, esp32_state.Pitch_Servo_Data);

  //Task 4 : Control Front && Rear LED
  Control_LED();

  //Task 5 : Get GPS data && analyse
  gpsRead();	     //获取GPS数据
	parseGpsBuffer();//解析GPS数据

  //Task 6 : Get mpu6050 Data.
  ReadMPU6050();

  //Task 7 : like ROS1 spin_once, execute ROS message.
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); // 执行一次spin 之后，就在DDS队列中检查新消息或等待计时器准备就绪的的等待配置为1ns，
}
