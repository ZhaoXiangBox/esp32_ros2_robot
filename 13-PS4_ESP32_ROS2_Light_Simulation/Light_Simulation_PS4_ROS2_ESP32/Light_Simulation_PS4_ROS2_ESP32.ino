#include "ros_driver.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  LED_Blink_Init();
  Init_ROS();
}

void loop() {

  // LED 闪烁 计数器
  if ((millis() - led_lasttime) > 400)
  {
    led_lasttime = millis();
    blink_time ++ ;
    if (blink_time > 2000 )
      blink_time = 0;
  }

  Control_LED();

  // like ROS1 spin_once
  RCCHECK(rclc_executor_spin_some(&Front_LED_executor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&Back_LED_executor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&Blink_LED_executor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&Close_All_executor, RCL_MS_TO_NS(10)));
}
