#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int8.h>

#include "led_control.h"


// 通过宏定义的方式判断各种ROS操作是否执行成功，判断依据是RCL API返回的句柄
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t Front_light_state_subscriber;
rcl_subscription_t Back_light_state_subscriber ;
rcl_subscription_t Blink_light_state_subscriber;
rcl_subscription_t Close_light_state_subscriber;

rclc_executor_t Front_LED_executor;
rclc_executor_t Back_LED_executor;
rclc_executor_t Blink_LED_executor;
rclc_executor_t Close_All_executor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 定义灯光话题 light_state 变量
std_msgs__msg__Int8 Front_light_state_msg;
std_msgs__msg__Int8 Back_light_state_msg ;
std_msgs__msg__Int8 Blink_light_state_msg;
std_msgs__msg__Int8 Close_light_state_msg;

void error_loop() {
  while (1) {
    delay(100);
    Serial.println("Init ROS Error.");
  }
}

// 订阅灯光话题 light_state 的回调函数。
void subs_Front_light_callback(const void *msgin) {
  light_state.Front_LED_State = true;
}
void subs_Back_light_callback(const void *msgin) {
  light_state.Back_LED_State = true;
}
void subs_Blink_light_callback(const void *msgin) {
  light_state.Blink_Front_Back_State = true;
}
void subs_Close_light_callback(const void *msgin) {
  light_state.Close_All_State = true;
}

// 初始化ROS节点 及 绑定灯光订阅者。
void Init_ROS()
{
  /******************************************************************************
  // set_microros_wifi_transports("Huawei", "12345678", "192.168.27.182", 8888)的意义
  // Huawei 是WIFI热点的名字
  // 12345678 是WIFI的密码
  // 192.168.248.182 是WIFI热点给笔记本电脑分配的IP
  // 8888 是局域网内ROS2消息传输的端口号，后续在电脑端运行 micro_ros_agent 时候需要加载的参数
  ******************************************************************************/
  set_microros_wifi_transports("Huawei", "12345678", "192.168.248.182", 8888);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create light_state_subscriber
  RCCHECK(rclc_subscription_init_default(
    &Front_light_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/Front_light_state"));

  RCCHECK(rclc_subscription_init_default(
    &Back_light_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/Back_light_state"));

  // create light_state_subscriber
  RCCHECK(rclc_subscription_init_default(
    &Blink_light_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/Blink_light_state"));

  RCCHECK(rclc_subscription_init_default(
    &Close_light_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/Close_light_state"));

  // create executor.
  RCCHECK(rclc_executor_init(&Front_LED_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&Back_LED_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&Blink_LED_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&Close_All_executor, &support.context, 1, &allocator));

  // bind callback function.
  RCCHECK(rclc_executor_add_subscription(&Front_LED_executor, &Front_light_state_subscriber, &Front_light_state_msg, &subs_Front_light_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&Back_LED_executor, &Back_light_state_subscriber , &Back_light_state_msg , &subs_Back_light_callback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&Blink_LED_executor, &Blink_light_state_subscriber, &Blink_light_state_msg, &subs_Blink_light_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&Close_All_executor, &Close_light_state_subscriber, &Close_light_state_msg, &subs_Close_light_callback, ON_NEW_DATA));

}
