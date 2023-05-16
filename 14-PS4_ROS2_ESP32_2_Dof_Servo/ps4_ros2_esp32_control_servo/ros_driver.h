#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>

// 通过宏定义的方式判断各种ROS操作是否执行成功，判断依据是RCL API返回的句柄
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


rcl_subscription_t Yaw_Servo_Subscriber;
rcl_subscription_t Pitch_Servo_Subscriber;

rclc_executor_t Yaw_Servo_executor;
rclc_executor_t Pitch_Servo_executor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 定义速度话题 servo 变量
std_msgs__msg__Int8 Yaw_Servo_msg;
std_msgs__msg__Int8 Pitch_Servo_msg;


struct Servos{
  int Yaw_Servo_Data = 0;
  int Pitch_Servo_Data = 0;
};

struct Servos temp_servo_data;

void error_loop() {
  while (1) {
    delay(100);
    // Serial.println("Init ROS Error!");
  }
}

void subs_Yaw_Servo_callback(const void *msgin) {
  const std_msgs__msg__Int8 * yaw_msg = (const std_msgs__msg__Int8 *)msgin;
  temp_servo_data.Yaw_Servo_Data = 90 + (int)yaw_msg->data;
  // Serial.print("yaw: ");
  // Serial.println(temp_servo_data.Yaw_Servo_Data);
}

void subs_Pitch_Servo_callback(const void *msgin) {
  const std_msgs__msg__Int8 * pitch_msg = (const std_msgs__msg__Int8 *)msgin;
  temp_servo_data.Pitch_Servo_Data = 90 + (int)pitch_msg->data;

  // Serial.print("Pitch: ");
  // Serial.println(temp_servo_data.Pitch_Servo_Data);
}

// 初始化ROS节点 及 绑定速度订阅者。
void Init_ROS()
{
  /******************************************************************************
  // set_microros_wifi_transports("Huawei", "12345678", "192.168.27.182", 8888)的意义
  // Huawei 是WIFI热点的名字
  // 12345678 是WIFI的密码
  // 192.168.27.182 是WIFI热点给笔记本电脑分配的IP
  // 8888 是局域网内ROS2消息传输的端口号，后续在电脑端运行 micro_ros_agent 时候需要加载的参数
  ******************************************************************************/
  set_microros_wifi_transports("Huawei", "12345678", "192.168.81.182", 8888);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create Servos_subscriber
  RCCHECK(rclc_subscription_init_default(
    &Yaw_Servo_Subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/servo1"));

  RCCHECK(rclc_subscription_init_default(
    &Pitch_Servo_Subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/servo2"));

  // create executor.
  RCCHECK(rclc_executor_init(&Yaw_Servo_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&Pitch_Servo_executor, &support.context, 1, &allocator));

  // bind callback function.
  RCCHECK(rclc_executor_add_subscription(&Yaw_Servo_executor,   &Yaw_Servo_Subscriber,    &Yaw_Servo_msg,    &subs_Yaw_Servo_callback,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&Pitch_Servo_executor, &Pitch_Servo_Subscriber , &Pitch_Servo_msg , &subs_Pitch_Servo_callback,  ON_NEW_DATA));

}
