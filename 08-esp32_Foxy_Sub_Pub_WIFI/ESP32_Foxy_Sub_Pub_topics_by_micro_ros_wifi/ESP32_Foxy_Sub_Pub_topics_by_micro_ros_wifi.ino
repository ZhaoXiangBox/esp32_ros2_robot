#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/range.h>


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif

rcl_publisher_t imu_publisher;
rcl_publisher_t gps_publisher;
rcl_publisher_t hc_sr04_publisher;
rcl_subscription_t twist_subscriber;

geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__NavSatFix gps_msg;
sensor_msgs__msg__Range ultrasonar_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;


#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  if(fabs(msg->linear.x > 0) ||fabs(msg->angular.z)>0)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

  Serial.print(msg->linear.x);
  Serial.print("  ");
  Serial.println(msg->angular.z);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);

  /******************************************************************************
  // set_microros_wifi_transports("Huawei", "12345678", "192.168.27.182", 8888)的意义
  // Huawei 是WIFI热点的名字
  // 12345678 是WIFI的密码
  // 192.168.27.182 是WIFI热点给笔记本电脑分配的IP
  // 8888 是局域网内ROS2消息传输的端口号，后续在电脑端运行 micro_ros_agent 时候需要加载的参数
  ******************************************************************************/
  set_microros_wifi_transports("Huawei", "12345678", "192.168.27.182", 8888);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create imu_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "mpu6050_imu"));

  // create gps_publisher
    RCCHECK(rclc_publisher_init_best_effort(
    &gps_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
    "gps"));

  // create hc_sr04_publisher
    RCCHECK(rclc_publisher_init_best_effort(
    &hc_sr04_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonar"));

  // create twist_subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));
 
}

void loop() {

    // update IMU data && publish.
    imu_msg.header.frame_id.data="mpu6050_imu";
    // mpu_msg.header.stamp= ;
    imu_msg.linear_acceleration.x = 0.1;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    imu_msg.angular_velocity.x = 0.1;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // update gps data && publish.
    gps_msg.header.frame_id.data = "gps";
    RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
    
    // update HC_SR04 data && publish.
    ultrasonar_msg.header.frame_id.data = "ultrasonar";
    RCSOFTCHECK(rcl_publish(&hc_sr04_publisher, &ultrasonar_msg, NULL));


    // like ROS1 spin_once
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
