#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "pid_v1.h"

// 自定义圆周率
#define PI 3.1415926

// 死机检测，若时间间隔大于1s没有收到指令，则停车
#define AUTO_STOP_INTERVAL 2000  

// 通过宏定义的方式判断各种ROS操作是否执行成功，判断依据是RCL API返回的句柄
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t twist_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 定义速度话题 Twist 变量
geometry_msgs__msg__Twist twist_msg;

// 自定义结构体： 存储解算之后的电机、舵机执行目标值
struct {
  int left_pulse_per_interval = 0;
  int right_pulse_per_interval = 0;
  int servo_angle = MIDDLE_Angle_Value;
}Twist_temp;

/*****************************  车轮配置  ********************************************
   由于是采用的中断方式捕获电机的霍尔脉冲，并且使用的是边沿触发方式，所以电机的编码值计算方法如下：
    encoder = （边沿触发）2 x 霍尔编码器相数量（如：2） x 霍尔编码器线束 (如 13 ) x 电机减速比 (如：30)/
   
   更新车轮间距、车轮直径等参数，需要自行核对
************************************************************************************/
double encoderresolution = 1320.0;  //编码器输出脉冲数/圈 2*2*11*30 = 1320
double wheel_diameter = 0.065;      // 轮胎直径 m
double L = 0.15 ; // 前后轮轴距 0.15m
double D = 0.16 ; // 后轮左右轮间距 0.16m

long lastMotorCommand = AUTO_STOP_INTERVAL;  // 间隔 AUTO_STOP_INTERVAL 时间未收到新的指令，车会自己停下来

void error_loop() {
  while (1) {
    delay(100);
  }
}

// 清零自定义的结构体
void Reset_twist_temp()
{
  Twist_temp.left_pulse_per_interval = 0;
  Twist_temp.right_pulse_per_interval = 0;
  Twist_temp.servo_angle = MIDDLE_Angle_Value;
}

// 设置PID运算对象的目标参考值
void setTargetTicksPerFrame(int left, int right)
{
  if (left == 0 && right == 0)
  {
    setSpeeds(0, 0);
    moving = 0;
  }
  else
  {
    moving = 1;
  }
  leftInfo.target = left;
  rightInfo.target = right;
  Reset_twist_temp();
}

// 车辆运动学中心的线角速度 -------->  每个电机和舵机需要执行的目标脉冲数
void Twist_to_pluse_and_Servo(double linear_vel, double angular_vel)
{
  // 阿克曼底盘，无法原地转向，所以需要筛选线速度为0，角速度不为0的 topic 数据。
  if(linear_vel == 0)
    return;
  
  // 阿克曼底盘运动学解算： Twist topic ------>  each wheel velocity.
  double V_left = (1 - ( D*angular_vel) / (2*linear_vel) ) * linear_vel;
  double V_right = (1 + ( D*angular_vel) / (2*linear_vel) ) * linear_vel;
  double servo_angle = atan( (2*angular_vel*L) / (2*linear_vel - D*angular_vel) ) * (180.0/3.1415926);  // 前轮需要转动的角度

  Twist_temp.left_pulse_per_interval = (int) ( V_left * (encoderresolution / (PI * wheel_diameter)) ) / pid_rate;
  Twist_temp.right_pulse_per_interval = (int)( V_right* (encoderresolution / (PI * wheel_diameter)) ) / pid_rate;

  setTargetTicksPerFrame(Twist_temp.left_pulse_per_interval ,Twist_temp.right_pulse_per_interval);
  // 转向角度和舵机PWM的关系转换
  Servo1_control(servo_angle);
  // Serial.print("V_L: ");
  // Serial.print(Twist_temp.left_pulse_per_interval);
  // Serial.print(" V_L: ");
  // Serial.print(Twist_temp.right_pulse_per_interval);
  // Serial.print(" angle: ");
  // Serial.println(servo_angle); 
}

// 订阅速度话题 cmd_vel 的回调函数。
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  double Velocity = msg->linear.x;
  double Angular = msg->angular.z;
  Twist_to_pluse_and_Servo(Velocity,Angular);

  lastMotorCommand = millis(); // 记录每一次订阅到 topic 的时间。
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
  set_microros_wifi_transports("Huawei", "12345678", "192.168.149.182", 8888);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create twist_subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));
  
  // 设置各个电机目标初始值为 0 0 90
  setTargetTicksPerFrame(0, 0);
}
