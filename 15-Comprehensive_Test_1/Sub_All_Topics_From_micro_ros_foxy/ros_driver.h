#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

#include "led_control.h"
#include "pid_v1.h"

// 自定义圆周率
#define PI 3.1415926

// 死机检测，若时间间隔大于100 ms没有收到指令，则停车
#define AUTO_STOP_INTERVAL 1000  

// 通过宏定义的方式判断各种ROS操作是否执行成功，判断依据是RCL API返回的句柄
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define ARRAY_LEN 4096

rcl_subscription_t twist_subscriber;
rcl_subscription_t Esp32_state_Subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

geometry_msgs__msg__Twist Twist_msg;             // 定义速度话题 Twist 变量
std_msgs__msg__String Esp32_state_msg;           // 定义舵机话题 servos 变量


char test_array[ARRAY_LEN];

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

int into_error_loop_count = 0;

rcl_ret_t rc;

void error_loop() {
  // into_error_loop_count ++;
  // Serial.println(into_error_loop_count);
  // while (1) {
  //   Serial.println("Init ROS Error!");
  // }
  Serial.println("Init ROS Error!");
  Open_Back_Red();
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
  {
    setTargetTicksPerFrame(0,0);
    Servo1_control(MIDDLE_Angle_Value);
    return;
  }
  
  // 阿克曼底盘运动学解算： Twist topic ------>  each wheel velocity.
  double V_left = (1 - ( D*angular_vel) / (2*linear_vel) ) * linear_vel;
  double V_right = (1 + ( D*angular_vel) / (2*linear_vel) ) * linear_vel;
  double servo_angle = atan( (2*angular_vel*L) / (2*linear_vel - D*angular_vel) ) * (180.0/3.1415926);  // 前轮需要转动的角度

  Twist_temp.left_pulse_per_interval = (int) ( V_left * (encoderresolution / (PI * wheel_diameter)) ) / pid_rate;
  Twist_temp.right_pulse_per_interval = (int)( V_right* (encoderresolution / (PI * wheel_diameter)) ) / pid_rate;

  setTargetTicksPerFrame(Twist_temp.left_pulse_per_interval ,Twist_temp.right_pulse_per_interval);
  // 转向角度和舵机PWM的关系转换

  int result_angle = 90 - (int)servo_angle;
  // Serial.print("angle: ");
  // Serial.print(servo_angle);
  // Serial.print(" ");
  // Serial.print("result_angle: ");
  // Serial.println(result_angle);

  Servo1_control(result_angle);
  // Serial.print("V_L: ");
  // Serial.print(Twist_temp.left_pulse_per_interval);
  // Serial.print(" V_L: ");
  // Serial.print(Twist_temp.right_pulse_per_interval);
  // Serial.print(" angle: ");
  // Serial.println(result_angle); 
}

/********************** 协议如下 *****************************
  pit_servo + " " + yaw_servo + " " + 
  Front_light_state + " " + Back_light_state + " " + 
  Blink_light_state + " " + Close_light_state 
************************************************************/
void subs_Esp32_state_callback(const void *msgin){
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  // Serial.println(msg->data.data);

  String Servos_str;
  String Sub_temp_str;
  String Sub_End_str;

  // 字符串按照 空格 拆分
  Servos_str = msg->data.data;
  int length = Servos_str.length();   // 记录字符串的总长度 字节数目

  int index_1 = Servos_str.indexOf(' '); // 第一个空格字符的 标号
  Sub_temp_str = Servos_str.substring(0,index_1);
  esp32_state.Yaw_Servo_Data = 90 + Sub_temp_str.toInt();
  Sub_End_str = Servos_str.substring(index_1+1,length);        // 将第一个空格之后的字符串 拷贝给 sub_End_str （相当于字符串移动了）

  int index_2 = Sub_End_str.indexOf(' ');     
  Sub_temp_str = Sub_End_str.substring(0,index_2);
  esp32_state.Pitch_Servo_Data = 90 + Sub_temp_str.toInt();
  Sub_End_str = Servos_str.substring(index_1 + 1 + index_2 + 1 ,length);

  int index_3 = Sub_End_str.indexOf(' ');
  Sub_temp_str = Sub_End_str.substring(0,index_3);
  esp32_state.Front_light_state = Sub_temp_str.toInt();
  Sub_End_str = Servos_str.substring(index_1 + 1  + index_2 + 1  + index_3 + 1 ,length);

  int index_4 = Sub_End_str.indexOf(' ');
  Sub_temp_str = Sub_End_str.substring(0,index_4);
  esp32_state.Back_light_state = Sub_temp_str.toInt();
  Sub_End_str = Servos_str.substring(index_1 + 1  + index_2 + 1  + index_3 + 1  + index_4 + 1 ,length);

  int index_5 = Sub_End_str.indexOf(' ');
  Sub_temp_str = Sub_End_str.substring(0,index_5);
  esp32_state.Blink_light_state = Sub_temp_str.toInt();
  Sub_End_str = Servos_str.substring(index_1 + 1  + index_2 + 1  + index_3 + 1  + index_4 + 1  + index_5 + 1 ,length);

  esp32_state.Close_light_state = Sub_End_str.toInt();

  // Serial.print("Esp32_state: ");
  // Serial.print(esp32_state.Yaw_Servo_Data );
  // Serial.print(" ");
  // Serial.print(esp32_state.Pitch_Servo_Data);

  // Serial.print(" ");
  // Serial.print(esp32_state.Front_light_state);
  // Serial.print(" ");
  // Serial.print(esp32_state.Back_light_state);
  // Serial.print(" ");
  // Serial.print(esp32_state.Blink_light_state);
  // Serial.print(" ");
  // Serial.println(esp32_state.Close_light_state);
}

// 订阅速度话题 cmd_vel 的回调函数。
void subs_Twist_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  double Velocity = msg->linear.x;
  double Angular = msg->angular.z;  
  Twist_to_pluse_and_Servo(Velocity,Angular);

  // Serial.println("Recv data");
  lastMotorCommand = millis(); // 记录每一次订阅到 topic 的时间。
}

// 初始化ROS节点 及 绑定速度订阅者。
void Init_ROS()
{
  /******************************************************************************
  // Huawei 是WIFI热点的名字
  // 12345678 是WIFI的密码
  // 192.168.27.182 是WIFI热点给笔记本电脑分配的IP
  // 8888 是局域网内ROS2消息传输的端口号，后续在电脑端运行 micro_ros_agent 时候需要加载的参数
  ******************************************************************************/
  set_microros_wifi_transports("Huawei", "12345678", "192.168.183.182", 8888);
  // Serial, not Wifi.
  // set_microros_transports();

  allocator = rcl_get_default_allocator();
  // Serial.println("\n Init ROS 1! ");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create twist 
  // Serial.println(" Init ROS 2! ");
  RCCHECK(rclc_subscription_init_default(&twist_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"cmd_vel"));

  // create esp32_state 
  // Serial.println(" Init ROS 3! ");
  RCCHECK(rclc_subscription_init_default(&Esp32_state_Subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/esp32_state"));

  // init excuter. 1 esp32_state + 1 twist = 2
  // Serial.println(" Init ROS 4! ");
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); 

  // 下面的顺序决定了 executor 消息管理的执行顺序。
  // Serial.println(" Init ROS 5! ");
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &Twist_msg, &subs_Twist_callback, ON_NEW_DATA));

  // Serial.println(" Init ROS 6! ");
  RCCHECK(rclc_executor_add_subscription(&executor, &Esp32_state_Subscriber, &Esp32_state_msg, &subs_Esp32_state_callback, ON_NEW_DATA));

	Esp32_state_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	Esp32_state_msg.data.size = 0;
	Esp32_state_msg.data.capacity = ARRAY_LEN;

  // 设置各个电机目标初始值为 0 0
  setTargetTicksPerFrame(0, 0);
}
