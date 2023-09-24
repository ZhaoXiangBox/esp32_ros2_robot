#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>

#include "led_control.h"
#include "pid_v1.h"

#include "hc_sr04.h"
#include "mpu6050.h"
#include "ATGM336H_GPS.h"

// 自定义圆周率
// #define PI 3.1415926

// 死机检测，若时间间隔大于100 ms没有收到指令，则停车
#define AUTO_STOP_INTERVAL 1000  

// 通过宏定义的方式判断各种ROS操作是否执行成功，判断依据是RCL API返回的句柄
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define ARRAY_LEN 4096

rcl_subscription_t twist_subscriber;
rcl_subscription_t Esp32_state_Subscriber;

rcl_publisher_t imu6050_publisher;
rcl_publisher_t gps_publisher;
rcl_publisher_t hc_sr04_publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t Ultra_timer;
rcl_timer_t Imu_timer;
rcl_timer_t Gps_timer;

// 定义订阅的话题： 速度 、 状态信息
geometry_msgs__msg__Twist Twist_msg;             
std_msgs__msg__String Esp32_state_msg;           

// 定义发布的话题：gps 、 超声波 、 姿态传感器
sensor_msgs__msg__NavSatFix gps_msg;
sensor_msgs__msg__Range ultrasonar_msg;
sensor_msgs__msg__Imu Imu_msg;

// 定时器回调函数执行的间隔时间 ms
const unsigned int Ultra_timer_timeout = 100;    // 10 Hz
const unsigned int Imu_timer_timeout = 50;       // 20 Hz
const unsigned int Gps_timer_timeout = 200;      // 5 Hz

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
  // Serial.println("Init ROS Error!");
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

// 定时器 回调函数 ，可根据设置的时间 定时发布topic数据
void Ultra_timer_callback(rcl_timer_t * Ultra_timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (Ultra_timer != NULL) {
    // update HC_SR04 data && publish.
    ultrasonar_msg.header.frame_id.data = (char*)"ultrasonar";
    ultrasonar_msg.min_range = 0.01;
    ultrasonar_msg.max_range = 1.02;
    unsigned long Temp = Read_HC_SR04_Data();
    ultrasonar_msg.range = (double)( Temp / 100.0);
    RCSOFTCHECK(rcl_publish(&hc_sr04_publisher, &ultrasonar_msg, NULL));
    // Serial.print(" Ultrasonar Dis :");
    // Serial.println(Read_HC_SR04_Data());
  }
}

void Imu_timer_callback(rcl_timer_t * Imu_timer, int64_t last_call_time)
{ 
  RCLC_UNUSED(last_call_time);
  if (Imu_timer != NULL) {
    Imu_msg.header.frame_id.data = (char*)"mpu6050";

    Imu_msg.orientation.x = q[1];
    Imu_msg.orientation.y = q[2];
    Imu_msg.orientation.z = q[3];
    Imu_msg.orientation.w = q[0];

    Imu_msg.angular_velocity.x = 0.0;
    Imu_msg.angular_velocity.y = 0.0;
    Imu_msg.angular_velocity.z = 0.0;

    Imu_msg.linear_acceleration.x = 0.0; 
    Imu_msg.linear_acceleration.y = 0.0; 
    Imu_msg.linear_acceleration.z = 0.0; 

    RCSOFTCHECK(rcl_publish(&imu6050_publisher, &Imu_msg, NULL));
  }
}

void Gps_timer_callback(rcl_timer_t * Gps_timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if ((Gps_timer != NULL) && (Save_Data.isUsefull)) {
    double latitude_temp  = (Save_Data.N_S[0] == 'N') ? atof(Save_Data.latitude)  : -atof(Save_Data.latitude)  ;
    double longitude_temp = (Save_Data.E_W[0] == 'E') ? atof(Save_Data.longitude) : -atof(Save_Data.longitude) ;

    gps_msg.header.frame_id.data = (char*)"gps";
    gps_msg.latitude = latitude_temp / 100.0 ;     // 纬度 默认 北纬是正  (度.分秒)
    gps_msg.longitude = longitude_temp / 100.0;    // 经度 默认 东经是正  (度.分秒)

    RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
  }
}

// 初始化ROS节点 及 绑定速度订阅者。
void Init_ROS()
{
  /******************************************************************************
  // Huawei 是WIFI热点的名字
  // 12345678 是WIFI的密码
  // 192.168.149.182 是WIFI热点给笔记本电脑分配的IP
  // 8888 是局域网内ROS2消息传输的端口号，后续在电脑端运行 micro_ros_agent 时候需要加载的参数
  ******************************************************************************/
  set_microros_wifi_transports((char*)"Huawei", (char*)"12345678", (char*)"192.168.223.182", 8888);
  // Serial, not Wifi.
  // set_microros_transports();

  allocator = rcl_get_default_allocator();
  // Serial.println("\n Init ROS 1! ");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create twist esp32_state subscriber
  // Serial.println(" Init ROS 2! ");
  RCCHECK(rclc_subscription_init_default(&twist_subscriber,      &node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&Esp32_state_Subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,      msg, String),"/esp32_state"));

  // create gps hc_sr04 imu publisher
  // Serial.println(" Init ROS 3! ");
  RCCHECK(rclc_publisher_init_best_effort(&imu6050_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),      "mpu6050"));
  RCCHECK(rclc_publisher_init_best_effort(&gps_publisher,    &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),"gps"));
  RCCHECK(rclc_publisher_init_best_effort(&hc_sr04_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),    "ultrasonar"));

  // create timer
  RCCHECK(rclc_timer_init_default(&Imu_timer,  &support,RCL_MS_TO_NS(Imu_timer_timeout),  Imu_timer_callback));
  RCCHECK(rclc_timer_init_default(&Ultra_timer,&support,RCL_MS_TO_NS(Ultra_timer_timeout),Ultra_timer_callback));
  RCCHECK(rclc_timer_init_default(&Gps_timer,  &support,RCL_MS_TO_NS(Gps_timer_timeout),  Gps_timer_callback));

  // init excuter. 1 esp32_state + 1 twist = 2
  // init excuter. 1 esp32_state + 1 twist + 1 gps + 1 hc_sr04 + 1 imu_ + 1 Ultra_timer + 1 Imu_timer + 1 Gps_timer = 8
  // Serial.println(" Init ROS 4! ");
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));

  // 下面的顺序决定了 executor 消息管理的执行顺序。
  // add subscribtion twist && esp32_state.
  // Serial.println(" Init ROS 5! ");
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber,       &Twist_msg,       &subs_Twist_callback,       ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &Esp32_state_Subscriber, &Esp32_state_msg, &subs_Esp32_state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &Ultra_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &Imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &Gps_timer));

  // init String type.
	Esp32_state_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	Esp32_state_msg.data.size = 0;
	Esp32_state_msg.data.capacity = ARRAY_LEN;

  // 设置各个电机目标初始值为 0 0
  setTargetTicksPerFrame(0, 0);
}
