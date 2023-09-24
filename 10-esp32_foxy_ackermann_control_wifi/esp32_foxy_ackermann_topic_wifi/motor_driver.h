#include <ESP32Servo.h>

// 左边电机转动方向控制位 引脚
#define Back_Left_D1 13
#define Back_Left_D1_B 12

// 右边电机转动方向控制位 引脚
#define Back_Right_D1 27
#define Back_Right_D1_B 14

// 转向舵机信号引脚
#define Ackermann_Servo 2

// 需要根据的车模 试探出 最大左转弯时候 舵机的信号值，以及该状态下的车轮转向角度(弧度)，测试的时候，取消对下面的一条 Serial.print 注释
#define MAX_Left_Servo_Value 35
#define MAX_Left_Angle 22.0

// 需要根据的车模 试探出 最大右转弯时候 舵机的信号值，以及该状态下的车轮转向角度(弧度) 测试的时候，取消对下面的一条 Serial.print 注释
#define MAX_Right_Servo_Value 145.0
#define MAX_Right_Angle 22.0

// 因为在Arduino mega 2560 中，180 度舵机，其角度范围为【0，180】度
// 所以就将舵机初始角度定为 90，并且此时车轮的实际转向角度也为 0 。
#define MIDDLE_Angle_Value 90.0    // 需要根据实际的安装情况，选择前轮转向零位的 PWM 值，default = 90 。
#define MIDDLE_Angle 0.0

Servo servo1;

int minUs = 1000;
int maxUs = 2000;

// 电机控制函数 分别通过A4950 和舵机执行对应的电机 舵机指令。
void setSpeeds(int m1Speed, int m2Speed)
{
  // 控制左侧电机
  if(m1Speed > 0)
  {
    analogWrite(Back_Left_D1, m1Speed);
    analogWrite(Back_Left_D1_B, LOW);
  }
  else
  {
    analogWrite(Back_Left_D1, LOW);
    analogWrite(Back_Left_D1_B, -m1Speed);  
  }
  
  // 控制右侧电机
  if(m2Speed > 0)
  {
    analogWrite(Back_Right_D1, m2Speed);
    analogWrite(Back_Right_D1_B, LOW);  
  }
  else
  {
    analogWrite(Back_Right_D1, LOW);
    analogWrite(Back_Right_D1_B, -m2Speed); 
  }
}

/********************************************************************
*函数名： Servo1_control
*MG996舵机位置：小车前面控制转向的舵机
*MG996舵机信号范围：【MAX_Left_Servo_Value，MAX_Right_Servo_Value】。
*起始位置(前轮转向 0 度)时的舵机信号值：90 
*功能：控制MG996舵机 1 旋转指定的角度；
*     degree: 舵机旋转的角度。
********************************************************************/
void Servo1_control(int degree) //定义函数用于输出PWM的占空比
{
  if (degree < MAX_Left_Servo_Value)
    degree = MAX_Left_Servo_Value;
  if (degree > MAX_Right_Servo_Value)
    degree = MAX_Right_Servo_Value;

  servo1.write(degree);

  // 测试实车 的 舵机转向 PWM 值 和对应的 角度值，测完毕后需要注释掉该句话。
  // Serial.print("degree: ");
  // Serial.println(degree);

}

void init_Servo_motor()
{
  // 安装舵机的时候，需要让其处在正中间的位置后再固定拉杆，
  servo1.setPeriodHertz(50);
  servo1.attach(Ackermann_Servo,minUs,maxUs);

  Servo1_control(MIDDLE_Angle_Value);                  // 初始舵机角度居中

}

// 电机舵机初始化函数
void Init_Motors()
{
  // 分别设置电机逻辑控制引脚的状态为 输出
  pinMode(Back_Left_D1, OUTPUT);
  pinMode(Back_Left_D1_B, OUTPUT);
  pinMode(Back_Right_D1, OUTPUT);
  pinMode(Back_Right_D1_B, OUTPUT);

  setSpeeds(0,0);
}