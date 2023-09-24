#include <ESP32Servo.h>

Servo Yaw_Servo;
Servo Pitch_Servo;

int minUs = 1000;
int maxUs = 2000;

#define Yaw_Servo_Pin 15
#define Pitch_Servo_Pin 23

#define Min_Servo_Value 0
#define Max_Servo_Value 180

#define Pitch_Min_Servo_Value 65
#define Pitch_Max_Servo_Value 145

#define Init_Angle_Value 90.0    // 需要根据实际的安装情况，选择前轮转向零位的 PWM 值，default = 90 。

int Check_PWM_Range(int Range_PWM)
{
  if (Range_PWM < Min_Servo_Value)
    Range_PWM = Min_Servo_Value;
  if (Range_PWM > Max_Servo_Value)
    Range_PWM = Max_Servo_Value;
  
  return Range_PWM;
}

void Servo_control(int Yaw_Degree, int Pitch_degree) //定义函数用于输出PWM的占空比
{
  Yaw_Degree = Check_PWM_Range(Yaw_Degree);
  Pitch_degree = Check_PWM_Range(Pitch_degree);


  if (Pitch_degree < Pitch_Min_Servo_Value)
    Pitch_degree = Pitch_Min_Servo_Value;
  if (Pitch_degree > Pitch_Max_Servo_Value)
    Pitch_degree = Pitch_Max_Servo_Value;

  Yaw_Servo.write(Yaw_Degree);
  Pitch_Servo.write(Pitch_degree);

  // 测试实车 的 舵机转向 PWM 值 和对应的 角度值，测完毕后需要注释掉该句话。
  // Serial.print("degree: ");
  // Serial.print(Yaw_Degree);
  // Serial.print(" ");
  // Serial.println(Pitch_degree);
}

void Init_Servos_Drivers()
{
	Yaw_Servo.setPeriodHertz(50);         // Standard 50hz servo
	Pitch_Servo.setPeriodHertz(50);      // Standard 50hz servo

	Yaw_Servo.attach(Yaw_Servo_Pin, minUs, maxUs);
	Pitch_Servo.attach(Pitch_Servo_Pin, minUs, maxUs);

  Servo_control(Init_Angle_Value,Init_Angle_Value);                  // 初始舵机角度居中
}


void Detach_Servos()
{
  Yaw_Servo.detach();
  Pitch_Servo.detach();
}