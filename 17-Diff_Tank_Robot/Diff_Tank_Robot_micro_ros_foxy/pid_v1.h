#include <PID_v1.h>

#include "encoder_driver.h"
#include "motor_driver.h"

typedef struct
{
  double target;
  double currentEncoder;
  double lastEncoder;
  double error;
  double input;
  double output;
} PIDInfo;
PIDInfo leftInfo, rightInfo;

//PID参数配置
double Kp_L = 15.0, Ki_L = 30.0, Kd_L = 0.0001;  //2.0 5.0 0.003
double Kp_R = 15.0, Ki_R = 30.0, Kd_R = 0.0001;  //2.0 5.0 0.003

PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp_L, Ki_L, Kd_L, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp_R, Ki_R, Kd_R, DIRECT);

// default is 100 Hz。
double pid_rate = 100.0;                 

// PID每次运算结果的执行时间 10 ms。
double pidinterval = 1000.0 / pid_rate;  

// 记录当前PID执行时的系统时间。
long nextmotion;

// 是否执行 PID 运算的标志位，有目标速度是否为0决定。
int moving;  

// 清零与PID相关的各种变量
void resetPIDInfo() {
  leftInfo.currentEncoder = 0;
  leftInfo.lastEncoder = 0;

  rightInfo.currentEncoder = 0;
  rightInfo.lastEncoder = 0;
}

// PID 初始化函数
void Init_PID() {
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(pidinterval);
  leftPID.SetOutputLimits(-255, 255);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-255, 255);

  resetPIDInfo();
}

// PID 计算函数
void compute_PID() {
  leftInfo.currentEncoder = readEncoder(LEFT);
  leftInfo.input = leftInfo.currentEncoder - leftInfo.lastEncoder;  //当前采样周期编码值和上次采样周期编码值之间的差值
  leftInfo.error = leftInfo.target - leftInfo.input;
  leftPID.Compute();
  leftInfo.lastEncoder = readEncoder(LEFT);

  rightInfo.currentEncoder = readEncoder(RIGHT);
  rightInfo.input = rightInfo.currentEncoder - rightInfo.lastEncoder;
  rightInfo.error = rightInfo.target - rightInfo.input;
  rightPID.Compute();
  rightInfo.lastEncoder = readEncoder(RIGHT);

  setSpeeds(leftInfo.output, rightInfo.output);
  nextmotion = millis() + pidinterval;  // 1000/100

  /*********   PID 调试时使用 正常使用应注释掉 ***********/
  /*********   格式严格 “:” 与“,”不可以注释掉 ***********/

  // 用串口绘图仪查看： 电机实际转动脉冲 和 目标脉冲 之间的曲线图
  // Serial.print("Left_Encoder_value:");
  // Serial.print(leftInfo.input);
  // Serial.print(",");
  // Serial.print("Target_encoder:");
  // Serial.println(leftInfo.target);

  // 用串口绘图仪查看： PID计算的PWM值 和 目标脉冲 之间的曲线图
  // Serial.print("Left_Encoder_value: ");
  // Serial.print(leftInfo.output);
  // Serial.print(",");
  // Serial.print("Target_encoder: ");
  // Serial.println(leftInfo.target);
}
