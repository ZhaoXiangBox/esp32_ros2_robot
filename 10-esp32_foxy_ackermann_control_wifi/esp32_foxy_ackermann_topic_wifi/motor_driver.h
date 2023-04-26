// 左边电机转动方向控制位 引脚
#define Back_Left_D1 12
#define Back_Left_D1_B 13

// 右边电机转动方向控制位 引脚
#define Back_Right_D1 14
#define Back_Right_D1_B 27

// 转向舵机信号引脚
#define Ackermann_Servo 2

// 需要根据的车模 试探出 最大左转弯时候 舵机的信号值，以及该状态下的车轮转向角度(弧度)，测试的时候，取消对下面的一条 Serial.print 注释
#define MAX_Left_Servo_Value 41.3
#define MAX_Left_Angle 22.0

// 需要根据的车模 试探出 最大右转弯时候 舵机的信号值，以及该状态下的车轮转向角度(弧度) 测试的时候，取消对下面的一条 Serial.print 注释
#define MAX_Right_Servo_Value 144.0
#define MAX_Right_Angle 22.0

// 因为在Arduino mega 2560 中，180 度舵机，其角度范围为【0，180】度
// 所以就将舵机初始角度定为 90，并且此时车轮的实际转向角度也为 0 。
#define MIDDLE_Angle_Value 100.0    // 需要根据实际的安装情况，选择前轮转向零位的 PWM 值，default = 90 。
#define MIDDLE_Angle 0.0

// 频率为50Hz,即周期为20ms
int freq = 50;    
// ESP32 一共有16个PWM通道【0，15】，其中前8个为高速PWM（80MHz），后8个为低速PWM（1MHz）
int channel = 8 ;
// 引脚输出的分辨率为 8bit 即范围为【0，255】
int resolution = 8;  

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
*前轮转向角度范围 【MAX_Left_Angle，MAX_Right_Angle 】
*起始位置(前轮转向 0 度)时的舵机信号值：90 
*功能：控制MG996舵机 1 旋转指定的角度；
*     PWM为【20，90】： 车轮向左旋转【0，MAX_Left_Angle】度,小于 0 ；
*     PWM为【90，160】；车轮向右旋转【0，MAX_Right_Angle】度，大于 0 ；
*     
*参数： current_angle参数范围： [-30,+30] 单位度  
********************************************************************/
void Servo1_control(double current_angle)
{
  // 轮胎转动的角度转 舵机的PWM信号。
  double execute_value = current_angle ;
  double servo_value = 0 ;

  if(execute_value < 0 )
  {
      execute_value = fabs(execute_value);
      servo_value = MIDDLE_Angle_Value - ( ( execute_value / MAX_Left_Angle )* (MIDDLE_Angle_Value - MAX_Left_Servo_Value) ) ;  
  }
  else if( execute_value == 0 )
  {
      servo_value = MIDDLE_Angle_Value ;
  }
  else
  {
      execute_value = fabs(execute_value);
      servo_value = MIDDLE_Angle_Value + ( ( execute_value / MAX_Right_Angle )* (MAX_Right_Servo_Value - MIDDLE_Angle_Value ) ) ;  
  }

  // 软件限位 保护转向拉杆 
  if(servo_value < MAX_Left_Servo_Value )
  {
    servo_value = MAX_Left_Servo_Value ;
  }
  else if( servo_value > MAX_Right_Servo_Value )
  {
    servo_value = MAX_Right_Servo_Value ;
  }

  // 最终舵机执行转动角度
  ledcWrite(channel,servo_value);

  // 测试实车 的 舵机转向 PWM 值 和对应的 角度值，测完毕后需要注释掉该句话。
  //  Serial.println(servo_value);
}

void init_Servo_motor()
{
  // 安装舵机的时候，需要让其处在正中间的位置后再固定拉杆，
  // 最后带动转向拉杆实现左右转向
  // 分别设置 每路 PWM 输出的通道、频率、引脚输出时的分辨率
  ledcSetup(channel,freq,resolution);
  ledcAttachPin(Ackermann_Servo,channel);
  ledcWrite(channel,90);
}

// 电机舵机初始化函数
void Init_Motors()
{
  // 分别设置电机逻辑控制引脚的状态为 输出
  pinMode(Back_Left_D1, OUTPUT);
  pinMode(Back_Left_D1_B, OUTPUT);
  pinMode(Back_Right_D1, OUTPUT);
  pinMode(Back_Right_D1_B, OUTPUT);
  pinMode(Ackermann_Servo,OUTPUT);

  setSpeeds(0,0);
}