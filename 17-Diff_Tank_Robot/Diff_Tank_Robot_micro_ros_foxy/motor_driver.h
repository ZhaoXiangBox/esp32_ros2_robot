
// 左边电机转动方向控制位 引脚
#define Back_Left_D1 13
#define Back_Left_D1_B 12

// 右边电机转动方向控制位 引脚
#define Back_Right_D1 27
#define Back_Right_D1_B 14


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