// 左边电机转动方向控制位 引脚
#define Back_Left_D1 12
#define Back_Left_D1_B 13

// 右边电机转动方向控制位 引脚
#define Back_Right_D1 14
#define Back_Right_D1_B 27


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

void Init_Motors()
{
  pinMode(Back_Left_D1, OUTPUT);
  pinMode(Back_Left_D1_B, OUTPUT);

  pinMode(Back_Right_D1, OUTPUT);
  pinMode(Back_Right_D1_B, OUTPUT);

  setSpeeds(0,0);
}