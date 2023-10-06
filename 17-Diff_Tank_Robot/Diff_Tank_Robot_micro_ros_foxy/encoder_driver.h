/*****************************************************************
Motor Encoder Information: 

  Pluse Per Rotation: 11
  Reduction Ratio: 30
  Motor Voltage: 12V
  Hall Encoder Voltage: 5.0V
  Total pluse per Rotation: 
    11(PPR) * 2(Hall) * 2(CHANGE) * 30(Reduction Ratio) =1320 

*****************************************************************/

// 左侧电机的 A B 相编码器接口引脚
#define LEFT_ENCODER_A 4  
#define LEFT_ENCODER_B 5  

// 右侧电机的 A B 相编码器接口引脚
#define RIGHT_ENCODER_A 19  
#define RIGHT_ENCODER_B 18  

#define LEFT 0
#define RIGHT 1

// 记录编码器上一次捕获脉冲的累计值
long leftPosition = 0;   
long rightPosition = 0;  

// 记录编码器捕获脉冲的累计值
long current_leftPosition = 0;  
long current_rightPosition = 0; 

// 左侧编码器 A 相的中断服务函数
void Left_encoder_isr_A() {
  // Reading the current state of encoder A and B
  // look for a low-to-high on channel A
  if (digitalRead(LEFT_ENCODER_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(LEFT_ENCODER_B) == LOW) {  
      current_leftPosition = current_leftPosition + 1;         // CW
    } 
    else {
      current_leftPosition = current_leftPosition - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(LEFT_ENCODER_B) == HIGH) {   
      current_leftPosition = current_leftPosition + 1;          // CW
    }
    else {
      current_leftPosition = current_leftPosition - 1;          // CCW
    }
  }
}
// 左侧编码器 B 相的中断服务函数
void Left_encoder_isr_B() {
  // Reading the current state of encoder A and B
  // look for a low-to-high on channel A
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(LEFT_ENCODER_A) == HIGH) {  
      current_leftPosition = current_leftPosition + 1;         // CW
    } 
    else {
      current_leftPosition = current_leftPosition - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(LEFT_ENCODER_A) == LOW) {   
      current_leftPosition = current_leftPosition + 1;          // CW
    } 
    else {
      current_leftPosition = current_leftPosition - 1;          // CCW
    }
  }
}

// 右侧编码器 A 相的中断服务函数
void Right_encoder_isr_A() {
  // Reading the current state of encoder A and B
  // look for a low-to-high on channel A
  if (digitalRead(RIGHT_ENCODER_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(RIGHT_ENCODER_B) == LOW) {  
      current_rightPosition = current_rightPosition + 1;         // CW
    } 
    else {
      current_rightPosition = current_rightPosition - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(RIGHT_ENCODER_B) == HIGH) {   
      current_rightPosition = current_rightPosition + 1;          // CW
    }
    else {
      current_rightPosition = current_rightPosition - 1;          // CCW
    }
  }
}

// 右侧编码器 B 相的中断服务函数
void Right_encoder_isr_B() {
  // Reading the current state of encoder A and B
  // look for a low-to-high on channel A
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(RIGHT_ENCODER_A) == HIGH) {  
      current_rightPosition = current_rightPosition + 1;         // CW
    } 
    else {
      current_rightPosition = current_rightPosition - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(RIGHT_ENCODER_A) == LOW) {   
      current_rightPosition = current_rightPosition + 1;          // CW
    } 
    else {
      current_rightPosition = current_rightPosition - 1;          // CCW
    }
  }
}

// 读取编码器在该段时间内的累计脉冲值
long readEncoder(int i)
{
  if (i==LEFT)
  {
    return (current_leftPosition -leftPosition );
  }
  else
  {
    return (current_rightPosition -rightPosition );
  }
}

// 更新最新的累计编码数值
void resetEncoder(int i)
{
  if (i==LEFT)
  {
    leftPosition=current_leftPosition;    
  }else
  {
    rightPosition=current_rightPosition;
  }
}

void resetEncoders()
{  
  resetEncoder(LEFT);
  resetEncoder(RIGHT); 
}

// 霍尔电机编码器初始化函数
void Init_Encoder() 
{
  resetEncoders();

  // 设置编码器捕获引脚的输入输出状态
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // 绑定左侧编码器的 中断触发方式(边沿)及中断服务函数
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), Left_encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), Left_encoder_isr_B, CHANGE);
  
  // 绑定右侧侧编码器的 中断触发方式(边沿)及中断服务函数
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), Right_encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), Right_encoder_isr_B, CHANGE);
}