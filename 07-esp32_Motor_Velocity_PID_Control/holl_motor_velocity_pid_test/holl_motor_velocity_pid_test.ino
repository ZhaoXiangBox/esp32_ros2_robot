#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_v1.h"

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index_ = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;


/* Clear the current command parameters */
void resetCommand() {
  cmd = 0 ; //NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index_ = 0;
}

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
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Init_Motors();
  Init_PID();
  Init_Encoder();

  setTargetTicksPerFrame(0, 0);
}

void loop() 
{
  while (Serial.available() > 0)
  {
    // Read the next character
    chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13)  // '\r'
    {
      if (arg == 1) {
        argv1[index_] = 0 ;//NULL;
      }
      else if (arg == 2) {
        argv2[index_] = 0; //NULL;
      }
      setTargetTicksPerFrame(atoi(argv1),atoi(argv2));
      lastMotorCommand = millis();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (arg == 0) 
        arg = 1;
      else if (arg == 1)
      {
        argv1[index_] = NULL;
        arg = 2;
        index_ = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[index_] = chr;
        index_++;
      }
      else if (arg == 2)
      {
        argv2[index_] = chr;
        index_++;
      }
    }
  }

  //若时间间隔小于规定PID rate，而且已经执行过PID运算了，则会继续进行PID运算
  if (nextmotion <= millis() && moving == 1)
  {
    leftInfo.currentEncoder = readEncoder(LEFT);
    leftInfo.input = leftInfo.currentEncoder - leftInfo.lastEncoder;    //当前采样周期编码值和上次采样周期编码值之间的差值
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
    Serial.print("Left_Encoder_value:");
    Serial.print(leftInfo.input);
    Serial.print(",");
    Serial.print("Target_encoder:");
    Serial.println(leftInfo.target);

    // 用串口绘图仪查看： PID计算的PWM值 和 目标脉冲 之间的关系曲线
    // Serial.print("Left_Encoder_value: ");
    // Serial.print(leftInfo.output);
    // Serial.print(",");
    // Serial.print("Target_encoder: ");
    // Serial.println(leftInfo.target);

    /******** PID 参数调试完毕后 应注释掉上述代码*********/
  }

  /************    自动超时停止保护,底盘将自动停车     *************/
  // if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  // {
  //   setTargetTicksPerFrame(0, 0);
  // }
}
