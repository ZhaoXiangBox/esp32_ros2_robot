#include <PID_v1.h>

#define PI 3.1415926
#define AUTO_STOP_INTERVAL 1000                // 死机检测，若时间间隔大于1s没有收到指令，则停车

typedef struct
{
  double target;
  double currentEncoder;
  double lastEncoder;
  double error;
  double input;
  double output;
}PIDInfo;
PIDInfo leftInfo, rightInfo;


//车轮配置
/*****************************  第一步修改 电机外输出轴 转动一圈 所输出的总脉冲数  ************
   由于是采用的中断方式捕获电机的霍尔脉冲，并且使用的是边沿触发方式，所以电机的编码值计算方法如下：
    encoder = （边沿触发）2 x 霍尔编码器相数量（如：2） x 霍尔编码器线束 (如 13 ) x 电机减速比 (如：30)/
   
   更新车轮间距、车轮直径等参数，需要自行核对
************************************************************************************/
double encoderresolution = 1320.0;   //编码器输出脉冲数/圈 2*2*11*30 = 1320
double wheel_diameter = 0.065;       // 轮胎直径 m

/*****************************  第四步修改 PID 参数，优化电机的调速性能  *********/
//PID参数配置
double Kp_L = 15.0, Ki_L = 30.0, Kd_L = 0.0001;   //2.0 5.0 0.003
double Kp_R = 15.0, Ki_R = 30.0, Kd_R = 0.0001;   //2.0 5.0 0.003

double Sum_count_L = 0;
double Sum_count_R = 0;


PID leftPID( &leftInfo.input,  &leftInfo.output,  &leftInfo.target,  Kp_L, Ki_L, Kd_L, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp_R, Ki_R, Kd_R, DIRECT);

double pid_rate = 100.0;                    // default is 100 Hz
double pidinterval = 1000.0 / pid_rate;    // PID每次运算结果的执行时间 10 ms
long nextmotion;
int  moving;                               // 是否执行 PID 运算的标志位

long lastMotorCommand = AUTO_STOP_INTERVAL;
long led_lasttime;

// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

void resetPIDInfo()
{
  leftInfo.currentEncoder = 0;
  leftInfo.lastEncoder = 0;

  rightInfo.currentEncoder = 0;
  rightInfo.lastEncoder = 0;
}

void Init_PID()
{
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(pidinterval);
  leftPID.SetOutputLimits(-255, 255);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-255, 255);
  led_lasttime = millis();

  resetPIDInfo();
}

