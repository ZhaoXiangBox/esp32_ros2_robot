#include <Arduino.h>

#define Servo_Pin1 2
#define Servo_Pin2 15
#define Servo_Pin3 23

int freq = 50;    // 频率为50Hz,即周期为20ms

// ESP32 一共有16个PWM通道【0，15】，其中前8个为高速PWM（80MHz），后8个为低速PWM（1MHz）
int channel = 8 ;
int channel2 = 9 ;
int channel3 = 10 ;

int resolution = 8;  // 引脚输出的分辨率为 8bit 即范围为【0，255】

int d = 0;

void setup() {

  // 分别设置 每路 PWM 输出的通道、频率、引脚输出时的分辨率
  ledcSetup(channel,freq,resolution);
  ledcAttachPin(Servo_Pin1,channel);

  ledcSetup(channel2,freq,resolution);
  ledcAttachPin(Servo_Pin2,channel2);

  ledcSetup(channel3,freq,resolution);
  ledcAttachPin(Servo_Pin3,channel3);
}

void loop() {
  // 让三路舵机往复运动
  for(d=0;d<255;d++)
  {
    ledcWrite(channel,d);
    ledcWrite(channel2,d);
    ledcWrite(channel3,d);
    delay(10);
  }

  for(d=0; d<255;d++)
  {
    ledcWrite(channel,255-d);
    ledcWrite(channel2,255-d);
    ledcWrite(channel3,255-d);
    delay(10);
  }

}
