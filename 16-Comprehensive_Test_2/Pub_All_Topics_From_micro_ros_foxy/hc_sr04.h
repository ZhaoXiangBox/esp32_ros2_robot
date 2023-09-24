/************************
  HC_SR04 接口定义：
    VCC: 5.0V
    GND: 0V
    Trig:超声波使能引脚
    Echo:超声波回波接收引脚

*************************/

int trigPin = 32;  //发出声波引脚 (ESP32 GPIO32)
int echoPin = 33;  //接收声波引脚 (ESP32 GPIO33)

void Init_HC_SR04() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

unsigned long Read_HC_SR04_Data() {
  
  //低高低电平发一个短时间脉冲去TrigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);                   //打开超声波发射引脚
  delayMicroseconds(10);                         //让该引脚保持 10us的高电平
  digitalWrite(trigPin, LOW);                    //关闭超声波发射引脚

  // 探测等待过长的时间，这里设置了超时保护，相当于减小了 探测距离： 340 （m/s） * 6 (ms) / 1000 (s) / 2.0 = 1.02 (m)
  return (pulseIn(echoPin, HIGH,6000) / 58.0);  //通过传回时间计算距离 cm
}