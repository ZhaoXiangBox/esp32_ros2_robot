#define Front_LED_White 25
#define Back_LED_Red 26

typedef struct {
  int Yaw_Servo_Data = 90;
  int Pitch_Servo_Data = 90;

  bool Front_light_state = false;
  bool Back_light_state  = false;
  bool Blink_light_state = false;
  bool Close_light_state = false;
}ESP32_STATE;
ESP32_STATE esp32_state;

// LED闪烁计数器变量
long blink_time = 0;
long led_lasttime = 0; 

void LED_Blink_Init()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(Front_LED_White, OUTPUT);
  pinMode(Back_LED_Red, OUTPUT);

  // 初始化各个引脚的状态
  digitalWrite(Front_LED_White, LOW);
  digitalWrite(Back_LED_Red, LOW);

  long led_lasttime = millis();
}

// 双闪
void Blink()
{
  if ( blink_time % 2 == 0 )
  {
  digitalWrite(Front_LED_White, HIGH);
  digitalWrite(Back_LED_Red, HIGH);
  Serial.println(" Open led.");
  }
  else if ( blink_time % 2 == 1 )
  {
  digitalWrite(Front_LED_White, LOW);
  digitalWrite(Back_LED_Red, LOW);
  Serial.println(" Close led.");
  }
}

void Open_Front_White()
{
  digitalWrite(Front_LED_White, HIGH);
}

void Close_Front_White()
{
  digitalWrite(Front_LED_White, LOW);
}

void Open_Back_Red()
{
  digitalWrite(Back_LED_Red, HIGH);
}

void Close_Back_Red()
{
  digitalWrite(Back_LED_Red, LOW);
}

void Reset_LED()
{
  Close_Front_White();
  Close_Back_Red();
}

// 根据回调函数更新的的指令 执行具体的灯光操作
void Control_LED()
{
  if( esp32_state.Front_light_state == true )
  {
    Open_Front_White();
  }

  if( esp32_state.Back_light_state == true )
  {
    Open_Back_Red();
  }

  if( esp32_state.Blink_light_state == true )
  {
    Blink();
  }

  if( esp32_state.Close_light_state == true )
  {
    Reset_LED();
  }

  // LED 闪烁 计数器
  if ((millis() - led_lasttime) > 400)
  {
    led_lasttime = millis();
    blink_time ++ ;
    if (blink_time > 2000 )
      blink_time = 0;
  }

}