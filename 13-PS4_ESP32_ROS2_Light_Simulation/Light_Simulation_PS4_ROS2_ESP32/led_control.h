#define Front_LED_White 25
#define Back_LED_Red 26


// LED闪烁计数器变量
long blink_time = 0;
long led_lasttime = 0;

// 自定义结构体： 存储按键按下的次数
struct LIGHT_STATE {

  bool Front_LED_State = false;
  bool Back_LED_State = false;
  bool Blink_Front_Back_State = false;
  bool Close_All_State = false;
};

struct LIGHT_STATE light_state;

void Reset_Light_State()
{
  light_state.Front_LED_State = false;
  light_state.Back_LED_State = false;
  light_state.Blink_Front_Back_State = false;
  light_state.Close_All_State = false;
}


void LED_Blink_Init()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(Front_LED_White, OUTPUT);
  pinMode(Back_LED_Red, OUTPUT);

  // 初始化各个引脚的状态
  digitalWrite(Front_LED_White, LOW);
  digitalWrite(Back_LED_Red, LOW);

  Reset_Light_State();

  long led_lasttime = millis();
}

// 双闪
void Blink()
{
  if ( blink_time % 2 == 0 )
  {
  digitalWrite(Front_LED_White, HIGH);
  digitalWrite(Back_LED_Red, HIGH);
  }
  else if ( blink_time % 2 == 1 )
  {
  digitalWrite(Front_LED_White, LOW);
  digitalWrite(Back_LED_Red, LOW);
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

void Control_LED()
{
  if( light_state.Front_LED_State == true )
  {
    Open_Front_White();
  }

  if( light_state.Back_LED_State == true )
  {
    Open_Back_Red();
  }

  if( light_state.Blink_Front_Back_State == true )
  {
    Blink();
  }

  if( light_state.Close_All_State == true )
  {
    Reset_LED();
    Reset_Light_State();
  }
}


