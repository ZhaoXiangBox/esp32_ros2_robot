/*****************************************************************
Motor Encoder Information: 

  Pluse Per Rotation: 11
  Reduction Ratio: 30
  Motor Voltage: 12V
  Hall Encoder Voltage: 5.0V
  Total pluse per Rotation: 
    11(PPR) * 2(Hall) * 2(CHANGE) * 30(Reduction Ratio) =1320 

*****************************************************************/

#define LEFT_ENCODER_A 4  // Pin for Left Encoder A Pin
#define LEFT_ENCODER_B 5  // Pin for Left Encoder B Pin

#define RIGHT_ENCODER_A 18  // Pin for Right Encoder A Pin
#define RIGHT_ENCODER_B 19  // Pin for Right Encoder B Pin

#define LEFT 0
#define RIGHT 1

long leftPosition = 0;   // Global variable for storing the encoder position
long rightPosition = 0;  // Global variable for storing the encoder position

long current_leftPosition = 0;  // Global variable for storing the current encoder position
long current_rightPosition = 0; // Global variable for storing the current encoder position

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

void Init_Encoder() 
{
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);

  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // Attaching the ISR to encoder Left A B
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), Left_encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), Left_encoder_isr_B, CHANGE);

  // Attaching the ISR to encoder Rigth A B
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), Right_encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), Right_encoder_isr_B, CHANGE);
}