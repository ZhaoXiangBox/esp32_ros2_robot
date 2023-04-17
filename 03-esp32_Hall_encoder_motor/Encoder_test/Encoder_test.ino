#include "encoder_driver.h"

void setup() {
  Serial.begin(115200);  // Initialize serial communication
  Init_Encoder();
}

void loop() {
  Serial.print("Left Encoder: ");
  Serial.println(readEncoder(LEFT));

  // Serial.print("Left Encoder: ");
  // Serial.println(readEncoder(RIGHT));
}