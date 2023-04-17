#include "hc_sr04.h"

void setup() {
  Serial.begin(9600);
  Init_HC_SR04();
}

void loop() {
  Serial.print(Read_HC_SR04_Data());
  Serial.println(" cm");
  delay(100);
  }