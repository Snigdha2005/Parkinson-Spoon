#include <Wire.h>
#include "Servo.h"
#include "I2Cdev.h"

Servo servo3;
float x = 0;

void setup() {
  servo3.attach(9);
  Wire.begin(9); 
  Wire.onReceive(receiveEvent);
}

void receiveEvent(int bytes) {
  x = Wire.read();    
}

void loop() {
  servo3.write(x);
  delay(10);
}


